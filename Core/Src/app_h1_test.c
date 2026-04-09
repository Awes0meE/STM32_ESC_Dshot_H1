#include "app_h1_test.h"

#include <stdio.h>
#include <string.h>

#include "adc.h"
#include "gpio.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"

#define H1_ADC_CHANNEL_COUNT              2U
#define H1_ADC_DMA_SAMPLES_PER_CHANNEL    64U
#define H1_ADC_DMA_BUFFER_LENGTH          (H1_ADC_CHANNEL_COUNT * H1_ADC_DMA_SAMPLES_PER_CHANNEL)

#define H1_DSHOT_FRAME_BITS               16U
#define H1_DSHOT_RESET_SLOTS              8U
#define H1_DSHOT_DMA_BUFFER_LENGTH        (H1_DSHOT_FRAME_BITS + H1_DSHOT_RESET_SLOTS)
#define H1_DSHOT_BIT_0_HIGH_TICKS         54U
#define H1_DSHOT_BIT_1_HIGH_TICKS         108U

#define H1_UART_TX_TIMEOUT_MS             500U
#define H1_STATUS_LED_ON_LEVEL            GPIO_PIN_RESET
#define H1_STATUS_LED_OFF_LEVEL           GPIO_PIN_SET
#define H1_BT_CONNECTED_LEVEL             GPIO_PIN_SET

typedef struct
{
    H1_TestState_t state;
    uint32_t test_start_tick;
    uint32_t state_enter_tick;
    uint32_t last_csv_tick;
    uint32_t last_dshot_send_tick;
    uint32_t last_zero_offset_sample_tick;
    uint32_t last_oled_update_tick;
    uint32_t button_change_tick;
    uint32_t bt_high_since_tick;
    uint16_t current_cmd_us;
    uint16_t current_dshot_value;
    uint16_t run_cmd_dshot;
    uint32_t zero_offset_sample_count;
    float zero_offset_sum_v;
    float zero_offset_voltage;
    uint8_t boot_banner_sent;
    uint8_t connected_banner_sent;
    uint8_t wait_start_banner_sent;
    uint8_t header_sent;
    uint8_t dshot_dma_busy;
    uint8_t bt_state_last;
    uint8_t bt_confirmed;
    uint8_t start_command_received;
    uint8_t start_armed_once;
    uint8_t button_raw_pressed;
    uint8_t button_stable_pressed;
    uint8_t oled_ready;
    H1_AdcProcessed_t adc;
} H1_TestContext_t;

static H1_TestContext_t g_h1;
static volatile uint16_t g_adc_dma_buffer[H1_ADC_DMA_BUFFER_LENGTH];
static uint16_t g_dshot_dma_buffer[H1_DSHOT_DMA_BUFFER_LENGTH];
static uint8_t g_oled_buffer[128U * 8U];
static uint8_t g_uart_rx_byte;
static char g_uart_cmd_buffer[16];
static uint8_t g_uart_cmd_index;

static void H1_Test_EnterState(H1_TestState_t new_state);
static void H1_StartSession(uint32_t now_ms);
static uint8_t H1_IsBluetoothConnected(void);
static void H1_SendBootBannerOnce(void);
static void H1_SendConnectedBannerOnce(void);
static void H1_SendWaitStartBannerOnce(void);
static void H1_SendStartAck(void);
static void H1_SendCsvHeaderOnce(void);
static void H1_SendStateChangeLine(H1_TestState_t state);
static void H1_UartWrite(const char *text);
static void H1_FormatFloat3(char *dst, size_t len, float value);
static void H1_StatusLed_Set(uint8_t on);
static void H1_StatusLed_Update(uint32_t now_ms);
static uint16_t H1_MapUsToDshot(uint16_t us);
static uint16_t H1_ClampDshotThrottle(uint16_t cmd);
static uint16_t H1_Dshot_BuildPacket(uint16_t throttle_value);
static void H1_Dshot_PrepareFrame(uint16_t throttle_value);
static void H1_Dshot_TriggerFrame(uint16_t throttle_value);
static void H1_Dshot_Service(uint32_t now_ms);
static void H1_UartRx_Start(void);
static void H1_ProcessReceivedByte(uint8_t byte);
static uint8_t H1_IsStartCommand(const char *text);
static void H1_ProcessStartCommand(void);
static void H1_Button_Task(uint32_t now_ms);
static void H1_HandleThrottleButtonPress(void);
static void H1_SendThrottleSetLine(void);
static HAL_StatusTypeDef H1_Oled_WriteCommand(uint8_t command);
static HAL_StatusTypeDef H1_Oled_WriteData(const uint8_t *data, uint16_t size);
static void H1_Oled_Init(void);
static void H1_Oled_Clear(void);
static void H1_Oled_Flush(void);
static void H1_Oled_Update(uint32_t now_ms);
static void H1_Oled_DrawLine(uint8_t page, const char *text);
static void H1_Oled_SetPixel(uint8_t x, uint8_t y, uint8_t on);
static void H1_Oled_GetGlyph5x7(char c, uint8_t glyph[5]);

void H1_Test_Init(void)
{
    memset(&g_h1, 0, sizeof(g_h1));

    g_h1.test_start_tick = HAL_GetTick();
    g_h1.state_enter_tick = g_h1.test_start_tick;
    g_h1.state = STATE_WAIT_BT;
    g_h1.current_cmd_us = 0U;
    g_h1.current_dshot_value = 0U;
    g_h1.run_cmd_dshot = H1_RUN_THROTTLE_DSHOT;
    g_h1.zero_offset_voltage = CURRENT_OFFSET_V;

    if (g_h1.run_cmd_dshot < H1_RUN_THROTTLE_MIN_DSHOT)
    {
        g_h1.run_cmd_dshot = H1_RUN_THROTTLE_MIN_DSHOT;
    }
    else if (g_h1.run_cmd_dshot > H1_RUN_THROTTLE_MAX_DSHOT)
    {
        g_h1.run_cmd_dshot = H1_RUN_THROTTLE_MAX_DSHOT;
    }

    set_throttle_command(0U);

    __HAL_TIM_SET_COUNTER(&htim3, 0U);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);

    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)g_adc_dma_buffer, H1_ADC_DMA_BUFFER_LENGTH) != HAL_OK)
    {
        Error_Handler();
    }

    H1_UartRx_Start();
    H1_Oled_Init();
    H1_Oled_Update(HAL_GetTick());
    H1_Dshot_TriggerFrame(g_h1.current_dshot_value);
    H1_StatusLed_Update(HAL_GetTick());
}

void H1_Test_Task(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint32_t state_elapsed_ms;
    uint8_t bt_connected;

    if (g_h1.state == STATE_IDLE)
    {
        return;
    }

    process_adc_average(&g_h1.adc);
    update_zero_offset(&g_h1.adc, now_ms);
    H1_Dshot_Service(now_ms);
    H1_StatusLed_Update(now_ms);
    H1_Button_Task(now_ms);
    H1_Oled_Update(now_ms);

    bt_connected = H1_IsBluetoothConnected();
    if (g_h1.state == STATE_WAIT_BT)
    {
        if (bt_connected != 0U)
        {
            if (g_h1.bt_state_last == 0U)
            {
                g_h1.bt_high_since_tick = now_ms;
            }

            if ((now_ms - g_h1.bt_high_since_tick) >= H1_BT_CONNECT_CONFIRM_MS)
            {
                g_h1.bt_confirmed = 1U;
                H1_SendBootBannerOnce();
                H1_SendConnectedBannerOnce();
                H1_SendWaitStartBannerOnce();
            }

            if ((g_h1.bt_confirmed != 0U) && (g_h1.start_command_received != 0U))
            {
                H1_StartSession(now_ms);
                return;
            }
        }
        else
        {
            g_h1.bt_high_since_tick = 0U;
            g_h1.bt_confirmed = 0U;
            g_h1.start_command_received = 0U;
            g_h1.start_armed_once = 0U;
            g_h1.boot_banner_sent = 0U;
            g_h1.connected_banner_sent = 0U;
            g_h1.wait_start_banner_sent = 0U;
        }
    }
    g_h1.bt_state_last = bt_connected;

    if ((g_h1.state != STATE_WAIT_BT) && ((now_ms - g_h1.test_start_tick) >= H1_SESSION_MAX_MS))
    {
        H1_Test_EnterState(STATE_IDLE);
        return;
    }

    if ((g_h1.state != STATE_WAIT_BT) && ((now_ms - g_h1.last_csv_tick) >= H1_CSV_INTERVAL_MS))
    {
        g_h1.last_csv_tick = now_ms;
        send_h1_csv_line(&g_h1.adc);
    }

    state_elapsed_ms = now_ms - g_h1.state_enter_tick;

    switch (g_h1.state)
    {
    case STATE_WAIT_BT:
        break;

    case STATE_PREPARE:
        if (state_elapsed_ms >= H1_BT_PREPARE_MS)
        {
            H1_Test_EnterState(STATE_RUN);
        }
        break;

    case STATE_RUN:
        if (state_elapsed_ms >= H1_RUN_MS)
        {
            H1_Test_EnterState(STATE_STOP);
        }
        break;

    case STATE_STOP:
        if (state_elapsed_ms >= H1_STOP_MS)
        {
            H1_Test_EnterState(STATE_DONE);
        }
        break;

    case STATE_DONE:
    case STATE_IDLE:
    default:
        break;
    }
}

const char *H1_Test_GetStateName(H1_TestState_t state)
{
    switch (state)
    {
    case STATE_WAIT_BT:
        return "WAIT_BT";
    case STATE_PREPARE:
        return "PREPARE";
    case STATE_RUN:
        return "RUN";
    case STATE_STOP:
        return "STOP";
    case STATE_DONE:
        return "DONE";
    case STATE_IDLE:
        return "IDLE";
    default:
        return "UNKNOWN";
    }
}

void set_throttle_us(uint16_t us)
{
    uint16_t pulse_us = us;

    if (pulse_us < H1_PWM_MIN_US)
    {
        pulse_us = H1_PWM_MIN_US;
    }
    else if (pulse_us > H1_PWM_MAX_US)
    {
        pulse_us = H1_PWM_MAX_US;
    }

    g_h1.current_cmd_us = pulse_us;
    g_h1.current_dshot_value = H1_MapUsToDshot(pulse_us);
}

void set_throttle_command(uint16_t cmd)
{
    g_h1.current_dshot_value = H1_ClampDshotThrottle(cmd);
    g_h1.current_cmd_us = g_h1.current_dshot_value;
}

void process_adc_average(H1_AdcProcessed_t *out)
{
    uint32_t i;
    uint32_t sum_i = 0U;
    uint32_t sum_vbat = 0U;
    float adc_to_volt = ADC_REF_VOLTAGE / ADC_MAX_COUNTS;

    if (out == NULL)
    {
        return;
    }

    for (i = 0U; i < H1_ADC_DMA_BUFFER_LENGTH; i += H1_ADC_CHANNEL_COUNT)
    {
        sum_i += g_adc_dma_buffer[i];
        sum_vbat += g_adc_dma_buffer[i + 1U];
    }

    out->adc_i_raw = (uint16_t)(sum_i / H1_ADC_DMA_SAMPLES_PER_CHANNEL);
    out->adc_vbat_raw = (uint16_t)(sum_vbat / H1_ADC_DMA_SAMPLES_PER_CHANNEL);

    out->v_i_sense = (float)out->adc_i_raw * adc_to_volt;
    out->v_vbat_adc = (float)out->adc_vbat_raw * adc_to_volt;
    out->current_A = (out->v_i_sense - g_h1.zero_offset_voltage) * CURRENT_SCALE_A_PER_V;
    out->vbat_V = out->v_vbat_adc * VBAT_DIVIDER_RATIO;
    out->power_W = out->current_A * out->vbat_V;
}

void update_zero_offset(const H1_AdcProcessed_t *adc_data, uint32_t now_ms)
{
    if (adc_data == NULL)
    {
        return;
    }

    if ((g_h1.state != STATE_WAIT_BT) && (g_h1.state != STATE_PREPARE))
    {
        return;
    }

    if (g_h1.current_dshot_value != 0U)
    {
        return;
    }

    if ((now_ms - g_h1.last_zero_offset_sample_tick) < H1_ZERO_OFFSET_SAMPLE_INTERVAL_MS)
    {
        return;
    }

    g_h1.last_zero_offset_sample_tick = now_ms;
    g_h1.zero_offset_sum_v += adc_data->v_i_sense;
    g_h1.zero_offset_sample_count++;

    if (g_h1.zero_offset_sample_count > 0U)
    {
        g_h1.zero_offset_voltage = g_h1.zero_offset_sum_v / (float)g_h1.zero_offset_sample_count;
    }
}

void send_h1_csv_line(const H1_AdcProcessed_t *adc_data)
{
    char line[224];
    char v_i_sense_str[20];
    char v_vbat_adc_str[20];
    char current_a_str[20];
    char vbat_v_str[20];
    char power_w_str[20];
    char zero_offset_str[20];
    uint32_t t_ms;
    int len;

    if (adc_data == NULL)
    {
        return;
    }

    H1_SendCsvHeaderOnce();

    t_ms = HAL_GetTick() - g_h1.test_start_tick;

    H1_FormatFloat3(v_i_sense_str, sizeof(v_i_sense_str), adc_data->v_i_sense);
    H1_FormatFloat3(v_vbat_adc_str, sizeof(v_vbat_adc_str), adc_data->v_vbat_adc);
    H1_FormatFloat3(current_a_str, sizeof(current_a_str), adc_data->current_A);
    H1_FormatFloat3(vbat_v_str, sizeof(vbat_v_str), adc_data->vbat_V);
    H1_FormatFloat3(power_w_str, sizeof(power_w_str), adc_data->power_W);
    H1_FormatFloat3(zero_offset_str, sizeof(zero_offset_str), g_h1.zero_offset_voltage);

    len = snprintf(line,
                   sizeof(line),
                   "h1:%lu,%u,%u,%u,%u,%u,%s,%s,%s,%s,%s,%s\r\n",
                   (unsigned long)t_ms,
                   (unsigned int)g_h1.state,
                   (unsigned int)g_h1.current_cmd_us,
                   (unsigned int)g_h1.current_dshot_value,
                   (unsigned int)adc_data->adc_i_raw,
                   (unsigned int)adc_data->adc_vbat_raw,
                   v_i_sense_str,
                   v_vbat_adc_str,
                   current_a_str,
                   vbat_v_str,
                   power_w_str,
                   zero_offset_str);

    if (len > 0)
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)line, (uint16_t)len, H1_UART_TX_TIMEOUT_MS);
    }
}

static void H1_Test_EnterState(H1_TestState_t new_state)
{
    g_h1.state = new_state;
    g_h1.state_enter_tick = HAL_GetTick();

    switch (new_state)
    {
    case STATE_WAIT_BT:
    case STATE_PREPARE:
    case STATE_STOP:
    case STATE_DONE:
    case STATE_IDLE:
        set_throttle_command(0U);
        break;

    case STATE_RUN:
        set_throttle_command(g_h1.run_cmd_dshot);
        break;

    default:
        set_throttle_command(0U);
        break;
    }

    if (new_state == STATE_IDLE)
    {
        if (g_h1.dshot_dma_busy == 0U)
        {
            H1_Dshot_TriggerFrame(g_h1.current_dshot_value);
        }
    }

    H1_SendStateChangeLine(new_state);
    H1_StatusLed_Update(g_h1.state_enter_tick);
}

static void H1_StartSession(uint32_t now_ms)
{
    g_h1.test_start_tick = now_ms;
    g_h1.last_csv_tick = now_ms;
    g_h1.last_zero_offset_sample_tick = now_ms;
    g_h1.zero_offset_sum_v = 0.0f;
    g_h1.zero_offset_sample_count = 0U;
    g_h1.zero_offset_voltage = CURRENT_OFFSET_V;
    g_h1.start_command_received = 0U;
    g_h1.start_armed_once = 0U;
    g_h1.header_sent = 0U;

    H1_Test_EnterState(STATE_PREPARE);
    H1_SendCsvHeaderOnce();
}

static uint8_t H1_IsBluetoothConnected(void)
{
    return (HAL_GPIO_ReadPin(BT_STATE_GPIO_Port, BT_STATE_Pin) == H1_BT_CONNECTED_LEVEL) ? 1U : 0U;
}

static void H1_SendBootBannerOnce(void)
{
    static const char banner[] =
        "# boot,t_ms=0,fw=H1_DSHOT500_HC05_TRIGGER,uart=9600,protocol=firewater\r\n";

    if (g_h1.boot_banner_sent == 0U)
    {
        H1_UartWrite(banner);
        g_h1.boot_banner_sent = 1U;
    }
}

static void H1_SendConnectedBannerOnce(void)
{
    char line[128];
    int len;

    if (g_h1.connected_banner_sent != 0U)
    {
        return;
    }

    len = snprintf(line,
                   sizeof(line),
                   "# connected,t_ms=0,state=%s,prepare_ms=%lu,confirm_ms=%lu,run_cmd_raw=%u,dshot_start=after_prepare\r\n",
                   H1_Test_GetStateName(g_h1.state),
                   (unsigned long)H1_BT_PREPARE_MS,
                   (unsigned long)H1_BT_CONNECT_CONFIRM_MS,
                   (unsigned int)H1_RUN_THROTTLE_DSHOT);
    if (len > 0)
    {
        H1_UartWrite(line);
    }

    g_h1.connected_banner_sent = 1U;
}

static void H1_SendWaitStartBannerOnce(void)
{
    static const char line[] =
        "# waiting,start_cmd=START,send_newline=optional,action=prepare_after_start\r\n";

    if (g_h1.wait_start_banner_sent == 0U)
    {
        H1_UartWrite(line);
        g_h1.wait_start_banner_sent = 1U;
    }
}

static void H1_SendStartAck(void)
{
    static const char line[] =
        "# ack,start_received,action=enter_prepare\r\n";

    H1_UartWrite(line);
}

static void H1_SendCsvHeaderOnce(void)
{
    static const char header[] =
        "# firewater_channels,ch0=t_ms,ch1=state,ch2=cmd_raw,ch3=dshot_cmd,ch4=adc_i_raw,ch5=adc_vbat_raw,ch6=v_i_sense,ch7=v_vbat_adc,ch8=current_A,ch9=vbat_V,ch10=power_W,ch11=zero_offset_V\r\n";

    if (g_h1.header_sent == 0U)
    {
        H1_UartWrite(header);
        g_h1.header_sent = 1U;
    }
}

static void H1_SendStateChangeLine(H1_TestState_t state)
{
    char line[96];
    uint32_t t_ms;
    int len;

    if (state == STATE_WAIT_BT)
    {
        return;
    }

    t_ms = HAL_GetTick() - g_h1.test_start_tick;
    len = snprintf(line,
                   sizeof(line),
                   "# state_change,t_ms=%lu,state=%u,name=%s\r\n",
                   (unsigned long)t_ms,
                   (unsigned int)state,
                   H1_Test_GetStateName(state));
    if (len > 0)
    {
        H1_UartWrite(line);
    }
}

static void H1_UartWrite(const char *text)
{
    size_t len;

    if (text == NULL)
    {
        return;
    }

    len = strlen(text);
    if (len > 0U)
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)len, H1_UART_TX_TIMEOUT_MS);
    }
}

static void H1_FormatFloat3(char *dst, size_t len, float value)
{
    int32_t scaled;
    int32_t integer_part;
    int32_t fractional_part;

    if ((dst == NULL) || (len == 0U))
    {
        return;
    }

    scaled = (int32_t)(value * 1000.0f + ((value >= 0.0f) ? 0.5f : -0.5f));
    integer_part = scaled / 1000;
    fractional_part = scaled % 1000;
    if (fractional_part < 0)
    {
        fractional_part = -fractional_part;
    }

    (void)snprintf(dst, len, "%ld.%03ld", (long)integer_part, (long)fractional_part);
}

static void H1_Button_Task(uint32_t now_ms)
{
    uint8_t raw_pressed;

    raw_pressed = (HAL_GPIO_ReadPin(BTN_THROTTLE_GPIO_Port, BTN_THROTTLE_Pin) == GPIO_PIN_RESET) ? 1U : 0U;

    if (raw_pressed != g_h1.button_raw_pressed)
    {
        g_h1.button_raw_pressed = raw_pressed;
        g_h1.button_change_tick = now_ms;
    }

    if ((now_ms - g_h1.button_change_tick) < H1_BUTTON_DEBOUNCE_MS)
    {
        return;
    }

    if (raw_pressed != g_h1.button_stable_pressed)
    {
        g_h1.button_stable_pressed = raw_pressed;

        if ((raw_pressed != 0U) &&
            (g_h1.state == STATE_WAIT_BT) &&
            (g_h1.bt_confirmed != 0U) &&
            (g_h1.start_command_received == 0U) &&
            (g_h1.start_armed_once == 0U))
        {
            H1_HandleThrottleButtonPress();
        }
    }
}

static void H1_HandleThrottleButtonPress(void)
{
    uint16_t next_cmd = g_h1.run_cmd_dshot;

    if ((next_cmd < H1_RUN_THROTTLE_MIN_DSHOT) || (next_cmd > H1_RUN_THROTTLE_MAX_DSHOT))
    {
        next_cmd = H1_RUN_THROTTLE_MIN_DSHOT;
    }
    else
    {
        next_cmd = (uint16_t)(next_cmd + H1_RUN_THROTTLE_STEP_DSHOT);
        if (next_cmd > H1_RUN_THROTTLE_MAX_DSHOT)
        {
            next_cmd = H1_RUN_THROTTLE_MIN_DSHOT;
        }
    }

    g_h1.run_cmd_dshot = next_cmd;
    H1_SendThrottleSetLine();
    g_h1.last_oled_update_tick = 0U;
    H1_Oled_Update(HAL_GetTick());
}

static void H1_SendThrottleSetLine(void)
{
    char line[64];
    int len;

    len = snprintf(line,
                   sizeof(line),
                   "# throttle_set,run_cmd_raw=%u\r\n",
                   (unsigned int)g_h1.run_cmd_dshot);
    if (len > 0)
    {
        H1_UartWrite(line);
    }
}

static HAL_StatusTypeDef H1_Oled_WriteCommand(uint8_t command)
{
    uint8_t frame[2];

    if (g_h1.oled_ready == 0U)
    {
        return HAL_ERROR;
    }

    frame[0] = 0x00U;
    frame[1] = command;
    return HAL_I2C_Master_Transmit(&hi2c1, H1_OLED_I2C_ADDR, frame, 2U, 50U);
}

static HAL_StatusTypeDef H1_Oled_WriteData(const uint8_t *data, uint16_t size)
{
    uint8_t frame[129];

    if ((g_h1.oled_ready == 0U) || (data == NULL) || (size > 128U))
    {
        return HAL_ERROR;
    }

    frame[0] = 0x40U;
    memcpy(&frame[1], data, size);
    return HAL_I2C_Master_Transmit(&hi2c1, H1_OLED_I2C_ADDR, frame, (uint16_t)(size + 1U), 50U);
}

static void H1_Oled_Init(void)
{
    static const uint8_t init_cmds[] =
    {
        0xAEU, 0x20U, 0x02U, 0xB0U, 0xC8U, 0x00U, 0x10U, 0x40U,
        0x81U, 0x7FU, 0xA1U, 0xA6U, 0xA8U, 0x3FU, 0xA4U, 0xD3U,
        0x00U, 0xD5U, 0x80U, 0xD9U, 0xF1U, 0xDAU, 0x12U, 0xDBU,
        0x20U, 0x8DU, 0x14U, 0xAFU
    };
    uint8_t i;

    g_h1.oled_ready = 0U;

    if (HAL_I2C_IsDeviceReady(&hi2c1, H1_OLED_I2C_ADDR, 2U, 100U) != HAL_OK)
    {
        return;
    }

    g_h1.oled_ready = 1U;

    for (i = 0U; i < (uint8_t)(sizeof(init_cmds) / sizeof(init_cmds[0])); i++)
    {
        if (H1_Oled_WriteCommand(init_cmds[i]) != HAL_OK)
        {
            g_h1.oled_ready = 0U;
            return;
        }
    }

    H1_Oled_Clear();
}

static void H1_Oled_Clear(void)
{
    if (g_h1.oled_ready == 0U)
    {
        return;
    }

    memset(g_oled_buffer, 0, sizeof(g_oled_buffer));
    H1_Oled_Flush();
}

static void H1_Oled_Flush(void)
{
    uint8_t page;

    if (g_h1.oled_ready == 0U)
    {
        return;
    }

    for (page = 0U; page < 8U; page++)
    {
        (void)H1_Oled_WriteCommand((uint8_t)(0xB0U | page));
        (void)H1_Oled_WriteCommand((uint8_t)(0x00U | (H1_OLED_COLUMN_OFFSET & 0x0FU)));
        (void)H1_Oled_WriteCommand((uint8_t)(0x10U | ((H1_OLED_COLUMN_OFFSET >> 4U) & 0x0FU)));
        (void)H1_Oled_WriteData(&g_oled_buffer[page * 128U], 128U);
    }
}

static void H1_Oled_Update(uint32_t now_ms)
{
    char line0[32];
    char line1[32];
    char line2[32];
    char line3[32];
    char value_str[20];

    if (g_h1.oled_ready == 0U)
    {
        return;
    }

    if ((g_h1.last_oled_update_tick != 0U) &&
        ((now_ms - g_h1.last_oled_update_tick) < H1_OLED_UPDATE_INTERVAL_MS))
    {
        return;
    }

    g_h1.last_oled_update_tick = now_ms;
    memset(g_oled_buffer, 0, sizeof(g_oled_buffer));

    H1_FormatFloat3(value_str, sizeof(value_str), g_h1.adc.vbat_V);
    (void)snprintf(line0, sizeof(line0), "VBAT %sV", value_str);

    H1_FormatFloat3(value_str, sizeof(value_str), g_h1.adc.current_A);
    (void)snprintf(line1, sizeof(line1), "CURR %sA", value_str);

    H1_FormatFloat3(value_str, sizeof(value_str), g_h1.adc.power_W);
    (void)snprintf(line2, sizeof(line2), "POWR %sW", value_str);

    (void)snprintf(line3, sizeof(line3), "THR  %u", (unsigned int)g_h1.run_cmd_dshot);

    H1_Oled_DrawLine(0U, line0);
    H1_Oled_DrawLine(1U, line1);
    H1_Oled_DrawLine(2U, line2);
    H1_Oled_DrawLine(3U, line3);
    H1_Oled_Flush();
}

static void H1_Oled_DrawLine(uint8_t line_index, const char *text)
{
    uint8_t glyph[5];
    uint8_t x = 0U;
    uint8_t y;
    uint8_t col;
    uint8_t row;

    if ((g_h1.oled_ready == 0U) || (line_index >= 4U))
    {
        return;
    }

    y = (uint8_t)(line_index * 11U);

    while ((text != NULL) && (*text != '\0') && (x <= 120U))
    {
        H1_Oled_GetGlyph5x7(*text, glyph);

        for (col = 0U; col < 5U; col++)
        {
            for (row = 0U; row < 7U; row++)
            {
                if ((glyph[col] & (uint8_t)(1U << row)) != 0U)
                {
                    H1_Oled_SetPixel((uint8_t)(x + col), (uint8_t)(y + row), 1U);
                    H1_Oled_SetPixel((uint8_t)(x + col + 1U), (uint8_t)(y + row), 1U);
                }
            }
        }

        x = (uint8_t)(x + 7U);
        text++;
    }
}

static void H1_Oled_SetPixel(uint8_t x, uint8_t y, uint8_t on)
{
    uint16_t index;
    uint8_t mask;

    if ((x >= 128U) || (y >= 64U))
    {
        return;
    }

    index = (uint16_t)((y / 8U) * 128U + x);
    mask = (uint8_t)(1U << (y % 8U));

    if (on != 0U)
    {
        g_oled_buffer[index] |= mask;
    }
    else
    {
        g_oled_buffer[index] &= (uint8_t)(~mask);
    }
}

static void H1_Oled_GetGlyph5x7(char c, uint8_t glyph[5])
{
    if (glyph == NULL)
    {
        return;
    }

    memset(glyph, 0, 5U);

    switch (c)
    {
    case '-': glyph[0] = 0x08U; glyph[1] = 0x08U; glyph[2] = 0x08U; glyph[3] = 0x08U; glyph[4] = 0x08U; break;
    case '.': glyph[1] = 0x60U; glyph[2] = 0x60U; break;
    case '0': glyph[0] = 0x3EU; glyph[1] = 0x51U; glyph[2] = 0x49U; glyph[3] = 0x45U; glyph[4] = 0x3EU; break;
    case '1': glyph[0] = 0x00U; glyph[1] = 0x42U; glyph[2] = 0x7FU; glyph[3] = 0x40U; glyph[4] = 0x00U; break;
    case '2': glyph[0] = 0x42U; glyph[1] = 0x61U; glyph[2] = 0x51U; glyph[3] = 0x49U; glyph[4] = 0x46U; break;
    case '3': glyph[0] = 0x21U; glyph[1] = 0x41U; glyph[2] = 0x45U; glyph[3] = 0x4BU; glyph[4] = 0x31U; break;
    case '4': glyph[0] = 0x18U; glyph[1] = 0x14U; glyph[2] = 0x12U; glyph[3] = 0x7FU; glyph[4] = 0x10U; break;
    case '5': glyph[0] = 0x27U; glyph[1] = 0x45U; glyph[2] = 0x45U; glyph[3] = 0x45U; glyph[4] = 0x39U; break;
    case '6': glyph[0] = 0x3CU; glyph[1] = 0x4AU; glyph[2] = 0x49U; glyph[3] = 0x49U; glyph[4] = 0x30U; break;
    case '7': glyph[0] = 0x01U; glyph[1] = 0x71U; glyph[2] = 0x09U; glyph[3] = 0x05U; glyph[4] = 0x03U; break;
    case '8': glyph[0] = 0x36U; glyph[1] = 0x49U; glyph[2] = 0x49U; glyph[3] = 0x49U; glyph[4] = 0x36U; break;
    case '9': glyph[0] = 0x06U; glyph[1] = 0x49U; glyph[2] = 0x49U; glyph[3] = 0x29U; glyph[4] = 0x1EU; break;
    case 'A': glyph[0] = 0x7EU; glyph[1] = 0x11U; glyph[2] = 0x11U; glyph[3] = 0x11U; glyph[4] = 0x7EU; break;
    case 'B': glyph[0] = 0x7FU; glyph[1] = 0x49U; glyph[2] = 0x49U; glyph[3] = 0x49U; glyph[4] = 0x36U; break;
    case 'C': glyph[0] = 0x3EU; glyph[1] = 0x41U; glyph[2] = 0x41U; glyph[3] = 0x41U; glyph[4] = 0x22U; break;
    case 'H': glyph[0] = 0x7FU; glyph[1] = 0x08U; glyph[2] = 0x08U; glyph[3] = 0x08U; glyph[4] = 0x7FU; break;
    case 'O': glyph[0] = 0x3EU; glyph[1] = 0x41U; glyph[2] = 0x41U; glyph[3] = 0x41U; glyph[4] = 0x3EU; break;
    case 'P': glyph[0] = 0x7FU; glyph[1] = 0x09U; glyph[2] = 0x09U; glyph[3] = 0x09U; glyph[4] = 0x06U; break;
    case 'R': glyph[0] = 0x7FU; glyph[1] = 0x09U; glyph[2] = 0x19U; glyph[3] = 0x29U; glyph[4] = 0x46U; break;
    case 'T': glyph[0] = 0x01U; glyph[1] = 0x01U; glyph[2] = 0x7FU; glyph[3] = 0x01U; glyph[4] = 0x01U; break;
    case 'U': glyph[0] = 0x3FU; glyph[1] = 0x40U; glyph[2] = 0x40U; glyph[3] = 0x40U; glyph[4] = 0x3FU; break;
    case 'V': glyph[0] = 0x1FU; glyph[1] = 0x20U; glyph[2] = 0x40U; glyph[3] = 0x20U; glyph[4] = 0x1FU; break;
    case 'W': glyph[0] = 0x7FU; glyph[1] = 0x20U; glyph[2] = 0x18U; glyph[3] = 0x20U; glyph[4] = 0x7FU; break;
    default: break;
    }
}

static void H1_StatusLed_Set(uint8_t on)
{
    HAL_GPIO_WritePin(STATUS_LED_GPIO_Port,
                      STATUS_LED_Pin,
                      (on != 0U) ? H1_STATUS_LED_ON_LEVEL : H1_STATUS_LED_OFF_LEVEL);
}

static void H1_StatusLed_Update(uint32_t now_ms)
{
    uint32_t phase_ms;

    switch (g_h1.state)
    {
    case STATE_WAIT_BT:
        phase_ms = now_ms % 2000U;
        H1_StatusLed_Set(((phase_ms < 80U) || ((phase_ms >= 250U) && (phase_ms < 330U))) ? 1U : 0U);
        break;

    case STATE_PREPARE:
        phase_ms = now_ms % 1000U;
        H1_StatusLed_Set((phase_ms < 120U) ? 1U : 0U);
        break;

    case STATE_RUN:
        H1_StatusLed_Set(1U);
        break;

    case STATE_STOP:
        phase_ms = now_ms % 250U;
        H1_StatusLed_Set((phase_ms < 125U) ? 1U : 0U);
        break;

    case STATE_DONE:
    default:
        H1_StatusLed_Set(0U);
        break;

    case STATE_IDLE:
        H1_StatusLed_Set(0U);
        break;
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM3)
    {
        return;
    }

    (void)HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COUNTER(htim, 0U);
    g_h1.dshot_dma_busy = 0U;
}

static uint16_t H1_MapUsToDshot(uint16_t us)
{
    uint32_t scaled;

    if (us <= H1_PWM_MIN_US)
    {
        return 0U;
    }

    if (us >= H1_PWM_MAX_US)
    {
        return H1_DSHOT_MAX_THROTTLE;
    }

    scaled = (uint32_t)(us - (H1_PWM_MIN_US + 1U)) * (uint32_t)(H1_DSHOT_MAX_THROTTLE - H1_DSHOT_MIN_THROTTLE);
    scaled /= (uint32_t)(H1_PWM_MAX_US - (H1_PWM_MIN_US + 1U));

    return (uint16_t)(H1_DSHOT_MIN_THROTTLE + scaled);
}

static uint16_t H1_ClampDshotThrottle(uint16_t cmd)
{
    if (cmd == 0U)
    {
        return 0U;
    }

    if (cmd < H1_DSHOT_MIN_THROTTLE)
    {
        return H1_DSHOT_MIN_THROTTLE;
    }

    if (cmd > H1_DSHOT_MAX_THROTTLE)
    {
        return H1_DSHOT_MAX_THROTTLE;
    }

    return cmd;
}

static uint16_t H1_Dshot_BuildPacket(uint16_t throttle_value)
{
    uint16_t packet;
    uint16_t checksum;
    uint16_t csum_data;
    uint8_t i;

    packet = (uint16_t)((throttle_value << 1U) | (H1_DSHOT_TELEMETRY_BIT & 0x1U));
    csum_data = packet;
    checksum = 0U;

    for (i = 0U; i < 3U; i++)
    {
        checksum ^= (uint16_t)(csum_data & 0xFU);
        csum_data >>= 4U;
    }

    checksum &= 0xFU;
    return (uint16_t)((packet << 4U) | checksum);
}

static void H1_Dshot_PrepareFrame(uint16_t throttle_value)
{
    uint16_t packet;
    uint8_t i;

    packet = H1_Dshot_BuildPacket(throttle_value);

    for (i = 0U; i < H1_DSHOT_FRAME_BITS; i++)
    {
        g_dshot_dma_buffer[i] = ((packet & 0x8000U) != 0U) ? H1_DSHOT_BIT_1_HIGH_TICKS : H1_DSHOT_BIT_0_HIGH_TICKS;
        packet <<= 1U;
    }

    for (i = H1_DSHOT_FRAME_BITS; i < H1_DSHOT_DMA_BUFFER_LENGTH; i++)
    {
        g_dshot_dma_buffer[i] = 0U;
    }
}

static void H1_Dshot_TriggerFrame(uint16_t throttle_value)
{
    HAL_StatusTypeDef status;

    if (g_h1.dshot_dma_busy != 0U)
    {
        return;
    }

    H1_Dshot_PrepareFrame(throttle_value);
    g_h1.dshot_dma_busy = 1U;

    __HAL_TIM_DISABLE(&htim3);
    __HAL_TIM_SET_COUNTER(&htim3, 0U);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, g_dshot_dma_buffer[0]);

    status = HAL_TIM_PWM_Start_DMA(&htim3,
                                   TIM_CHANNEL_1,
                                   (const uint32_t *)&g_dshot_dma_buffer[1],
                                   (uint16_t)(H1_DSHOT_DMA_BUFFER_LENGTH - 1U));
    if (status != HAL_OK)
    {
        g_h1.dshot_dma_busy = 0U;
        Error_Handler();
    }
}

static void H1_Dshot_Service(uint32_t now_ms)
{
    if ((now_ms - g_h1.last_dshot_send_tick) < H1_DSHOT_SEND_INTERVAL_MS)
    {
        return;
    }

    if (g_h1.dshot_dma_busy != 0U)
    {
        return;
    }

    g_h1.last_dshot_send_tick = now_ms;
    H1_Dshot_TriggerFrame(g_h1.current_dshot_value);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        H1_ProcessReceivedByte(g_uart_rx_byte);
        H1_UartRx_Start();
    }
}

static void H1_UartRx_Start(void)
{
    if (HAL_UART_Receive_IT(&huart1, &g_uart_rx_byte, 1U) != HAL_OK)
    {
        Error_Handler();
    }
}

static void H1_ProcessReceivedByte(uint8_t byte)
{
    char c = (char)byte;

    if ((c == '\r') || (c == '\n'))
    {
        if (g_uart_cmd_index > 0U)
        {
            g_uart_cmd_buffer[g_uart_cmd_index] = '\0';
            if ((g_h1.state == STATE_WAIT_BT) &&
                (g_h1.bt_confirmed != 0U) &&
                (H1_IsStartCommand(g_uart_cmd_buffer) != 0U))
            {
                H1_ProcessStartCommand();
            }
            g_uart_cmd_index = 0U;
        }
        return;
    }

    if ((c >= 'a') && (c <= 'z'))
    {
        c = (char)(c - 'a' + 'A');
    }

    if (((c >= 'A') && (c <= 'Z')) || ((c >= '0') && (c <= '9')) || (c == '_'))
    {
        if (g_uart_cmd_index < (uint8_t)(sizeof(g_uart_cmd_buffer) - 1U))
        {
            g_uart_cmd_buffer[g_uart_cmd_index++] = c;
            g_uart_cmd_buffer[g_uart_cmd_index] = '\0';

            if ((g_h1.state == STATE_WAIT_BT) &&
                (g_h1.bt_confirmed != 0U) &&
                (H1_IsStartCommand(g_uart_cmd_buffer) != 0U))
            {
                H1_ProcessStartCommand();
                g_uart_cmd_index = 0U;
            }
        }
        else
        {
            g_uart_cmd_index = 0U;
        }
    }
    else
    {
        g_uart_cmd_index = 0U;
    }
}

static uint8_t H1_IsStartCommand(const char *text)
{
    if (text == NULL)
    {
        return 0U;
    }

    return (strcmp(text, "START") == 0) ? 1U : 0U;
}

static void H1_ProcessStartCommand(void)
{
    if (g_h1.start_armed_once == 0U)
    {
        g_h1.start_command_received = 1U;
        g_h1.start_armed_once = 1U;
        H1_SendStartAck();
    }
}
