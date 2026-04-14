#include "app_e1_test.h"

#include <stdio.h>
#include <string.h>

#include "adc.h"
#include "gpio.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"

#define E1_ADC_CHANNEL_COUNT              2U
#define E1_ADC_DMA_SAMPLES_PER_CHANNEL    64U
#define E1_ADC_DMA_BUFFER_LENGTH          (E1_ADC_CHANNEL_COUNT * E1_ADC_DMA_SAMPLES_PER_CHANNEL)

#define E1_DSHOT_FRAME_BITS               16U
#define E1_DSHOT_RESET_SLOTS              8U
#define E1_DSHOT_DMA_BUFFER_LENGTH        (E1_DSHOT_FRAME_BITS + E1_DSHOT_RESET_SLOTS)
#define E1_DSHOT_BIT_0_HIGH_TICKS         90U
#define E1_DSHOT_BIT_1_HIGH_TICKS         180U

#define E1_UART_TX_TIMEOUT_MS             500U
#define E1_STATUS_LED_ON_LEVEL            GPIO_PIN_RESET
#define E1_STATUS_LED_OFF_LEVEL           GPIO_PIN_SET
#define E1_BT_CONNECTED_LEVEL             GPIO_PIN_SET

typedef struct
{
    E1_TestState_t state;
    uint32_t test_start_tick;
    uint32_t state_enter_tick;
    uint32_t last_csv_tick;
    uint32_t last_dshot_send_tick;
    uint32_t last_zero_offset_sample_tick;
    uint32_t last_current_filter_tick;
    uint32_t last_oled_update_tick;
    uint32_t last_oled_init_attempt_tick;
    uint32_t button_change_tick;
    uint32_t bt_high_since_tick;
    uint16_t current_cmd_us;
    uint16_t current_dshot_value;
    uint16_t run_cmd_dshot;
    uint32_t zero_offset_sample_count;
    float zero_offset_sum_v;
    float baseline_zero_offset_voltage;
    float active_zero_offset_voltage;
    float current_filter_sum_A;
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
    uint8_t current_filter_index;
    uint8_t current_filter_count;
    float current_filter_buffer[E1_CURRENT_FILTER_WINDOW_SAMPLES];
    E1_AdcProcessed_t adc;
} E1_TestContext_t;

static E1_TestContext_t g_h1;
static volatile uint16_t g_adc_dma_buffer[E1_ADC_DMA_BUFFER_LENGTH];
static uint16_t g_dshot_dma_buffer[E1_DSHOT_DMA_BUFFER_LENGTH];
static uint8_t g_oled_buffer[128U * 8U];
static uint8_t g_uart_rx_byte;
static char g_uart_cmd_buffer[16];
static uint8_t g_uart_cmd_index;

static void E1_Test_EnterState(E1_TestState_t new_state);
static void E1_StartSession(uint32_t now_ms);
static uint8_t E1_IsBluetoothConnected(void);
static void E1_SendBootBannerOnce(void);
static void E1_SendConnectedBannerOnce(void);
static void E1_SendWaitStartBannerOnce(void);
static void E1_SendStartAck(void);
static void E1_SendCsvHeaderOnce(void);
static void E1_SendStateChangeLine(E1_TestState_t state);
static void E1_UartWrite(const char *text);
static void E1_FormatFloat3(char *dst, size_t len, float value);
static void E1_StatusLed_Set(uint8_t on);
static void E1_StatusLed_Update(uint32_t now_ms);
static uint16_t E1_MapUsToDshot(uint16_t us);
static uint16_t E1_ClampDshotThrottle(uint16_t cmd);
static uint16_t E1_Dshot_BuildPacket(uint16_t throttle_value);
static void E1_Dshot_PrepareFrame(uint16_t throttle_value);
static void E1_Dshot_TriggerFrame(uint16_t throttle_value);
static void E1_Dshot_Service(uint32_t now_ms);
static void E1_UartRx_Start(void);
static void E1_ProcessReceivedByte(uint8_t byte);
static uint8_t E1_IsStartCommand(const char *text);
static void E1_ProcessStartCommand(void);
static void E1_ResetCurrentFilter(void);
static void E1_UpdateCurrentDerived(E1_AdcProcessed_t *adc_data, uint32_t now_ms);
static void E1_Button_Task(uint32_t now_ms);
static void E1_HandleThrottleButtonPress(void);
static void E1_SendThrottleSetLine(void);
static HAL_StatusTypeDef E1_Oled_WriteCommand(uint8_t command);
static HAL_StatusTypeDef E1_Oled_WriteData(const uint8_t *data, uint16_t size);
static void E1_Oled_Init(void);
static void E1_Oled_Service(uint32_t now_ms);
static void E1_Oled_Clear(void);
static void E1_Oled_Flush(void);
static void E1_Oled_Update(uint32_t now_ms);
static void E1_Oled_DrawLine(uint8_t page, const char *text);
static void E1_Oled_DrawChar(uint8_t x, uint8_t y, char c);
static void E1_Oled_SetPixel(uint8_t x, uint8_t y, uint8_t on);
static void E1_Oled_GetGlyph5x7(char c, uint8_t glyph[5]);

void E1_Test_Init(void)
{
    memset(&g_h1, 0, sizeof(g_h1));

    g_h1.test_start_tick = HAL_GetTick();
    g_h1.state_enter_tick = g_h1.test_start_tick;
    g_h1.state = STATE_WAIT_BT;
    g_h1.current_cmd_us = 0U;
    g_h1.current_dshot_value = 0U;
    g_h1.run_cmd_dshot = E1_RUN_THROTTLE_DSHOT;
    g_h1.baseline_zero_offset_voltage = CURRENT_OFFSET_V;
    g_h1.active_zero_offset_voltage = CURRENT_OFFSET_V;

    if (g_h1.run_cmd_dshot < E1_RUN_THROTTLE_MIN_DSHOT)
    {
        g_h1.run_cmd_dshot = E1_RUN_THROTTLE_MIN_DSHOT;
    }
    else if (g_h1.run_cmd_dshot > E1_RUN_THROTTLE_MAX_DSHOT)
    {
        g_h1.run_cmd_dshot = E1_RUN_THROTTLE_MAX_DSHOT;
    }

    set_throttle_command(0U);

    __HAL_TIM_SET_COUNTER(&htim3, 0U);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);

    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)g_adc_dma_buffer, E1_ADC_DMA_BUFFER_LENGTH) != HAL_OK)
    {
        Error_Handler();
    }

    E1_UartRx_Start();
    HAL_Delay(E1_OLED_POWERUP_DELAY_MS);
    E1_Oled_Service(HAL_GetTick());
    E1_Dshot_TriggerFrame(g_h1.current_dshot_value);
    E1_StatusLed_Update(HAL_GetTick());
}

void E1_Test_Task(void)
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
    E1_UpdateCurrentDerived(&g_h1.adc, now_ms);
    E1_Dshot_Service(now_ms);
    E1_StatusLed_Update(now_ms);
    E1_Button_Task(now_ms);
    E1_Oled_Service(now_ms);
    E1_Oled_Update(now_ms);

    bt_connected = E1_IsBluetoothConnected();
    if (g_h1.state == STATE_WAIT_BT)
    {
        if (bt_connected != 0U)
        {
            if (g_h1.bt_state_last == 0U)
            {
                g_h1.bt_high_since_tick = now_ms;
            }

            if ((now_ms - g_h1.bt_high_since_tick) >= E1_BT_CONNECT_CONFIRM_MS)
            {
                g_h1.bt_confirmed = 1U;
                E1_SendBootBannerOnce();
                E1_SendConnectedBannerOnce();
                E1_SendWaitStartBannerOnce();
            }

            if ((g_h1.bt_confirmed != 0U) && (g_h1.start_command_received != 0U))
            {
                E1_StartSession(now_ms);
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

    if ((g_h1.state != STATE_WAIT_BT) && ((now_ms - g_h1.test_start_tick) >= E1_SESSION_MAX_MS))
    {
        E1_Test_EnterState(STATE_IDLE);
        return;
    }

    if ((g_h1.state != STATE_WAIT_BT) && ((now_ms - g_h1.last_csv_tick) >= E1_CSV_INTERVAL_MS))
    {
        g_h1.last_csv_tick = now_ms;
        send_e1_csv_line(&g_h1.adc);
    }

    state_elapsed_ms = now_ms - g_h1.state_enter_tick;

    switch (g_h1.state)
    {
    case STATE_WAIT_BT:
        break;

    case STATE_PREPARE:
        if (state_elapsed_ms >= E1_BT_PREPARE_MS)
        {
            E1_Test_EnterState(STATE_RUN);
        }
        break;

    case STATE_RUN:
        if (state_elapsed_ms >= E1_RUN_MS)
        {
            E1_Test_EnterState(STATE_STOP);
        }
        break;

    case STATE_STOP:
        if (state_elapsed_ms >= E1_STOP_MS)
        {
            E1_Test_EnterState(STATE_DONE);
        }
        break;

    case STATE_DONE:
    case STATE_IDLE:
    default:
        break;
    }
}

const char *E1_Test_GetStateName(E1_TestState_t state)
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

    if (pulse_us < E1_PWM_MIN_US)
    {
        pulse_us = E1_PWM_MIN_US;
    }
    else if (pulse_us > E1_PWM_MAX_US)
    {
        pulse_us = E1_PWM_MAX_US;
    }

    g_h1.current_cmd_us = pulse_us;
    g_h1.current_dshot_value = E1_MapUsToDshot(pulse_us);
}

void set_throttle_command(uint16_t cmd)
{
    g_h1.current_dshot_value = E1_ClampDshotThrottle(cmd);
    g_h1.current_cmd_us = g_h1.current_dshot_value;
}

void process_adc_average(E1_AdcProcessed_t *out)
{
    uint32_t i;
    uint32_t sum_i = 0U;
    uint32_t sum_vbat = 0U;
    float adc_to_volt = ADC_REF_VOLTAGE / ADC_MAX_COUNTS;

    if (out == NULL)
    {
        return;
    }

    for (i = 0U; i < E1_ADC_DMA_BUFFER_LENGTH; i += E1_ADC_CHANNEL_COUNT)
    {
        sum_i += g_adc_dma_buffer[i];
        sum_vbat += g_adc_dma_buffer[i + 1U];
    }

    out->adc_i_raw = (uint16_t)(sum_i / E1_ADC_DMA_SAMPLES_PER_CHANNEL);
    out->adc_vbat_raw = (uint16_t)(sum_vbat / E1_ADC_DMA_SAMPLES_PER_CHANNEL);

    out->v_i_sense = (float)out->adc_i_raw * adc_to_volt;
    out->v_vbat_adc = (float)out->adc_vbat_raw * adc_to_volt;
    out->vbat_V = out->v_vbat_adc * VBAT_DIVIDER_RATIO;
}

void update_zero_offset(const E1_AdcProcessed_t *adc_data, uint32_t now_ms)
{
    if (adc_data == NULL)
    {
        return;
    }

    if (g_h1.current_dshot_value != 0U)
    {
        return;
    }

    if (g_h1.zero_offset_sample_count >= E1_BASELINE_SAMPLE_COUNT)
    {
        return;
    }

    if ((now_ms - g_h1.last_zero_offset_sample_tick) < E1_ZERO_OFFSET_SAMPLE_INTERVAL_MS)
    {
        return;
    }

    g_h1.last_zero_offset_sample_tick = now_ms;
    g_h1.zero_offset_sum_v += adc_data->v_i_sense;
    g_h1.zero_offset_sample_count++;

    g_h1.baseline_zero_offset_voltage = g_h1.zero_offset_sum_v / (float)g_h1.zero_offset_sample_count;
    g_h1.active_zero_offset_voltage = g_h1.baseline_zero_offset_voltage;
}

static void E1_ResetCurrentFilter(void)
{
    memset(g_h1.current_filter_buffer, 0, sizeof(g_h1.current_filter_buffer));
    g_h1.current_filter_sum_A = 0.0f;
    g_h1.current_filter_index = 0U;
    g_h1.current_filter_count = 0U;
    g_h1.last_current_filter_tick = 0U;
}

static void E1_UpdateCurrentDerived(E1_AdcProcessed_t *adc_data, uint32_t now_ms)
{
    float signed_delta_v;
    float instant_current_a;

    if (adc_data == NULL)
    {
        return;
    }

    adc_data->active_zero_offset_V = g_h1.baseline_zero_offset_voltage;
    adc_data->delta_i_V = adc_data->v_i_sense - g_h1.baseline_zero_offset_voltage;

#if (E1_CURRENT_SIGN_INVERT != 0U)
    signed_delta_v = -adc_data->delta_i_V;
#else
    signed_delta_v = adc_data->delta_i_V;
#endif

    instant_current_a = signed_delta_v * CURRENT_SCALE_A_PER_V;
    if (instant_current_a < 0.0f)
    {
        instant_current_a = 0.0f;
    }

    if ((g_h1.last_current_filter_tick == 0U) ||
        ((now_ms - g_h1.last_current_filter_tick) >= E1_CURRENT_FILTER_SAMPLE_INTERVAL_MS))
    {
        g_h1.last_current_filter_tick = now_ms;

        if (g_h1.current_filter_count < E1_CURRENT_FILTER_WINDOW_SAMPLES)
        {
            g_h1.current_filter_buffer[g_h1.current_filter_index] = instant_current_a;
            g_h1.current_filter_sum_A += instant_current_a;
            g_h1.current_filter_count++;
        }
        else
        {
            g_h1.current_filter_sum_A -= g_h1.current_filter_buffer[g_h1.current_filter_index];
            g_h1.current_filter_buffer[g_h1.current_filter_index] = instant_current_a;
            g_h1.current_filter_sum_A += instant_current_a;
        }

        g_h1.current_filter_index++;
        if (g_h1.current_filter_index >= E1_CURRENT_FILTER_WINDOW_SAMPLES)
        {
            g_h1.current_filter_index = 0U;
        }
    }

    if (g_h1.current_filter_count > 0U)
    {
        adc_data->current_A = g_h1.current_filter_sum_A / (float)g_h1.current_filter_count;
    }
    else
    {
        adc_data->current_A = instant_current_a;
    }

    adc_data->power_W = adc_data->vbat_V * adc_data->current_A;
}

void send_e1_csv_line(const E1_AdcProcessed_t *adc_data)
{
    char line[288];
    char v_i_sense_str[20];
    char v_vbat_adc_str[20];
    char current_a_str[20];
    char vbat_v_str[20];
    char power_w_str[20];
    char zero_offset_str[20];
    char active_zero_offset_str[20];
    char delta_i_v_str[20];
    uint32_t t_ms;
    int len;

    if (adc_data == NULL)
    {
        return;
    }

    E1_SendCsvHeaderOnce();

    t_ms = HAL_GetTick() - g_h1.test_start_tick;

    E1_FormatFloat3(v_i_sense_str, sizeof(v_i_sense_str), adc_data->v_i_sense);
    E1_FormatFloat3(v_vbat_adc_str, sizeof(v_vbat_adc_str), adc_data->v_vbat_adc);
    E1_FormatFloat3(current_a_str, sizeof(current_a_str), adc_data->current_A);
    E1_FormatFloat3(vbat_v_str, sizeof(vbat_v_str), adc_data->vbat_V);
    E1_FormatFloat3(power_w_str, sizeof(power_w_str), adc_data->power_W);
    E1_FormatFloat3(zero_offset_str, sizeof(zero_offset_str), g_h1.baseline_zero_offset_voltage);
    E1_FormatFloat3(active_zero_offset_str, sizeof(active_zero_offset_str), adc_data->active_zero_offset_V);
    E1_FormatFloat3(delta_i_v_str, sizeof(delta_i_v_str), adc_data->delta_i_V);

    len = snprintf(line,
                   sizeof(line),
                   "h1:%lu,%u,%u,%u,%u,%u,%s,%s,%s,%s,%s,%s,%s,%s\r\n",
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
                   zero_offset_str,
                   active_zero_offset_str,
                   delta_i_v_str);

    if (len > 0)
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)line, (uint16_t)len, E1_UART_TX_TIMEOUT_MS);
    }
}

static void E1_Test_EnterState(E1_TestState_t new_state)
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
        E1_ResetCurrentFilter();
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
            E1_Dshot_TriggerFrame(g_h1.current_dshot_value);
        }
    }

    E1_SendStateChangeLine(new_state);
    E1_StatusLed_Update(g_h1.state_enter_tick);
}

static void E1_StartSession(uint32_t now_ms)
{
    g_h1.test_start_tick = now_ms;
    g_h1.last_csv_tick = now_ms;
    g_h1.last_zero_offset_sample_tick = now_ms;
    g_h1.zero_offset_sum_v = 0.0f;
    g_h1.zero_offset_sample_count = 0U;
    g_h1.baseline_zero_offset_voltage = CURRENT_OFFSET_V;
    g_h1.active_zero_offset_voltage = CURRENT_OFFSET_V;
    E1_ResetCurrentFilter();
    g_h1.start_command_received = 0U;
    g_h1.start_armed_once = 0U;
    g_h1.header_sent = 0U;

    E1_Test_EnterState(STATE_PREPARE);
    E1_SendCsvHeaderOnce();
}

static uint8_t E1_IsBluetoothConnected(void)
{
    return (HAL_GPIO_ReadPin(BT_STATE_GPIO_Port, BT_STATE_Pin) == E1_BT_CONNECTED_LEVEL) ? 1U : 0U;
}

static void E1_SendBootBannerOnce(void)
{
    static const char banner[] =
        "# boot,t_ms=0,fw=E1_DSHOT300_BASELINE_AVG,uart=9600,protocol=firewater\r\n";

    if (g_h1.boot_banner_sent == 0U)
    {
        E1_UartWrite(banner);
        g_h1.boot_banner_sent = 1U;
    }
}

static void E1_SendConnectedBannerOnce(void)
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
                   E1_Test_GetStateName(g_h1.state),
                   (unsigned long)E1_BT_PREPARE_MS,
                   (unsigned long)E1_BT_CONNECT_CONFIRM_MS,
                   (unsigned int)E1_RUN_THROTTLE_DSHOT);
    if (len > 0)
    {
        E1_UartWrite(line);
    }

    g_h1.connected_banner_sent = 1U;
}

static void E1_SendWaitStartBannerOnce(void)
{
    static const char line[] =
        "# waiting,start_cmd=START,send_newline=optional,action=prepare_after_start\r\n";

    if (g_h1.wait_start_banner_sent == 0U)
    {
        E1_UartWrite(line);
        g_h1.wait_start_banner_sent = 1U;
    }
}

static void E1_SendStartAck(void)
{
    static const char line[] =
        "# ack,start_received,action=enter_prepare\r\n";

    E1_UartWrite(line);
}

static void E1_SendCsvHeaderOnce(void)
{
    static const char header[] =
        "# firewater_channels,ch0=t_ms,ch1=state,ch2=cmd_raw,ch3=dshot_cmd,ch4=adc_i_raw,ch5=adc_vbat_raw,ch6=v_i_sense,ch7=v_vbat_adc,ch8=current_A,ch9=vbat_V,ch10=power_W,ch11=zero_offset_V,ch12=active_zero_offset_V,ch13=delta_i_V\r\n";

    if (g_h1.header_sent == 0U)
    {
        E1_UartWrite(header);
        g_h1.header_sent = 1U;
    }
}

static void E1_SendStateChangeLine(E1_TestState_t state)
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
                   E1_Test_GetStateName(state));
    if (len > 0)
    {
        E1_UartWrite(line);
    }
}

static void E1_UartWrite(const char *text)
{
    size_t len;

    if (text == NULL)
    {
        return;
    }

    len = strlen(text);
    if (len > 0U)
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)len, E1_UART_TX_TIMEOUT_MS);
    }
}

static void E1_FormatFloat3(char *dst, size_t len, float value)
{
    int32_t scaled;
    int32_t abs_scaled;
    int32_t integer_part;
    int32_t fractional_part;
    const char *sign_str;

    if ((dst == NULL) || (len == 0U))
    {
        return;
    }

    scaled = (int32_t)(value * 1000.0f + ((value >= 0.0f) ? 0.5f : -0.5f));
    sign_str = (scaled < 0) ? "-" : "";
    abs_scaled = (scaled < 0) ? -scaled : scaled;
    integer_part = abs_scaled / 1000;
    fractional_part = abs_scaled % 1000;

    (void)snprintf(dst, len, "%s%ld.%03ld", sign_str, (long)integer_part, (long)fractional_part);
}

static void E1_Button_Task(uint32_t now_ms)
{
    uint8_t raw_pressed;

    raw_pressed = (HAL_GPIO_ReadPin(BTN_THROTTLE_GPIO_Port, BTN_THROTTLE_Pin) == GPIO_PIN_RESET) ? 1U : 0U;

    if (raw_pressed != g_h1.button_raw_pressed)
    {
        g_h1.button_raw_pressed = raw_pressed;
        g_h1.button_change_tick = now_ms;
    }

    if ((now_ms - g_h1.button_change_tick) < E1_BUTTON_DEBOUNCE_MS)
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
            E1_HandleThrottleButtonPress();
        }
    }
}

static void E1_HandleThrottleButtonPress(void)
{
    uint16_t next_cmd = g_h1.run_cmd_dshot;

    if ((next_cmd < E1_RUN_THROTTLE_MIN_DSHOT) || (next_cmd > E1_RUN_THROTTLE_MAX_DSHOT))
    {
        next_cmd = E1_RUN_THROTTLE_MIN_DSHOT;
    }
    else
    {
        next_cmd = (uint16_t)(next_cmd + E1_RUN_THROTTLE_STEP_DSHOT);
        if (next_cmd > E1_RUN_THROTTLE_MAX_DSHOT)
        {
            next_cmd = E1_RUN_THROTTLE_MIN_DSHOT;
        }
    }

    g_h1.run_cmd_dshot = next_cmd;
    E1_SendThrottleSetLine();
    g_h1.last_oled_update_tick = 0U;
    E1_Oled_Update(HAL_GetTick());
}

static void E1_SendThrottleSetLine(void)
{
    char line[64];
    int len;

    len = snprintf(line,
                   sizeof(line),
                   "# throttle_set,run_cmd_raw=%u\r\n",
                   (unsigned int)g_h1.run_cmd_dshot);
    if (len > 0)
    {
        E1_UartWrite(line);
    }
}

static HAL_StatusTypeDef E1_Oled_WriteCommand(uint8_t command)
{
    uint8_t frame[2];

    if (g_h1.oled_ready == 0U)
    {
        return HAL_ERROR;
    }

    frame[0] = 0x00U;
    frame[1] = command;
    return HAL_I2C_Master_Transmit(&hi2c1, E1_OLED_I2C_ADDR, frame, 2U, 50U);
}

static HAL_StatusTypeDef E1_Oled_WriteData(const uint8_t *data, uint16_t size)
{
    uint8_t frame[129];

    if ((g_h1.oled_ready == 0U) || (data == NULL) || (size > 128U))
    {
        return HAL_ERROR;
    }

    frame[0] = 0x40U;
    memcpy(&frame[1], data, size);
    return HAL_I2C_Master_Transmit(&hi2c1, E1_OLED_I2C_ADDR, frame, (uint16_t)(size + 1U), 50U);
}

static void E1_Oled_Init(void)
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

    if (HAL_I2C_IsDeviceReady(&hi2c1, E1_OLED_I2C_ADDR, 2U, 100U) != HAL_OK)
    {
        return;
    }

    g_h1.oled_ready = 1U;

    for (i = 0U; i < (uint8_t)(sizeof(init_cmds) / sizeof(init_cmds[0])); i++)
    {
        if (E1_Oled_WriteCommand(init_cmds[i]) != HAL_OK)
        {
            g_h1.oled_ready = 0U;
            return;
        }
    }

    E1_Oled_Clear();
}

static void E1_Oled_Service(uint32_t now_ms)
{
    if (g_h1.oled_ready != 0U)
    {
        return;
    }

    if (g_h1.last_oled_init_attempt_tick == 0U)
    {
        if (now_ms < E1_OLED_POWERUP_DELAY_MS)
        {
            return;
        }
    }
    else if ((now_ms - g_h1.last_oled_init_attempt_tick) < E1_OLED_RETRY_INTERVAL_MS)
    {
        return;
    }

    g_h1.last_oled_init_attempt_tick = now_ms;
    E1_Oled_Init();

    if (g_h1.oled_ready != 0U)
    {
        g_h1.last_oled_update_tick = 0U;
        E1_Oled_Update(now_ms);
    }
}

static void E1_Oled_Clear(void)
{
    if (g_h1.oled_ready == 0U)
    {
        return;
    }

    memset(g_oled_buffer, 0, sizeof(g_oled_buffer));
    E1_Oled_Flush();
}

static void E1_Oled_Flush(void)
{
    uint8_t page;

    if (g_h1.oled_ready == 0U)
    {
        return;
    }

    for (page = 0U; page < 8U; page++)
    {
        (void)E1_Oled_WriteCommand((uint8_t)(0xB0U | page));
        (void)E1_Oled_WriteCommand((uint8_t)(0x00U | (E1_OLED_COLUMN_OFFSET & 0x0FU)));
        (void)E1_Oled_WriteCommand((uint8_t)(0x10U | ((E1_OLED_COLUMN_OFFSET >> 4U) & 0x0FU)));
        (void)E1_Oled_WriteData(&g_oled_buffer[page * 128U], 128U);
    }
}

static void E1_Oled_Update(uint32_t now_ms)
{
    char line0[32];
    char line1[32];
    char line2[32];
    char line3[32];
    char line4[32];
    char value_str[20];
    uint32_t prepare_elapsed_ms;
    uint32_t prepare_remaining_s;
    uint32_t run_elapsed_ms;
    uint32_t run_elapsed_s;
    char prompt_char = '\0';

    if (g_h1.oled_ready == 0U)
    {
        return;
    }

    if ((g_h1.last_oled_update_tick != 0U) &&
        ((now_ms - g_h1.last_oled_update_tick) < E1_OLED_UPDATE_INTERVAL_MS))
    {
        return;
    }

    g_h1.last_oled_update_tick = now_ms;
    memset(g_oled_buffer, 0, sizeof(g_oled_buffer));

    E1_FormatFloat3(value_str, sizeof(value_str), g_h1.adc.vbat_V);
    (void)snprintf(line0, sizeof(line0), "VBAT %sV", value_str);

    E1_FormatFloat3(value_str, sizeof(value_str), g_h1.adc.current_A);
    (void)snprintf(line1, sizeof(line1), "CURR %sA", value_str);

    E1_FormatFloat3(value_str, sizeof(value_str), g_h1.adc.power_W);
    (void)snprintf(line2, sizeof(line2), "POWR %sW", value_str);

    if (g_h1.state == STATE_PREPARE)
    {
        prepare_elapsed_ms = now_ms - g_h1.state_enter_tick;
        if (prepare_elapsed_ms >= E1_BT_PREPARE_MS)
        {
            prepare_remaining_s = 0U;
        }
        else
        {
            prepare_remaining_s = (E1_BT_PREPARE_MS - prepare_elapsed_ms + 999U) / 1000U;
        }
    }
    else
    {
        prepare_remaining_s = 0U;
    }

    if (g_h1.state == STATE_RUN)
    {
        run_elapsed_ms = now_ms - g_h1.state_enter_tick;
    }
    else if ((g_h1.state == STATE_STOP) || (g_h1.state == STATE_DONE) || (g_h1.state == STATE_IDLE))
    {
        run_elapsed_ms = E1_RUN_MS;
    }
    else
    {
        run_elapsed_ms = 0U;
    }

    run_elapsed_s = run_elapsed_ms / 1000U;
    (void)snprintf(line3, sizeof(line3), "THR  %u", (unsigned int)g_h1.run_cmd_dshot);
    if (g_h1.state == STATE_PREPARE)
    {
        (void)snprintf(line4, sizeof(line4), "PREP %2luS", (unsigned long)prepare_remaining_s);
    }
    else
    {
        (void)snprintf(line4, sizeof(line4), "TIME %2luS", (unsigned long)run_elapsed_s);
    }

    if ((g_h1.state == STATE_RUN) && (((now_ms / 300U) % 2U) == 0U))
    {
        if ((run_elapsed_ms >= 25000U) && (run_elapsed_ms < 30000U))
        {
            prompt_char = 'R';
        }
        else if ((run_elapsed_ms >= 35000U) && (run_elapsed_ms < 40000U))
        {
            prompt_char = 'H';
        }
    }

    E1_Oled_DrawLine(0U, line0);
    E1_Oled_DrawLine(1U, line1);
    E1_Oled_DrawLine(2U, line2);
    E1_Oled_DrawLine(3U, line3);
    E1_Oled_DrawLine(4U, line4);
    if (prompt_char != '\0')
    {
        E1_Oled_DrawChar(116U, 55U, prompt_char);
    }
    E1_Oled_Flush();
}

static void E1_Oled_DrawLine(uint8_t line_index, const char *text)
{
    uint8_t x = 0U;
    uint8_t y;

    if ((g_h1.oled_ready == 0U) || (line_index >= 5U))
    {
        return;
    }

    y = (uint8_t)(line_index * 11U);

    while ((text != NULL) && (*text != '\0') && (x <= 120U))
    {
        E1_Oled_DrawChar(x, y, *text);
        x = (uint8_t)(x + 7U);
        text++;
    }
}

static void E1_Oled_DrawChar(uint8_t x, uint8_t y, char c)
{
    uint8_t glyph[5];
    uint8_t col;
    uint8_t row;

    E1_Oled_GetGlyph5x7(c, glyph);

    for (col = 0U; col < 5U; col++)
    {
        for (row = 0U; row < 7U; row++)
        {
            if ((glyph[col] & (uint8_t)(1U << row)) != 0U)
            {
                E1_Oled_SetPixel((uint8_t)(x + col), (uint8_t)(y + row), 1U);
                E1_Oled_SetPixel((uint8_t)(x + col + 1U), (uint8_t)(y + row), 1U);
            }
        }
    }
}

static void E1_Oled_SetPixel(uint8_t x, uint8_t y, uint8_t on)
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

static void E1_Oled_GetGlyph5x7(char c, uint8_t glyph[5])
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
    case 'E': glyph[0] = 0x7FU; glyph[1] = 0x49U; glyph[2] = 0x49U; glyph[3] = 0x49U; glyph[4] = 0x41U; break;
    case 'H': glyph[0] = 0x7FU; glyph[1] = 0x08U; glyph[2] = 0x08U; glyph[3] = 0x08U; glyph[4] = 0x7FU; break;
    case 'I': glyph[0] = 0x00U; glyph[1] = 0x41U; glyph[2] = 0x7FU; glyph[3] = 0x41U; glyph[4] = 0x00U; break;
    case 'M': glyph[0] = 0x7FU; glyph[1] = 0x02U; glyph[2] = 0x0CU; glyph[3] = 0x02U; glyph[4] = 0x7FU; break;
    case 'O': glyph[0] = 0x3EU; glyph[1] = 0x41U; glyph[2] = 0x41U; glyph[3] = 0x41U; glyph[4] = 0x3EU; break;
    case 'P': glyph[0] = 0x7FU; glyph[1] = 0x09U; glyph[2] = 0x09U; glyph[3] = 0x09U; glyph[4] = 0x06U; break;
    case 'R': glyph[0] = 0x7FU; glyph[1] = 0x09U; glyph[2] = 0x19U; glyph[3] = 0x29U; glyph[4] = 0x46U; break;
    case 'S': glyph[0] = 0x46U; glyph[1] = 0x49U; glyph[2] = 0x49U; glyph[3] = 0x49U; glyph[4] = 0x31U; break;
    case 'T': glyph[0] = 0x01U; glyph[1] = 0x01U; glyph[2] = 0x7FU; glyph[3] = 0x01U; glyph[4] = 0x01U; break;
    case 'U': glyph[0] = 0x3FU; glyph[1] = 0x40U; glyph[2] = 0x40U; glyph[3] = 0x40U; glyph[4] = 0x3FU; break;
    case 'V': glyph[0] = 0x1FU; glyph[1] = 0x20U; glyph[2] = 0x40U; glyph[3] = 0x20U; glyph[4] = 0x1FU; break;
    case 'W': glyph[0] = 0x7FU; glyph[1] = 0x20U; glyph[2] = 0x18U; glyph[3] = 0x20U; glyph[4] = 0x7FU; break;
    default: break;
    }
}

static void E1_StatusLed_Set(uint8_t on)
{
    HAL_GPIO_WritePin(STATUS_LED_GPIO_Port,
                      STATUS_LED_Pin,
                      (on != 0U) ? E1_STATUS_LED_ON_LEVEL : E1_STATUS_LED_OFF_LEVEL);
}

static void E1_StatusLed_Update(uint32_t now_ms)
{
    uint32_t phase_ms;

    switch (g_h1.state)
    {
    case STATE_WAIT_BT:
        phase_ms = now_ms % 2000U;
        E1_StatusLed_Set(((phase_ms < 80U) || ((phase_ms >= 250U) && (phase_ms < 330U))) ? 1U : 0U);
        break;

    case STATE_PREPARE:
        phase_ms = now_ms % 1000U;
        E1_StatusLed_Set((phase_ms < 120U) ? 1U : 0U);
        break;

    case STATE_RUN:
        E1_StatusLed_Set(1U);
        break;

    case STATE_STOP:
        phase_ms = now_ms % 250U;
        E1_StatusLed_Set((phase_ms < 125U) ? 1U : 0U);
        break;

    case STATE_DONE:
    default:
        E1_StatusLed_Set(0U);
        break;

    case STATE_IDLE:
        E1_StatusLed_Set(0U);
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

static uint16_t E1_MapUsToDshot(uint16_t us)
{
    uint32_t scaled;

    if (us <= E1_PWM_MIN_US)
    {
        return 0U;
    }

    if (us >= E1_PWM_MAX_US)
    {
        return E1_DSHOT_MAX_THROTTLE;
    }

    scaled = (uint32_t)(us - (E1_PWM_MIN_US + 1U)) * (uint32_t)(E1_DSHOT_MAX_THROTTLE - E1_DSHOT_MIN_THROTTLE);
    scaled /= (uint32_t)(E1_PWM_MAX_US - (E1_PWM_MIN_US + 1U));

    return (uint16_t)(E1_DSHOT_MIN_THROTTLE + scaled);
}

static uint16_t E1_ClampDshotThrottle(uint16_t cmd)
{
    if (cmd == 0U)
    {
        return 0U;
    }

    if (cmd < E1_DSHOT_MIN_THROTTLE)
    {
        return E1_DSHOT_MIN_THROTTLE;
    }

    if (cmd > E1_DSHOT_MAX_THROTTLE)
    {
        return E1_DSHOT_MAX_THROTTLE;
    }

    return cmd;
}

static uint16_t E1_Dshot_BuildPacket(uint16_t throttle_value)
{
    uint16_t packet;
    uint16_t checksum;
    uint16_t csum_data;
    uint8_t i;

    packet = (uint16_t)((throttle_value << 1U) | (E1_DSHOT_TELEMETRY_BIT & 0x1U));
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

static void E1_Dshot_PrepareFrame(uint16_t throttle_value)
{
    uint16_t packet;
    uint8_t i;

    packet = E1_Dshot_BuildPacket(throttle_value);

    for (i = 0U; i < E1_DSHOT_FRAME_BITS; i++)
    {
        g_dshot_dma_buffer[i] = ((packet & 0x8000U) != 0U) ? E1_DSHOT_BIT_1_HIGH_TICKS : E1_DSHOT_BIT_0_HIGH_TICKS;
        packet <<= 1U;
    }

    for (i = E1_DSHOT_FRAME_BITS; i < E1_DSHOT_DMA_BUFFER_LENGTH; i++)
    {
        g_dshot_dma_buffer[i] = 0U;
    }
}

static void E1_Dshot_TriggerFrame(uint16_t throttle_value)
{
    HAL_StatusTypeDef status;

    if (g_h1.dshot_dma_busy != 0U)
    {
        return;
    }

    E1_Dshot_PrepareFrame(throttle_value);
    g_h1.dshot_dma_busy = 1U;

    __HAL_TIM_DISABLE(&htim3);
    __HAL_TIM_SET_COUNTER(&htim3, 0U);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, g_dshot_dma_buffer[0]);

    status = HAL_TIM_PWM_Start_DMA(&htim3,
                                   TIM_CHANNEL_1,
                                   (const uint32_t *)&g_dshot_dma_buffer[1],
                                   (uint16_t)(E1_DSHOT_DMA_BUFFER_LENGTH - 1U));
    if (status != HAL_OK)
    {
        g_h1.dshot_dma_busy = 0U;
        Error_Handler();
    }
}

static void E1_Dshot_Service(uint32_t now_ms)
{
    if ((now_ms - g_h1.last_dshot_send_tick) < E1_DSHOT_SEND_INTERVAL_MS)
    {
        return;
    }

    if (g_h1.dshot_dma_busy != 0U)
    {
        return;
    }

    g_h1.last_dshot_send_tick = now_ms;
    E1_Dshot_TriggerFrame(g_h1.current_dshot_value);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        E1_ProcessReceivedByte(g_uart_rx_byte);
        E1_UartRx_Start();
    }
}

static void E1_UartRx_Start(void)
{
    if (HAL_UART_Receive_IT(&huart1, &g_uart_rx_byte, 1U) != HAL_OK)
    {
        Error_Handler();
    }
}

static void E1_ProcessReceivedByte(uint8_t byte)
{
    char c = (char)byte;

    if ((c == '\r') || (c == '\n'))
    {
        if (g_uart_cmd_index > 0U)
        {
            g_uart_cmd_buffer[g_uart_cmd_index] = '\0';
            if ((g_h1.state == STATE_WAIT_BT) &&
                (g_h1.bt_confirmed != 0U) &&
                (E1_IsStartCommand(g_uart_cmd_buffer) != 0U))
            {
                E1_ProcessStartCommand();
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
                (E1_IsStartCommand(g_uart_cmd_buffer) != 0U))
            {
                E1_ProcessStartCommand();
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

static uint8_t E1_IsStartCommand(const char *text)
{
    if (text == NULL)
    {
        return 0U;
    }

    return (strcmp(text, "START") == 0) ? 1U : 0U;
}

static void E1_ProcessStartCommand(void)
{
    if (g_h1.start_armed_once == 0U)
    {
        g_h1.start_command_received = 1U;
        g_h1.start_armed_once = 1U;
        E1_SendStartAck();
    }
}
