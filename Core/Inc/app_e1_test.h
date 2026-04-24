#ifndef APP_E1_TEST_H
#define APP_E1_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Experiment command interface kept in "us-like" semantics so it is still
 * easy to tune from the test script side even though the transport is DShot.
 */
#ifndef E1_PWM_MIN_US
#define E1_PWM_MIN_US                  1000U
#endif

#ifndef E1_PWM_MAX_US
#define E1_PWM_MAX_US                  2000U
#endif

/* DShot transport tuning. */
#ifndef E1_DSHOT_SEND_INTERVAL_MS
#define E1_DSHOT_SEND_INTERVAL_MS      1U
#endif

#ifndef E1_DSHOT_MIN_THROTTLE
#define E1_DSHOT_MIN_THROTTLE          48U
#endif

#ifndef E1_DSHOT_MAX_THROTTLE
#define E1_DSHOT_MAX_THROTTLE          2047U
#endif

#ifndef E1_DSHOT_TELEMETRY_BIT
#define E1_DSHOT_TELEMETRY_BIT         0U
#endif

/* Experiment timing. */
#ifndef E1_BT_PREPARE_MS
#define E1_BT_PREPARE_MS               10000U
#endif

#ifndef E1_BT_CONNECT_CONFIRM_MS
#define E1_BT_CONNECT_CONFIRM_MS       500U
#endif

#ifndef E1_RUN_THROTTLE_DSHOT
#define E1_RUN_THROTTLE_DSHOT          800U
#endif

#ifndef E1_RUN_THROTTLE_MIN_DSHOT
#define E1_RUN_THROTTLE_MIN_DSHOT      48U
#endif

#ifndef E1_RUN_THROTTLE_MAX_DSHOT
#define E1_RUN_THROTTLE_MAX_DSHOT      2047U
#endif

#ifndef E1_RUN_THROTTLE_STEP_DSHOT
#define E1_RUN_THROTTLE_STEP_DSHOT     100U
#endif

#ifndef E1_RAMP_START_DSHOT
#define E1_RAMP_START_DSHOT            800U
#endif

#ifndef E1_RAMP_MS
#define E1_RAMP_MS                     10000U
#endif

#ifndef E1_RUN_MS
#define E1_RUN_MS                      60000U
#endif

#ifndef E1_TEST_CYCLE_COUNT
#define E1_TEST_CYCLE_COUNT            3U
#endif

#ifndef E1_REST_MS
#define E1_REST_MS                     30000U
#endif

#ifndef E1_SESSION_MAX_MS
#define E1_SESSION_MAX_MS \
    (E1_BT_PREPARE_MS + (E1_TEST_CYCLE_COUNT * (E1_RAMP_MS + E1_RUN_MS)) + \
     ((E1_TEST_CYCLE_COUNT - 1U) * E1_REST_MS) + 10000U)
#endif

#ifndef E1_CSV_INTERVAL_MS
#define E1_CSV_INTERVAL_MS             1000U
#endif

#ifndef E1_UART_BOOT_QUIET_MS
#define E1_UART_BOOT_QUIET_MS          2000U
#endif

#ifndef E1_UART_TX_BUFFER_SIZE
#define E1_UART_TX_BUFFER_SIZE         1024U
#endif

#ifndef E1_CURRENT_TRIP_A
#define E1_CURRENT_TRIP_A              55.0f
#endif

#ifndef E1_CURRENT_TRIP_HOLD_MS
#define E1_CURRENT_TRIP_HOLD_MS        100U
#endif

#ifndef E1_ZERO_OFFSET_SAMPLE_INTERVAL_MS
#define E1_ZERO_OFFSET_SAMPLE_INTERVAL_MS 10U
#endif

#ifndef E1_BASELINE_SAMPLE_COUNT
#define E1_BASELINE_SAMPLE_COUNT       600U
#endif

#ifndef E1_CURRENT_FILTER_SAMPLE_INTERVAL_MS
#define E1_CURRENT_FILTER_SAMPLE_INTERVAL_MS 10U
#endif

#ifndef E1_CURRENT_FILTER_WINDOW_MS
#define E1_CURRENT_FILTER_WINDOW_MS    250U
#endif

#define E1_CURRENT_FILTER_WINDOW_SAMPLES \
    (E1_CURRENT_FILTER_WINDOW_MS / E1_CURRENT_FILTER_SAMPLE_INTERVAL_MS)

#ifndef E1_CURRENT_SIGN_INVERT
#define E1_CURRENT_SIGN_INVERT         0U
#endif

#ifndef E1_BUTTON_DEBOUNCE_MS
#define E1_BUTTON_DEBOUNCE_MS          30U
#endif

#ifndef E1_OLED_UPDATE_INTERVAL_MS
#define E1_OLED_UPDATE_INTERVAL_MS     200U
#endif

#ifndef E1_OLED_I2C_TIMEOUT_MS
#define E1_OLED_I2C_TIMEOUT_MS         25U
#endif

#ifndef E1_OLED_POWERUP_DELAY_MS
#define E1_OLED_POWERUP_DELAY_MS       120U
#endif

#ifndef E1_OLED_RETRY_INTERVAL_MS
#define E1_OLED_RETRY_INTERVAL_MS      500U
#endif

#ifndef E1_OLED_I2C_ADDR
#define E1_OLED_I2C_ADDR               (0x3CU << 1)
#endif

#ifndef E1_OLED_COLUMN_OFFSET
#define E1_OLED_COLUMN_OFFSET          2U
#endif

/* ADC and analog conversion constants. */
#ifndef ADC_REF_VOLTAGE
#define ADC_REF_VOLTAGE                3.3f
#endif

#ifndef ADC_MAX_COUNTS
#define ADC_MAX_COUNTS                 4095.0f
#endif

/* Measured divider ratio using 9.98k upper and 992R lower resistor. */
#ifndef VBAT_DIVIDER_RATIO
#define VBAT_DIVIDER_RATIO             11.060484f
#endif

#ifndef CURRENT_OFFSET_V
#define CURRENT_OFFSET_V               0.0f
#endif

/* INA199 gain options: A1=50, A2=100, A3=200 V/V. */
#ifndef INA199_GAIN_V_V
#define INA199_GAIN_V_V                200.0f
#endif

/* Keep this in sync with the actual shunt resistor on the board. */
#ifndef CURRENT_SHUNT_RESISTANCE_OHM
#define CURRENT_SHUNT_RESISTANCE_OHM   0.00025f
#endif

#ifndef CURRENT_SCALE_A_PER_V
#define CURRENT_SCALE_A_PER_V          (1.0f / (INA199_GAIN_V_V * CURRENT_SHUNT_RESISTANCE_OHM))
#endif

typedef enum
{
    STATE_WAIT_BT = 0,
    STATE_PREPARE = 1,
    STATE_RUN     = 2,
    STATE_STOP    = 3,
    STATE_DONE    = 4,
    STATE_IDLE    = 5,
    STATE_RAMP    = 6
} E1_TestState_t;

typedef struct
{
    uint16_t adc_i_raw;
    uint16_t adc_vbat_raw;
    float v_i_sense;
    float v_vbat_adc;
    float delta_i_V;
    float active_zero_offset_V;
    float current_A;
    float vbat_V;
    float power_W;
} E1_AdcProcessed_t;

void E1_Test_Init(void);
void E1_Test_Task(void);
const char *E1_Test_GetStateName(E1_TestState_t state);
void set_throttle_us(uint16_t us);
void set_throttle_command(uint16_t cmd);
void process_adc_average(E1_AdcProcessed_t *out);
void update_zero_offset(const E1_AdcProcessed_t *adc_data, uint32_t now_ms);
void send_e1_csv_line(const E1_AdcProcessed_t *adc_data);

#ifdef __cplusplus
}
#endif

#endif /* APP_E1_TEST_H */
