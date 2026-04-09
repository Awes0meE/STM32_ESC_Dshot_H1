#ifndef APP_H1_TEST_H
#define APP_H1_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Experiment command interface kept in "us-like" semantics so it is still
 * easy to tune from the test script side even though the transport is DShot.
 */
#ifndef H1_PWM_MIN_US
#define H1_PWM_MIN_US                  1000U
#endif

#ifndef H1_PWM_MAX_US
#define H1_PWM_MAX_US                  2000U
#endif

/* DShot transport tuning. */
#ifndef H1_DSHOT_SEND_INTERVAL_MS
#define H1_DSHOT_SEND_INTERVAL_MS      1U
#endif

#ifndef H1_DSHOT_MIN_THROTTLE
#define H1_DSHOT_MIN_THROTTLE          48U
#endif

#ifndef H1_DSHOT_MAX_THROTTLE
#define H1_DSHOT_MAX_THROTTLE          2047U
#endif

#ifndef H1_DSHOT_TELEMETRY_BIT
#define H1_DSHOT_TELEMETRY_BIT         0U
#endif

/* Experiment timing. */
#ifndef H1_BT_PREPARE_MS
#define H1_BT_PREPARE_MS               10000U
#endif

#ifndef H1_BT_CONNECT_CONFIRM_MS
#define H1_BT_CONNECT_CONFIRM_MS       500U
#endif

#ifndef H1_RUN_THROTTLE_DSHOT
#define H1_RUN_THROTTLE_DSHOT          500U
#endif

#ifndef H1_RUN_THROTTLE_MIN_DSHOT
#define H1_RUN_THROTTLE_MIN_DSHOT      48U
#endif

#ifndef H1_RUN_THROTTLE_MAX_DSHOT
#define H1_RUN_THROTTLE_MAX_DSHOT      2047U
#endif

#ifndef H1_RUN_THROTTLE_STEP_DSHOT
#define H1_RUN_THROTTLE_STEP_DSHOT     50U
#endif

#ifndef H1_RUN_MS
#define H1_RUN_MS                      60000U
#endif

#ifndef H1_STOP_MS
#define H1_STOP_MS                     10000U
#endif

#ifndef H1_SESSION_MAX_MS
#define H1_SESSION_MAX_MS              100000U
#endif

#ifndef H1_CSV_INTERVAL_MS
#define H1_CSV_INTERVAL_MS             500U
#endif

#ifndef H1_ZERO_OFFSET_SAMPLE_INTERVAL_MS
#define H1_ZERO_OFFSET_SAMPLE_INTERVAL_MS 10U
#endif

#ifndef H1_BUTTON_DEBOUNCE_MS
#define H1_BUTTON_DEBOUNCE_MS          30U
#endif

#ifndef H1_OLED_UPDATE_INTERVAL_MS
#define H1_OLED_UPDATE_INTERVAL_MS     200U
#endif

#ifndef H1_OLED_I2C_ADDR
#define H1_OLED_I2C_ADDR               (0x3CU << 1)
#endif

#ifndef H1_OLED_COLUMN_OFFSET
#define H1_OLED_COLUMN_OFFSET          2U
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

#ifndef CURRENT_SCALE_A_PER_V
#define CURRENT_SCALE_A_PER_V          80.0f
#endif

typedef enum
{
    STATE_WAIT_BT = 0,
    STATE_PREPARE = 1,
    STATE_RUN     = 2,
    STATE_STOP    = 3,
    STATE_DONE    = 4,
    STATE_IDLE    = 5
} H1_TestState_t;

typedef struct
{
    uint16_t adc_i_raw;
    uint16_t adc_vbat_raw;
    float v_i_sense;
    float v_vbat_adc;
    float current_A;
    float vbat_V;
    float power_W;
} H1_AdcProcessed_t;

void H1_Test_Init(void);
void H1_Test_Task(void);
const char *H1_Test_GetStateName(H1_TestState_t state);
void set_throttle_us(uint16_t us);
void set_throttle_command(uint16_t cmd);
void process_adc_average(H1_AdcProcessed_t *out);
void update_zero_offset(const H1_AdcProcessed_t *adc_data, uint32_t now_ms);
void send_h1_csv_line(const H1_AdcProcessed_t *adc_data);

#ifdef __cplusplus
}
#endif

#endif /* APP_H1_TEST_H */
