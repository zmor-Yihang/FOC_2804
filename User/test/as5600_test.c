#include "as5600_test.h"

#include <math.h>

#define AS5600_TEST_ALIGN_VOLTAGE (FOC_ALIGN_D_AXIS_VOLTAGE)
#define AS5600_TEST_MECH_SCAN_POINTS (128U)
#define AS5600_TEST_SETTLE_TIME_MS (120U)
#define AS5600_TEST_SAMPLE_COUNT (8U)
#define AS5600_TEST_STEP_PERIOD_MS (150U)
#define AS5600_TEST_RAW_TO_RAD (MATH_TWO_PI / (float)ENCODER_CPR)

typedef enum
{
    AS5600_TEST_STATE_SETTLE = 0,
    AS5600_TEST_STATE_SAMPLE,
    AS5600_TEST_STATE_DONE
} as5600_test_state_e;

static as5600_test_state_e as5600_test_state = AS5600_TEST_STATE_SETTLE;
static uint16_t as5600_test_step_idx = 0U;
static uint16_t as5600_test_sample_idx = 0U;
static uint32_t as5600_test_state_tick = 0U;
static float as5600_test_sin_sum = 0.0f;
static float as5600_test_cos_sum = 0.0f;
static float as5600_test_err_min = 1.0e9f;
static float as5600_test_err_max = -1.0e9f;
static float as5600_test_err_abs_max = 0.0f;
static float as5600_test_cmd_mech_rad = 0.0f;
static float as5600_test_cmd_elec_rad = 0.0f;

static void as5600_test_apply_step(uint16_t step_idx)
{
    dq_t u_dq = {.d = AS5600_TEST_ALIGN_VOLTAGE, .q = 0.0f};

    as5600_test_cmd_mech_rad = (MATH_TWO_PI * (float)step_idx) / (float)AS5600_TEST_MECH_SCAN_POINTS;
    as5600_test_cmd_elec_rad = as5600_test_cmd_mech_rad * (float)MOTOR_POLE_PAIRS;
    gateDrive_set_voltage(ipark_transform(u_dq, as5600_test_cmd_elec_rad));
}

static uint8_t as5600_test_read_raw(uint16_t *raw_count)
{
    uint32_t start_tick = HAL_GetTick();

    while ((HAL_GetTick() - start_tick) < 20U)
    {
        if (as5600_read_rawCountAsync(raw_count) != 0U)
        {
            return 1U;
        }
    }

    return 0U;
}

void as5600_test_init(void)
{
    as5600_init();

    as5600_test_state = AS5600_TEST_STATE_SETTLE;
    as5600_test_step_idx = 0U;
    as5600_test_sample_idx = 0U;
    as5600_test_state_tick = HAL_GetTick();
    as5600_test_sin_sum = 0.0f;
    as5600_test_cos_sum = 0.0f;
    as5600_test_err_min = 1.0e9f;
    as5600_test_err_max = -1.0e9f;
    as5600_test_err_abs_max = 0.0f;

    as5600_test_apply_step(0U);

    printf("as5600 step scan start: points=%u pole_pairs=%u voltage=%.3fV\r\n",
           AS5600_TEST_MECH_SCAN_POINTS,
           MOTOR_POLE_PAIRS,
           (double)AS5600_TEST_ALIGN_VOLTAGE);
}

void as5600_test_poll(void)
{
    uint16_t raw_count;
    float debug_data[5];

    if (as5600_test_state == AS5600_TEST_STATE_DONE)
    {
        return;
    }

    if (as5600_test_state == AS5600_TEST_STATE_SETTLE)
    {
        if ((HAL_GetTick() - as5600_test_state_tick) >= AS5600_TEST_SETTLE_TIME_MS)
        {
            as5600_test_sample_idx = 0U;
            as5600_test_sin_sum = 0.0f;
            as5600_test_cos_sum = 0.0f;
            as5600_test_state = AS5600_TEST_STATE_SAMPLE;
        }
        return;
    }

    if (as5600_test_read_raw(&raw_count) == 0U)
    {
        return;
    }

    float meas_mech_rad = (float)raw_count * AS5600_TEST_RAW_TO_RAD;
    float err_rad = wrap_neg_pi_to_pi(meas_mech_rad - as5600_test_cmd_mech_rad);
    float sin_err;
    float cos_err;

    fast_sin_cos(err_rad, &sin_err, &cos_err);
    as5600_test_sin_sum += sin_err;
    as5600_test_cos_sum += cos_err;
    as5600_test_sample_idx++;

    debug_data[0] = (float)raw_count;
    debug_data[1] = as5600_test_cmd_mech_rad;
    debug_data[2] = meas_mech_rad;
    debug_data[3] = err_rad;
    debug_data[4] = (float)as5600_test_step_idx;
    vofa_send(debug_data, 5);

    if (as5600_test_sample_idx < AS5600_TEST_SAMPLE_COUNT)
    {
        return;
    }

    float err_avg_rad = atan2f(as5600_test_sin_sum, as5600_test_cos_sum);
    float err_abs_rad = fabsf(err_avg_rad);

    if (err_avg_rad < as5600_test_err_min)
    {
        as5600_test_err_min = err_avg_rad;
    }

    if (err_avg_rad > as5600_test_err_max)
    {
        as5600_test_err_max = err_avg_rad;
    }

    if (err_abs_rad > as5600_test_err_abs_max)
    {
        as5600_test_err_abs_max = err_abs_rad;
    }

    printf("as5600 step=%u raw=%u cmd=%.6f meas=%.6f err=%.6f err_deg=%.3f\r\n",
           as5600_test_step_idx,
           raw_count,
           (double)as5600_test_cmd_mech_rad,
           (double)meas_mech_rad,
           (double)err_avg_rad,
           (double)(err_avg_rad * 180.0f / MATH_PI));

    as5600_test_step_idx++;

    if (as5600_test_step_idx >= AS5600_TEST_MECH_SCAN_POINTS)
    {
        gateDrive_stop();
        as5600_test_state = AS5600_TEST_STATE_DONE;
        printf("as5600 scan done: err[min=%.6f max=%.6f pp=%.6f abs_max=%.6f] rad, pp_deg=%.3f\r\n",
               (double)as5600_test_err_min,
               (double)as5600_test_err_max,
               (double)(as5600_test_err_max - as5600_test_err_min),
               (double)as5600_test_err_abs_max,
               (double)((as5600_test_err_max - as5600_test_err_min) * 180.0f / MATH_PI));
        return;
    }

    as5600_test_apply_step(as5600_test_step_idx);
    as5600_test_state_tick = HAL_GetTick();
    as5600_test_state = AS5600_TEST_STATE_SETTLE;
}
