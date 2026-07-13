#include "tim_sync_test.h"
#include <stdio.h>
#include "../bsp/tim.h"
#include "../utils/print.h"

#define TIM_SYNC_TEST_SUMMARY_PERIOD_MS (200U)
#define TIM_SYNC_TEST_SUMMARY_SAMPLES (512U)

static uint32_t tim_sync_calc_signed_diff(uint32_t cnt_master, uint32_t cnt_slave) {
    int32_t diff      = (int32_t)cnt_slave - (int32_t)cnt_master;
    int32_t period    = (int32_t)TIM1_PERIOD;
    int32_t full_span = period * 2;

    if (diff > period) {
        diff -= full_span;
    } else if (diff < -period) {
        diff += full_span;
    }

    return (uint32_t)diff;
}

void tim_sync_test_init(void) {
    tim_init();
    tim_set_pwmDuty(0.5f, 0.5f, 0.5f);
}

void tim_sync_test_poll(void) {
    static uint32_t sample_count       = 0;
    static int32_t  diff_min           = INT32_MAX;
    static int32_t  diff_max           = INT32_MIN;
    static int64_t  diff_sum           = 0;
    static uint32_t dir_mismatch_count = 0;
    uint32_t        cnt_tim2           = __HAL_TIM_GET_COUNTER(&htim2);
    uint32_t        cnt_tim3           = __HAL_TIM_GET_COUNTER(&htim3);
    uint32_t        dir_tim2           = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2) ? 1U : 0U;
    uint32_t        dir_tim3           = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3) ? 1U : 0U;
    int32_t         diff               = (int32_t)tim_sync_calc_signed_diff(cnt_tim2, cnt_tim3);
    float           debug_data[5];

    debug_data[0] = (float)cnt_tim2;
    debug_data[1] = (float)cnt_tim3;
    debug_data[2] = (float)diff;
    debug_data[3] = (float)dir_tim2;
    debug_data[4] = (float)dir_tim3;

    vofa_send(debug_data, 5);

    sample_count++;

    if (diff < diff_min) {
        diff_min = diff;
    }

    if (diff > diff_max) {
        diff_max = diff;
    }

    diff_sum += diff;

    if (dir_tim2 != dir_tim3) {
        dir_mismatch_count++;
    }

    printf("tim_sync samples=%lu last=(%lu,%lu) diff[last=%ld min=%ld max=%ld avg=%ld] dir_mismatch=%lu/%lu last_dir=%s/%s\r\n", (unsigned long)sample_count, (unsigned long)cnt_tim2,
           (unsigned long)cnt_tim3, (long)diff, (long)diff_min, (long)diff_max, (long)(diff_sum / (int64_t)sample_count), (unsigned long)dir_mismatch_count, (unsigned long)sample_count,
           dir_tim2 ? "down" : "up", dir_tim3 ? "down" : "up");

    if (sample_count >= TIM_SYNC_TEST_SUMMARY_SAMPLES) {
        sample_count       = 0;
        diff_min           = INT32_MAX;
        diff_max           = INT32_MIN;
        diff_sum           = 0;
        dir_mismatch_count = 0;
    }
}
