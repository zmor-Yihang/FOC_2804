#include <math.h>

#include "foc.h"
#include "gate_drive.h"
#include "../sensor/encoder.h"

void zero_alignment(foc_t *handle)
{
    float sin_sum = 0.0f;
    float cos_sum = 0.0f;
    uint16_t sample_idx;
    uint8_t repeat_idx;
    dq_t u_dq = {.d = FOC_ALIGN_D_AXIS_VOLTAGE, .q = 0.0f};

    /* 把电机拉到 d 轴 */
    handle->duty_cycle = gateDrive_set_voltage(ipark_transform(u_dq, 0.0f));
    HAL_Delay(FOC_ALIGN_SETTLE_TIME_MS);

    /* 缓慢扫描一整圈电角度，累计每个采样点的偏移圆均值 */
    for (repeat_idx = 0U; repeat_idx < FOC_ALIGN_SCAN_REPEAT; repeat_idx++)
    {
        for (sample_idx = 0U; sample_idx < FOC_ALIGN_SCAN_POINTS; sample_idx++)
        {
            float angle_cmd = (MATH_TWO_PI * (float)sample_idx) / (float)FOC_ALIGN_SCAN_POINTS;
            float angle_meas;
            float offset_sample;
            float sin_offset;
            float cos_offset;

            handle->duty_cycle = gateDrive_set_voltage(ipark_transform(u_dq, angle_cmd));
            HAL_Delay(FOC_ALIGN_SAMPLE_INTERVAL_MS);

            /* 阻塞读取AS5600，确保此处使用的就是实际测量角度，不走PLL预测值 */
            angle_meas = encoder_get_mechanicalAngleBlock();
            angle_meas *= MOTOR_POLE_PAIRS;
            offset_sample = wrap_0_2pi(angle_meas - angle_cmd);

            fast_sin_cos(offset_sample, &sin_offset, &cos_offset);
            sin_sum += sin_offset;
            cos_sum += cos_offset;
        }

        /* 每圈结束后回到零角，减少下一圈起点跳变 */
        handle->duty_cycle = gateDrive_set_voltage(ipark_transform(u_dq, 0.0f));
        HAL_Delay(FOC_ALIGN_SETTLE_TIME_MS);
    }

    handle->angle_offset = wrap_0_2pi(atan2f(sin_sum, cos_sum) - FOC_ELEC_ANGLE_TRIM_RAD);

    HAL_Delay(10);     /* 确保I2C通信完成 */
    encoder_update(); /* 刷新PLL状态 */

    /* 关闭PWM输出 */
    handle->duty_cycle = gateDrive_stop();
}
