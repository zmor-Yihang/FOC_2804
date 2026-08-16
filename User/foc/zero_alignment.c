#include <math.h>
#include "../sensor/encoder.h"
#include "foc.h"
#include "gate_drive.h"

void zero_alignment(foc_t *handle) {
    float    sin_sum = 0.0f;
    float    cos_sum = 0.0f;
    uint16_t sample_idx;
    uint8_t  repeat_idx;
    dq_t     u_dq = {.d = FOC_ALIGN_D_AXIS_VOLTAGE, .q = 0.0f};

    // 把电机拉到 d 轴
    handle->duty_cycle = gateDrive_set_voltage(ipark_transform(u_dq, 0.0f));
    HAL_Delay(FOC_ALIGN_SETTLE_TIME_MS);

    // 缓慢扫描一整圈电角度，累计每个采样点的偏移圆均值
    for (repeat_idx = 0U; repeat_idx < FOC_ALIGN_SCAN_REPEAT; repeat_idx++) {
        for (sample_idx = 0U; sample_idx < FOC_ALIGN_SCAN_POINTS; sample_idx++) {
            // 把一圈电角度均匀分成FOC_ALIGN_SCAN_POINTS份，计算当前采样点的目标电角度
            float angle_cmd = (MATH_TWO_PI * (float)sample_idx) / (float)FOC_ALIGN_SCAN_POINTS;
            float angle_meas;    // 实际测量角度
            float offset_sample; // 当前采样点与目标电角度之差
            float sin_offset;    // 当前采样点与目标电角度之差对应的正弦值
            float cos_offset;    // 当前采样点与目标电角度之差对应的余弦值

            handle->duty_cycle = gateDrive_set_voltage(ipark_transform(u_dq, angle_cmd));
            HAL_Delay(FOC_ALIGN_SAMPLE_INTERVAL_MS);

            // 阻塞读取AS5600，此处使用的就是实际测量角度
            angle_meas = encoder_get_mechanicalAngleBlock();
            angle_meas *= MOTOR_POLE_PAIRS;
            offset_sample = wrap_0_2pi(angle_meas - angle_cmd);

            // 偏移角可能跨越0/2π边界，直接对角度做算术平均会得到错误结果
            // 将每个偏移角映射为单位圆向量(cosθ, sinθ)并累加，最后用atan2求圆均值
            fast_sin_cos(offset_sample, &sin_offset, &cos_offset);
            sin_sum += sin_offset;
            cos_sum += cos_offset;
        }

        // 每圈结束后回到零角，减少下一圈起点跳变
        handle->duty_cycle = gateDrive_set_voltage(ipark_transform(u_dq, 0.0f));
        HAL_Delay(FOC_ALIGN_SETTLE_TIME_MS);
    }

    handle->angle_offset = wrap_0_2pi(atan2f(sin_sum, cos_sum) - FOC_ELEC_ANGLE_TRIM_RAD);

    HAL_Delay(10);    // 确保I2C通信完成
    encoder_update(); // 刷新PLL状态

    // 关闭PWM输出
    handle->duty_cycle = gateDrive_stop();
}
