#ifndef __COGGING_CALIBRATION_H__
#define __COGGING_CALIBRATION_H__

#include "stm32g4xx_hal.h"
#include "../app/user_config.h"

#if (COGGING_CALIB_ENABLE != 0U)

#include "../alg/pid.h"
#include <string.h>
#include "../utils/angle_utils.h"
#include "../utils/math_utils.h"

typedef enum
{
    COGGING_CALIB_STATE_IDLE = 0, // 空闲，尚未启动标定
    COGGING_CALIB_STATE_SETTLING, // 正在等待当前目标角度稳定
    COGGING_CALIB_STATE_DONE      // 整张补偿表已经标定完成
} cogging_calib_state_t;

typedef struct
{
    uint8_t enabled;        // 标定器使能标志，1=运行，0=关闭
    uint8_t finished;       // 标定完成标志，1=整张表已采完
    uint8_t repeat_count;   // 当前正在进行的重复采样圈次计数
    uint16_t index;         // 目标采样点序号
    uint16_t tick_count;    // 当前状态下已累计的控制周期计数
    float start_angle_rad;  // 标定起始机械角(rad)
    float target_angle_rad; // 当前正在逼近/保持的目标机械角(rad)
    float target_iq;        // 当前输出给电流环的目标 q 轴电流(A)

    uint32_t raw_count_accum[COGGING_CALIB_TABLE_SIZE]; // 标定过程中，落入 bin k 的次数（计数器）
    float iq_comp_accum[COGGING_CALIB_TABLE_SIZE];      // 标定过程中，落入 bin k 的所有 iq 之和
    uint16_t raw_count_table[COGGING_CALIB_TABLE_SIZE]; // 最终结果表：bin k 对应的网格点 raw count（k*8，0~4088）
    float iq_comp_table[COGGING_CALIB_TABLE_SIZE];      // 最终结果表：bin k 的平均补偿 iq = iq_comp_accum[k] / raw_count_accum[k]

    cogging_calib_state_t state;  // 当前标定状态机状态
    pid_controller_t position_pd; // 标定内部使用的位置 PD 控制器
} cogging_calib_t;

void coggingCalib_init(cogging_calib_t *handle, float start_angle_rad);
uint8_t coggingCalib_update(cogging_calib_t *handle, float mech_angle_rad, uint16_t raw_count, float mech_speed_rpm, float measured_iq, float *target_iq);
void coggingCalib_getDebugData(cogging_calib_t *handle, float *data, uint16_t *len);
uint8_t coggingCalib_isFinished(cogging_calib_t *handle);
uint16_t coggingCalib_getTableSize(void);
uint16_t coggingCalib_getRawCountByIndex(cogging_calib_t *handle, uint16_t index);
float coggingCalib_getIqCompByIndex(cogging_calib_t *handle, uint16_t index);

#endif /* COGGING_CALIB_ENABLE */

#endif /* __COGGING_CALIBRATION_H__ */