#ifndef __COGGING_CALIBRATION_H__
#define __COGGING_CALIBRATION_H__

#include "stm32g4xx_hal.h"
#include "../app/user_config.h"
#include "../alg/pid.h"
#include <string.h>
#include "../utils/angle_utils.h"
#include "../utils/math_utils.h"

#define COGGING_CALIB_TABLE_SIZE            512U   // 单圈补偿表采样点数
#define COGGING_CALIB_SETTLE_TICKS          200U   // 每个采样点到位后的稳定等待拍数
#define COGGING_CALIB_SAMPLE_TICKS          200U   // 稳定后进入采样状态时，额外保持的采样拍数
#define COGGING_CALIB_REPEAT_COUNT          5U    // 整张单圈补偿表重复采样次数，最终按圈求平均
#define COGGING_CALIB_MAX_MECH_SPEED_RPM    5.0f   // 允许记录数据时的最大机械转速，超过认为尚未稳定
#define COGGING_CALIB_MAX_ANGLE_ERROR_RAD   0.010f // 允许记录数据时的最大机械角误差(rad)，约0.57度
#define COGGING_CALIB_POSITION_KP           0.8f   // 标定用位置 PD 的比例系数
#define COGGING_CALIB_POSITION_KD           0.02f  // 标定用位置 PD 的微分系数
#define COGGING_CALIB_POSITION_OUT_MIN      (-0.8f) // 标定用 q 轴目标电流下限(A)
#define COGGING_CALIB_POSITION_OUT_MAX      (0.8f)  // 标定用 q 轴目标电流上限(A)

typedef enum
{
    COGGING_CALIB_STATE_IDLE = 0, // 空闲，尚未启动标定
    COGGING_CALIB_STATE_SETTLING, // 正在等待当前目标角度稳定
    COGGING_CALIB_STATE_SAMPLING, // 当前点已稳定，等待记录补偿值
    COGGING_CALIB_STATE_DONE      // 整张补偿表已经标定完成
} cogging_calib_state_t;

typedef struct
{
    uint8_t enabled;                                  // 标定器使能标志，1=运行，0=关闭
    uint8_t finished;                                 // 标定完成标志，1=整张表已采完
    uint8_t repeat_count;                             // 当前正在进行的重复采样圈次计数
    uint16_t index;                                   // 当前采样点索引，同时也是下一次写表位置
    uint16_t tick_count;                              // 当前状态下已累计的控制周期计数
    uint16_t start_raw_count;                         // 标定起始编码器原始计数(0~4095)
    float start_angle_rad;                            // 标定起始机械角(rad)
    float target_angle_rad;                           // 当前正在逼近/保持的目标机械角(rad)
    float target_iq;                                  // 当前输出给电流环的目标 q 轴电流(A)
    uint32_t sample_raw_count_accum;                  // 当前采样窗口内的编码器原始计数累加值
    float sample_iq_accum;                            // 当前采样窗口内的 q 轴电流累加值(A)
    uint16_t sample_count;                            // 当前采样窗口内的有效采样次数
    uint16_t raw_count_table[COGGING_CALIB_TABLE_SIZE]; // 补偿表的编码器原始计数点(0~4095)
    float iq_comp_table[COGGING_CALIB_TABLE_SIZE];    // 对应机械角下测得的补偿 iq(A)
    uint32_t raw_count_accum[COGGING_CALIB_TABLE_SIZE]; // 固定 raw 索引 bin 的有效采样次数
    float iq_comp_accum[COGGING_CALIB_TABLE_SIZE];    // 固定 raw 索引 bin 的补偿 iq 累加值(A)
    cogging_calib_state_t state;                      // 当前标定状态机状态
    pid_controller_t position_pd;                     // 标定内部使用的位置 PD 控制器
} cogging_calib_t;

void coggingCalib_init(cogging_calib_t *handle, float start_angle_rad);
uint8_t coggingCalib_update(cogging_calib_t *handle, float mech_angle_rad, uint16_t raw_count, float mech_speed_rpm, float measured_iq, float *target_iq);
void coggingCalib_getDebugData(cogging_calib_t *handle, float *data, uint16_t *len);
uint8_t coggingCalib_isFinished(cogging_calib_t *handle);
uint16_t coggingCalib_getTableSize(void);
uint16_t coggingCalib_getRawCountByIndex(cogging_calib_t *handle, uint16_t index);
float coggingCalib_getIqCompByIndex(cogging_calib_t *handle, uint16_t index);

#endif /* __COGGING_CALIBRATION_H__ */