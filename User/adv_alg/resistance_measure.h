#ifndef __RESISTANCE_MEASURE_H_
#define __RESISTANCE_MEASURE_H_

#include <stdint.h>

/**
 * @brief 电阻辨识状态机状态
 */
typedef enum
{
    RES_MEAS_IDLE = 0,
    RES_MEAS_RAMP_UP,
    RES_MEAS_SETTLE,
    RES_MEAS_SAMPLE,
    RES_MEAS_DONE
} res_meas_state_t;

/**
 * @brief 电阻辨识配置
 */
typedef struct
{
    float target_current;  // 辨识注入电流 (A)
    float ramp_rate;       // 电流爬升速率 (A/s)
    float ts;              // 控制周期 (s)
    uint32_t settle_ticks; // 稳态等待拍数
    uint32_t sample_count; // 采样累积拍数
} res_meas_cfg_t;

/**
 * @brief 电阻辨识运行状态
 */
typedef struct
{
    const res_meas_cfg_t *cfg;
    res_meas_state_t state;

    float current_ref;     // 当前电流参考值（爬升中）
    uint32_t tick_counter; // 通用计数器

    // 采样累积
    float sum_vd;
    float sum_id;
    uint32_t sample_num;

    // 结果
    float resistance;      // 辨识结果 (Ω)
} res_meas_t;

void resMeas_init(res_meas_t *rm, const res_meas_cfg_t *cfg);
void resMeas_start(res_meas_t *rm);
void resMeas_update(res_meas_t *rm, float vd_out, float id_fb);
res_meas_state_t resMeas_get_state(const res_meas_t *rm);
float resMeas_get_result(const res_meas_t *rm);
float resMeas_get_current_ref(const res_meas_t *rm);

#endif