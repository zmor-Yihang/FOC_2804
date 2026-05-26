#include "resistance_measure.h"

void resMeas_init(res_meas_t *rm, const res_meas_cfg_t *cfg)
{
    rm->cfg = cfg;
    rm->state = RES_MEAS_IDLE;
    rm->current_ref = 0.0f;
    rm->tick_counter = 0;
    rm->sum_vd = 0.0f;
    rm->sum_id = 0.0f;
    rm->sample_num = 0;
    rm->resistance = 0.0f;
}

void resMeas_start(res_meas_t *rm)
{
    rm->state = RES_MEAS_RAMP_UP;
    rm->current_ref = 0.0f;
    rm->tick_counter = 0;
    rm->sum_vd = 0.0f;
    rm->sum_id = 0.0f;
    rm->sample_num = 0;
    rm->resistance = 0.0f;
}

/**
 * @brief 每拍调用一次，驱动状态机
 * @param rm     辨识句柄
 * @param vd_out 当前拍电流环输出的 d 轴电压 (V)
 * @param id_fb  当前拍 d 轴电流反馈 (A)
 */
void resMeas_update(res_meas_t *rm, float vd_out, float id_fb)
{
    const res_meas_cfg_t *cfg = rm->cfg;

    switch (rm->state)
    {
    case RES_MEAS_RAMP_UP:
        // 以固定速率爬升电流参考
        rm->current_ref += cfg->ramp_rate * cfg->ts;
        if (rm->current_ref >= cfg->target_current)
        {
            rm->current_ref = cfg->target_current;
            rm->tick_counter = 0;
            rm->state = RES_MEAS_SETTLE;
        }
        break;

    case RES_MEAS_SETTLE:
        // 等待电流稳定
        rm->tick_counter++;
        if (rm->tick_counter >= cfg->settle_ticks)
        {
            rm->tick_counter = 0;
            rm->sum_vd = 0.0f;
            rm->sum_id = 0.0f;
            rm->sample_num = 0;
            rm->state = RES_MEAS_SAMPLE;
        }
        break;

    case RES_MEAS_SAMPLE:
        // 累积采样
        rm->sum_vd += vd_out;
        rm->sum_id += id_fb;
        rm->sample_num++;
        if (rm->sample_num >= cfg->sample_count)
        {
            float avg_vd = rm->sum_vd / (float)rm->sample_num;
            float avg_id = rm->sum_id / (float)rm->sample_num;
            if (avg_id > 0.01f)
            {
                rm->resistance = avg_vd / avg_id;
            }
            rm->current_ref = 0.0f;
            rm->state = RES_MEAS_DONE;
        }
        break;

    case RES_MEAS_DONE:
    case RES_MEAS_IDLE:
    default:
        break;
    }
}

res_meas_state_t resMeas_get_state(const res_meas_t *rm)
{
    return rm->state;
}

float resMeas_get_result(const res_meas_t *rm)
{
    return rm->resistance;
}

float resMeas_get_current_ref(const res_meas_t *rm)
{
    return rm->current_ref;
}