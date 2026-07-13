#include "inductance_measure.h"
#include <math.h>

/*
 * 电感辨识按“零电压等待 → 单轴脉冲 → 零电压等待”循环运行。
 * 每个轴依次施加正、负脉冲以抵消初始电流和偏置带来的系统误差；D 轴完成后可选测量 Q 轴。
 * 调用方必须以 cfg->ts 周期调用 indMeas_update()，并将输出的 d/q 电压参考送入控制环。
 */

/*
 * 限制后续参与除法和时间计算的配置项，避免产生无意义的辨识结果。
 * rest_ticks 可为 0，用于不插入额外的零电压等待周期。
 */
static uint8_t indMeas_config_is_valid(const ind_meas_cfg_t *cfg) {
    return (cfg != 0) && (cfg->voltage > 0.0f) && (cfg->ts > 0.0f) && (cfg->pulse_ticks > 0U) && (cfg->sample_count > 0U) && (cfg->result_scale > 0.0f) && (cfg->min_delta_current >= 0.0f);
}

/* 将一次测量的运行状态和结果复位；该函数不修改已绑定的 cfg 配置指针。 */
static void indMeas_clear(ind_meas_t *im) {
    im->state    = IND_MEAS_IDLE;
    im->fault    = IND_MEAS_FAULT_NONE;
    im->axis     = IND_MEAS_AXIS_D;
    im->polarity = 1;

    im->tick_counter = 0U;
    im->pair_count   = 0U;

    im->start_current = 0.0f;
    im->v_d_ref       = 0.0f;
    im->v_q_ref       = 0.0f;

    im->sum_ld          = 0.0f;
    im->sum_lq          = 0.0f;
    im->valid_d_samples = 0U;
    im->valid_q_samples = 0U;

    im->current_delta_sum     = 0.0f;
    im->current_delta_samples = 0U;

    im->ld           = 0.0f;
    im->lq           = 0.0f;
    im->inductance   = 0.0f;
    im->ld_lq_diff   = 0.0f;
    im->current_used = 0.0f;

    im->last_delta_current = 0.0f;
    im->last_l_sample      = 0.0f;
}

/* 空闲、完成和故障时始终撤销注入电压，避免电机继续受控激励。 */
static void indMeas_set_zero_voltage(ind_meas_t *im) {
    im->v_d_ref = 0.0f;
    im->v_q_ref = 0.0f;
}

/* 将带符号测试电压仅施加到当前被辨识的坐标轴，另一轴保持为零。 */
static void indMeas_set_pulse_voltage(ind_meas_t *im) {
    float v = (float)im->polarity * im->cfg->voltage;

    if (im->axis == IND_MEAS_AXIS_D) {
        im->v_d_ref = v;
        im->v_q_ref = 0.0f;
    } else {
        im->v_d_ref = 0.0f;
        im->v_q_ref = v;
    }
}

/* 只使用当前注入轴的电流计算 Δi，避免另一轴的耦合电流混入辨识结果。 */
static float indMeas_get_axis_current(const ind_meas_t *im, float id_fb, float iq_fb) {
    return (im->axis == IND_MEAS_AXIS_D) ? id_fb : iq_fb;
}

/*
 * 汇总有效样本并结束本轮测量。
 * 未获得任一必需轴的有效样本时，不输出部分结果，而是进入故障状态以防上层误用。
 */
static void indMeas_finish(ind_meas_t *im) {
    if ((im->valid_d_samples == 0U) || ((im->cfg->measure_q_axis != 0U) && (im->valid_q_samples == 0U))) {
        im->fault = IND_MEAS_FAULT_NO_VALID_SAMPLE;
        im->state = IND_MEAS_FAULT;
        indMeas_set_zero_voltage(im);
        return;
    }

    im->ld = im->sum_ld / (float)im->valid_d_samples;

    /* 双轴模式以 Ld/Lq 的算术平均值作为总电感，并保留凸极差值供上层使用。 */
    if (im->cfg->measure_q_axis != 0U) {
        im->lq         = im->sum_lq / (float)im->valid_q_samples;
        im->inductance = 0.5f * (im->ld + im->lq);
        im->ld_lq_diff = im->lq - im->ld;
    } else {
        im->lq         = im->ld;
        im->inductance = im->ld;
        im->ld_lq_diff = 0.0f;
    }

    if (im->current_delta_samples > 0U) {
        /* 仅统计实际参与均值的样本，避免无效脉冲降低该诊断量的可信度。 */
        im->current_used = im->current_delta_sum / (float)im->current_delta_samples;
    }

    im->state = IND_MEAS_DONE;
    indMeas_set_zero_voltage(im);
}

/*
 * 收尾单次脉冲并推进测量序列。
 * 一个样本对固定为“正脉冲 + 负脉冲”；完成指定对数后，D 轴切换至 Q 轴或直接汇总结果。
 */
static void indMeas_next_pulse(ind_meas_t *im) {
    im->tick_counter = 0U;
    indMeas_set_zero_voltage(im);

    if (im->polarity > 0) {
        im->polarity = -1;
        im->state    = IND_MEAS_REST;
        return;
    }

    im->pair_count++;
    im->polarity = 1;

    if (im->pair_count < im->cfg->sample_count) {
        im->state = IND_MEAS_REST;
        return;
    }

    if ((im->axis == IND_MEAS_AXIS_D) && (im->cfg->measure_q_axis != 0U)) {
        im->axis       = IND_MEAS_AXIS_Q;
        im->pair_count = 0U;
        im->state      = IND_MEAS_REST;
        return;
    }

    indMeas_finish(im);
}

/*
 * 从一个脉冲前后的同轴电流计算电感单样本。
 * 计算模型为 L = (Vpulse - R * Iavg) * Tpulse / ΔI，并乘以 result_scale 进行标定。
 * 小电流变化、非有限值或负电感均视为无效样本，不参与均值计算。
 */
static void indMeas_accumulate_sample(ind_meas_t *im, float end_current) {
    float delta_i     = end_current - im->start_current;
    float abs_delta_i = fabsf(delta_i);
    /* 即使样本被拒绝也保留本次 Δi，便于上层定位阈值或采样时序问题。 */
    im->last_delta_current = delta_i;

    if (abs_delta_i < im->cfg->min_delta_current) {
        return;
    }

    float pulse_time   = im->cfg->ts * (float)im->cfg->pulse_ticks;
    float axis_voltage = (float)im->polarity * im->cfg->voltage;
    float avg_current  = 0.5f * (im->start_current + end_current);
    /* 用脉冲期间的平均电流估计电阻压降，得到驱动电感变化的有效电压。 */
    float effective_voltage = axis_voltage - im->cfg->phase_resistance * avg_current;
    float l_sample          = (effective_voltage * pulse_time / delta_i) * im->cfg->result_scale;

    if (isfinite(l_sample) && (l_sample > 0.0f)) {
        im->last_l_sample = l_sample;

        if (im->axis == IND_MEAS_AXIS_D) {
            im->sum_ld += l_sample;
            im->valid_d_samples++;
        } else {
            im->sum_lq += l_sample;
            im->valid_q_samples++;
        }

        im->current_delta_sum += abs_delta_i;
        im->current_delta_samples++;
    }
}

/*
 * 绑定配置并初始化测量器。
 * 配置无效时立即进入 FAULT；调用方可通过 indMeas_get_fault() 获取原因。
 */
void indMeas_init(ind_meas_t *im, const ind_meas_cfg_t *cfg) {
    im->cfg = cfg;
    indMeas_clear(im);

    if (!indMeas_config_is_valid(cfg)) {
        im->fault = IND_MEAS_FAULT_BAD_CONFIG;
        im->state = IND_MEAS_FAULT;
    }
}

/*
 * 开始一轮新的测量并清除上次结果，保留初始化阶段绑定的配置指针。
 * 成功启动后先进入 REST，以零电压稳定电流，再记录脉冲起始电流。
 */
void indMeas_start(ind_meas_t *im) {
    const ind_meas_cfg_t *cfg = im->cfg;
    indMeas_clear(im);
    im->cfg = cfg;

    if (!indMeas_config_is_valid(cfg)) {
        im->fault = IND_MEAS_FAULT_BAD_CONFIG;
        im->state = IND_MEAS_FAULT;
        return;
    }

    im->state = IND_MEAS_REST;
}

/*
 * 每个控制周期调用一次的测量状态机。
 * 输入为当前 D/Q 轴电流反馈；函数同步更新 v_d_ref/v_q_ref，调用方应在本周期使用对应输出。
 * 状态转移：REST → PULSE → REST，完成全部样本后进入 DONE；任何过流立即进入 FAULT。
 */
void indMeas_update(ind_meas_t *im, float id_fb, float iq_fb) {
    if ((im->state == IND_MEAS_IDLE) || (im->state == IND_MEAS_DONE) || (im->state == IND_MEAS_FAULT)) {
        /* 终态不可继续施加脉冲，必须由 indMeas_start() 显式开启下一轮。 */
        indMeas_set_zero_voltage(im);
        return;
    }

    if ((im->cfg->max_abs_current > 0.0f) && ((fabsf(id_fb) > im->cfg->max_abs_current) || (fabsf(iq_fb) > im->cfg->max_abs_current))) {
        /* 无论当前注入哪个轴，两个电流反馈均参与保护，避免交叉耦合导致遗漏过流。 */
        im->fault = IND_MEAS_FAULT_OVER_CURRENT;
        im->state = IND_MEAS_FAULT;
        indMeas_set_zero_voltage(im);
        return;
    }

    float axis_current = indMeas_get_axis_current(im, id_fb, iq_fb);

    switch (im->state) {
        case IND_MEAS_REST:
            /* REST 阶段不注入电压；等待结束瞬间取样，作为随后的脉冲起始电流。 */
            indMeas_set_zero_voltage(im);
            im->tick_counter++;
            if (im->tick_counter >= im->cfg->rest_ticks) {
                im->tick_counter  = 0U;
                im->start_current = axis_current;
                im->state         = IND_MEAS_PULSE;
                indMeas_set_pulse_voltage(im);
            }
            break;

        case IND_MEAS_PULSE:
            /* 保持同极性、同轴电压 pulse_ticks 个周期，结束时用当前反馈形成一个样本。 */
            indMeas_set_pulse_voltage(im);
            im->tick_counter++;
            if (im->tick_counter >= im->cfg->pulse_ticks) {
                indMeas_accumulate_sample(im, axis_current);
                indMeas_next_pulse(im);
            }
            break;

        case IND_MEAS_IDLE:
        case IND_MEAS_DONE:
        case IND_MEAS_FAULT:
        default:
            indMeas_set_zero_voltage(im);
            break;
    }
}

ind_meas_state_t indMeas_get_state(const ind_meas_t *im) {
    return im->state;
}

ind_meas_fault_t indMeas_get_fault(const ind_meas_t *im) {
    return im->fault;
}

ind_meas_axis_t indMeas_get_axis(const ind_meas_t *im) {
    return im->axis;
}

float indMeas_get_vd_ref(const ind_meas_t *im) {
    return im->v_d_ref;
}

float indMeas_get_vq_ref(const ind_meas_t *im) {
    return im->v_q_ref;
}

float indMeas_get_ld(const ind_meas_t *im) {
    return im->ld;
}

float indMeas_get_lq(const ind_meas_t *im) {
    return im->lq;
}

float indMeas_get_inductance(const ind_meas_t *im) {
    return im->inductance;
}

float indMeas_get_ld_lq_diff(const ind_meas_t *im) {
    return im->ld_lq_diff;
}

float indMeas_get_current_used(const ind_meas_t *im) {
    return im->current_used;
}

float indMeas_get_last_delta_current(const ind_meas_t *im) {
    return im->last_delta_current;
}

float indMeas_get_last_l_sample(const ind_meas_t *im) {
    return im->last_l_sample;
}