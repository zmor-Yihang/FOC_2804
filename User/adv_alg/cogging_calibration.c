#include "cogging_calibration.h"

static uint16_t coggingCalib_rawCountToIndex(uint16_t raw_count)
{
    uint32_t index = ((uint32_t)raw_count * (uint32_t)COGGING_CALIB_TABLE_SIZE) / 4096UL;

    if (index >= (uint32_t)COGGING_CALIB_TABLE_SIZE)
    {
        index = (uint32_t)COGGING_CALIB_TABLE_SIZE - 1UL;
    }

    return (uint16_t)index;
}

static uint16_t coggingCalib_indexToRawCount(uint16_t index)
{
    return (uint16_t)(((uint32_t)index * 4096UL) / (uint32_t)COGGING_CALIB_TABLE_SIZE);
}

static uint16_t coggingCalib_nextValidIndex(cogging_calib_t *handle, uint16_t start_index)
{
    uint16_t offset;

    for (offset = 1U; offset <= COGGING_CALIB_TABLE_SIZE; ++offset)
    {
        uint16_t idx = (uint16_t)((start_index + offset) % COGGING_CALIB_TABLE_SIZE);
        if (handle->raw_count_accum[idx] > 0UL)
        {
            return idx;
        }
    }

    return start_index;
}

static uint16_t coggingCalib_prevValidIndex(cogging_calib_t *handle, uint16_t start_index)
{
    uint16_t offset;

    for (offset = 1U; offset <= COGGING_CALIB_TABLE_SIZE; ++offset)
    {
        uint16_t idx = (uint16_t)((start_index + COGGING_CALIB_TABLE_SIZE - offset) % COGGING_CALIB_TABLE_SIZE);
        if (handle->raw_count_accum[idx] > 0UL)
        {
            return idx;
        }
    }

    return start_index;
}

static float coggingCalib_interpMissingIq(cogging_calib_t *handle, uint16_t index)
{
    uint16_t prev_idx = coggingCalib_prevValidIndex(handle, index);
    uint16_t next_idx = coggingCalib_nextValidIndex(handle, index);
    uint16_t dist_prev;
    uint16_t dist_next;
    float ratio;
    float prev_iq;
    float next_iq;

    if ((handle->raw_count_accum[prev_idx] == 0UL) || (handle->raw_count_accum[next_idx] == 0UL))
    {
        return 0.0f;
    }

    if (prev_idx == next_idx)
    {
        return handle->iq_comp_accum[prev_idx] / (float)handle->raw_count_accum[prev_idx];
    }

    dist_prev = (uint16_t)((index + COGGING_CALIB_TABLE_SIZE - prev_idx) % COGGING_CALIB_TABLE_SIZE);
    dist_next = (uint16_t)((next_idx + COGGING_CALIB_TABLE_SIZE - prev_idx) % COGGING_CALIB_TABLE_SIZE);
    ratio = (dist_next > 0U) ? ((float)dist_prev / (float)dist_next) : 0.0f;
    prev_iq = handle->iq_comp_accum[prev_idx] / (float)handle->raw_count_accum[prev_idx];
    next_iq = handle->iq_comp_accum[next_idx] / (float)handle->raw_count_accum[next_idx];

    return prev_iq + (next_iq - prev_iq) * ratio;
}

/**
 * @brief 初始化齿槽转矩标定器
 * @param handle            齿槽标定句柄
 * @param start_angle_rad   标定起始机械角度(rad)
 * @note 初始化后状态机会从起始角度开始，按单圈均匀采样补偿表
 */
void coggingCalib_init(cogging_calib_t *handle, float start_angle_rad)
{
    const float encoder_cpr = 4096.0f;

    // 清空句柄数据
    memset(handle, 0, sizeof(*handle));

    // 初始化基本参数
    handle->enabled = 1U;
    handle->finished = 0U;
    handle->repeat_count = 0U;
    handle->index = 0U;
    handle->tick_count = 0U;
    handle->start_angle_rad = wrap_0_2pi(start_angle_rad);
    handle->start_raw_count = ((uint16_t)((handle->start_angle_rad * encoder_cpr) / MATH_TWO_PI)) & 0x0FFFU;
    handle->target_angle_rad = handle->start_angle_rad;
    handle->target_iq = 0.0f;
    handle->state = COGGING_CALIB_STATE_SETTLING;

    // 标定过程内部使用的位置 PD，只负责把转子拉到每个采样机械角
    pid_init(&handle->position_pd,
             PID_MODE_PD,
             COGGING_CALIB_POSITION_KP,
             0.0f,
             COGGING_CALIB_POSITION_KD,
             COGGING_CALIB_POSITION_OUT_MIN,
             COGGING_CALIB_POSITION_OUT_MAX,
             PID_LIMIT_DISABLE);
}

/**
 * @brief 执行一次齿槽标定状态机更新
 * @param handle            齿槽标定句柄
 * @param mech_angle_rad    当前机械角度(rad)
 * @param mech_speed_rpm    当前机械转速(rpm)
 * @param measured_iq       当前实际测得的 q 轴电流(A)
 * @param target_iq         输出的标定目标 q 轴电流(A)
 * @return 1 表示标定完成；0 表示标定仍在进行
 * @note 标定流程：
 *       1) 先把转子拉到目标角
 *       2) 进入稳定等待
 *       3) 等速度足够小后记录该点的机械角与 iq
 *       4) 切换到下一个采样点
 */
uint8_t coggingCalib_update(cogging_calib_t *handle, float mech_angle_rad, uint16_t raw_count, float mech_speed_rpm, float measured_iq, float *target_iq)
{
    float position_error;
    float speed_rad_s;
    float out_unclamped;

    if ((handle == NULL) || (target_iq == NULL) || (handle->enabled == 0U))
    {
        return 0U;
    }

    if (handle->finished != 0U)
    {
        *target_iq = 0.0f;
        return 1U;
    }

    // 位置 PD 内联展开：根据当前位置误差生成标定用目标 iq，把转子拉向当前采样角
    position_error = wrap_pm_pi(handle->target_angle_rad - mech_angle_rad);
    speed_rad_s = mech_speed_rpm * (MATH_TWO_PI / 60.0f);

    // P项：根据位置误差产生校正转矩电流
    handle->position_pd.error = position_error;
    handle->position_pd.p_term = handle->position_pd.kp * position_error;

    // D项：直接使用机械速度作阻尼，速度越大，抑制越强
    handle->position_pd.derivative = -speed_rad_s;
    handle->position_pd.d_term = handle->position_pd.kd * handle->position_pd.derivative;
    handle->position_pd.i_term = 0.0f;

    // 位置 PD 输出限幅到允许的标定电流范围
    out_unclamped = handle->position_pd.p_term + handle->position_pd.d_term;
    handle->position_pd.out = utils_clampf(out_unclamped, handle->position_pd.out_min, handle->position_pd.out_max);

    handle->target_iq = handle->position_pd.out;
    *target_iq = handle->target_iq;

    // 转速过大说明还没稳定，此时不累计等待计数，避免误采样
    if (fabsf(mech_speed_rpm) > COGGING_CALIB_MAX_MECH_SPEED_RPM)
    {
        handle->tick_count = 0U;
        return 0U;
    }

    handle->tick_count++;

    if (handle->state == COGGING_CALIB_STATE_SETTLING)
    {
        // 在当前目标角停留一段时间，让位置和电流充分稳定
        if (handle->tick_count >= COGGING_CALIB_SETTLE_TICKS)
        {
            handle->tick_count = 0U;
            handle->state = COGGING_CALIB_STATE_SAMPLING;
        }

        return 0U;
    }

    if (handle->state == COGGING_CALIB_STATE_SAMPLING)
    {
        if (handle->tick_count >= COGGING_CALIB_SAMPLE_TICKS)
        {
            // 记录该采样点：按编码器单圈 raw 直接映射到固定补偿表索引
            {
                uint16_t sample_idx = coggingCalib_rawCountToIndex(raw_count);
                handle->raw_count_accum[sample_idx] += 1UL;
                handle->iq_comp_accum[sample_idx] += measured_iq;
            }
            handle->index++;
            handle->tick_count = 0U;

            if (handle->index >= COGGING_CALIB_TABLE_SIZE)
            {
                handle->repeat_count++;

                if (handle->repeat_count >= COGGING_CALIB_REPEAT_COUNT)
                {
                    uint16_t table_idx;
                    float iq_mean = 0.0f;

                    // 固定索引补偿表：每个 index 对应固定 raw 网格，缺失 bin 用相邻有效 bin 环形插值补齐
                    for (table_idx = 0U; table_idx < COGGING_CALIB_TABLE_SIZE; table_idx++)
                    {
                        handle->raw_count_table[table_idx] = coggingCalib_indexToRawCount(table_idx);

                        if (handle->raw_count_accum[table_idx] > 0UL)
                        {
                            handle->iq_comp_table[table_idx] = handle->iq_comp_accum[table_idx] / (float)handle->raw_count_accum[table_idx];
                        }
                        else
                        {
                            handle->iq_comp_table[table_idx] = coggingCalib_interpMissingIq(handle, table_idx);
                        }

                        iq_mean += handle->iq_comp_table[table_idx];
                    }

                    // 去掉整张表的直流分量，仅保留齿槽转矩随角度变化的补偿量
                    iq_mean /= (float)COGGING_CALIB_TABLE_SIZE;
                    for (table_idx = 0U; table_idx < COGGING_CALIB_TABLE_SIZE; table_idx++)
                    {
                        handle->iq_comp_table[table_idx] -= iq_mean;
                    }

                    handle->state = COGGING_CALIB_STATE_DONE;
                    handle->finished = 1U;
                    handle->target_iq = 0.0f;
                    *target_iq = 0.0f;
                    return 1U;
                }

                // 一圈采样完成但未达到目标圈数，回到起点再采下一圈
                handle->index = 0U;
                handle->target_angle_rad = handle->start_angle_rad;
                handle->state = COGGING_CALIB_STATE_SETTLING;
                return 0U;
            }

            // 切换到下一个目标机械角，继续下一点标定
            handle->target_angle_rad = wrap_0_2pi(handle->start_angle_rad + (MATH_TWO_PI / (float)COGGING_CALIB_TABLE_SIZE) * (float)handle->index);
            handle->state = COGGING_CALIB_STATE_SETTLING;
        }
    }

    return 0U;
}

/**
 * @brief 获取用于上位机观察的标定调试数据
 * @param handle 齿槽标定句柄
 * @param data   输出数据数组
 * @param len    输出数据长度
 * @note 当前各字段定义：
 *       data[0] = 当前状态 state
 *       data[1] = 当前采样索引 index
 *       data[2] = 当前目标机械角 target_angle_rad
 *       data[3] = 当前重复采样圈次 repeat_count
 *       data[4] = 最近一次写入补偿表的编码器原始计数 raw_count_table[index-1]
 *       data[5] = 最近一次写入补偿表的补偿 iq iq_comp_table[index-1]
 */
void coggingCalib_getDebugData(cogging_calib_t *handle, float *data, uint16_t *len)
{
    if ((handle == NULL) || (data == NULL) || (len == NULL))
    {
        return;
    }

    data[0] = (float)handle->state;
    data[1] = (float)handle->index;
    data[2] = handle->target_angle_rad;
    data[3] = (float)handle->repeat_count;

    if ((handle->index > 0U) && (handle->index <= COGGING_CALIB_TABLE_SIZE))
    {
        if (handle->finished != 0U)
        {
            data[4] = (float)handle->raw_count_table[handle->index - 1U];
            data[5] = handle->iq_comp_table[handle->index - 1U];
        }
        else
        {
            uint16_t debug_idx = coggingCalib_rawCountToIndex((uint16_t)(((uint32_t)(handle->index - 1U) * 4096UL) / (uint32_t)COGGING_CALIB_TABLE_SIZE));
            data[4] = (float)coggingCalib_indexToRawCount(debug_idx);
            data[5] = (handle->raw_count_accum[debug_idx] > 0UL) ?
                          (handle->iq_comp_accum[debug_idx] / (float)handle->raw_count_accum[debug_idx]) :
                          0.0f;
        }
    }
    else
    {
        data[4] = 0.0f;
        data[5] = 0.0f;
    }

    *len = 6U;
}

uint8_t coggingCalib_isFinished(cogging_calib_t *handle)
{
    if (handle == NULL)
    {
        return 0U;
    }

    return handle->finished;
}

uint16_t coggingCalib_getTableSize(void)
{
    return COGGING_CALIB_TABLE_SIZE;
}

uint16_t coggingCalib_getRawCountByIndex(cogging_calib_t *handle, uint16_t index)
{
    if ((handle == NULL) || (index >= COGGING_CALIB_TABLE_SIZE))
    {
        return 0U;
    }

    return handle->raw_count_table[index];
}

float coggingCalib_getIqCompByIndex(cogging_calib_t *handle, uint16_t index)
{
    if ((handle == NULL) || (index >= COGGING_CALIB_TABLE_SIZE))
    {
        return 0.0f;
    }

    return handle->iq_comp_table[index];
}