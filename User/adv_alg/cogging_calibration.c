#include "cogging_calibration.h"

#if (COGGING_CALIB_ENABLE != 0U)

/**
 * @file cogging_calibration.c
 * @brief 齿槽转矩补偿表离线标定状态机
 *
 * 标定思路：
 * 1. 使用内部位置 PD 将转子依次拉到一圈内的固定机械角度；
 * 2. 每个角度点等待速度降低并稳定一段时间；
 * 3. 读取当前位置下维持转子静止所需的 measured_iq；
 * 4. 按编码器 raw count 映射到固定补偿表 bin，并多圈累加平均；
 * 5. 对缺失 bin 做环形线性插值；
 * 6. 去掉整张表的直流分量，仅保留随机械角变化的周期性齿槽补偿量。
 *
 * 最终得到的 iq_comp_table 是一张“编码器单圈位置 -> q 轴补偿电流”的前馈表。
 */

/**
 * @brief 将 0~4095 的编码器单圈原始计数映射到补偿表索引
 * @param raw_count 编码器单圈原始计数，通常来自 12bit AS5600/磁编码器
 * @return 补偿表索引，范围为 0 ~ COGGING_CALIB_TABLE_SIZE-1
 *
 * 当前表大小为 512，编码器单圈为 4096 count，因此每个表项覆盖 8 个 raw count。
 * 使用 raw count 建表可以让后续运行时补偿直接按编码器位置查表，避免受标定起点浮点角度误差影响。
 */
static uint16_t coggingCalib_rawCountToIndex(uint16_t raw_count)
{
    /* NOTE: 必须先乘后除。若先除法会因整数截断导致精度全部丢失（raw_count/4096 恒为 0）。
     * 乘法最大值为 4095 * TABLE_SIZE，会超过 16 位范围，因此中间运算必须用 32 位避免溢出。
     */
    uint32_t index = ((uint32_t)raw_count * (uint32_t)COGGING_CALIB_TABLE_SIZE) / 4096;

    return (uint16_t)index;
}

/**
 * @brief 将补偿表索引反向换算为该表项对应的编码器 raw count 起点
 * @param index 补偿表索引
 * @return 该索引对应的编码器 raw count 网格点
 */
static uint16_t coggingCalib_indexToRawCount(uint16_t index)
{
    return (uint16_t)(((uint32_t)index * 4096UL) / (uint32_t)COGGING_CALIB_TABLE_SIZE);
}

/**
 * @brief 从指定索引开始向前查找下一个已有有效采样的 bin
 * @param start_index 起始索引，该索引本身不会被优先检查
 * @return 下一个有效 bin；如果整表都没有有效点，则返回 start_index
 * @note 这个函数用于定位空bin两侧的bin，用来插值补全整张表，确保运行时每个位置都有补偿值可用。
 */
static uint16_t coggingCalib_nextValidIndex(cogging_calib_t *handle, uint16_t start_index)
{
    uint16_t offset;

    for (offset = 1U; offset <= COGGING_CALIB_TABLE_SIZE; ++offset)
    {
        // 计算环形索引
        uint16_t idx = (start_index + offset) % COGGING_CALIB_TABLE_SIZE;

        // 大于0表示该bin内已有采样值
        if (handle->raw_count_accum[idx] > 0UL)
        {
            return idx;
        }
    }

    return start_index;
}

/**
 * @brief 从指定索引开始向后查找上一个已有有效采样的 bin
 * @param start_index 起始索引，该索引本身不会被优先检查
 * @return 上一个有效 bin；如果整表都没有有效点，则返回 start_index
 * @note 这个函数用于定位空bin两侧的bin，用来插值补全整张表，确保运行时每个位置都有补偿值可用。
 */
static uint16_t coggingCalib_prevValidIndex(cogging_calib_t *handle, uint16_t start_index)
{
    uint16_t offset;

    for (offset = 1U; offset <= COGGING_CALIB_TABLE_SIZE; ++offset)
    {
        // 计算环形索引
        uint16_t idx = (uint16_t)((start_index + COGGING_CALIB_TABLE_SIZE - offset) % COGGING_CALIB_TABLE_SIZE);

        // 大于0表示该bin内已有采样值
        if (handle->raw_count_accum[idx] > 0UL)
        {
            return idx;
        }
    }

    return start_index;
}

/**
 * @brief 对没有直接采到数据的表项做环形线性插值
 * @param handle 齿槽标定句柄
 * @param index 需要补齐的表索引
 * @return 插值得到的 q 轴补偿电流
 *
 * 标定时目标角是均匀给定的，但实际转子停稳位置会受齿槽转矩、PD 参数、编码器量化影响，
 * 因此某些 raw count bin 可能没有采样值。这里使用前后最近有效 bin 的平均 iq 做线性插值，
 * 并按机械一圈周期处理 0 与末尾索引的连接关系。
 */
static float coggingCalib_interpMissingIq(cogging_calib_t *handle, uint16_t index)
{
    // 查找相邻的非空bin，用于插值计算
    uint16_t prev_idx = coggingCalib_prevValidIndex(handle, index);
    uint16_t next_idx = coggingCalib_nextValidIndex(handle, index);

    // 计算两个有效 bin 之间沿正方向的环形步数
    uint16_t dist_prev = (uint16_t)((index + COGGING_CALIB_TABLE_SIZE - prev_idx) % COGGING_CALIB_TABLE_SIZE);
    uint16_t dist_next = (uint16_t)((next_idx + COGGING_CALIB_TABLE_SIZE - prev_idx) % COGGING_CALIB_TABLE_SIZE);

    // 将环形距离转换为 0~1 的线性插值比例
    float ratio = (float)dist_prev / (float)dist_next;

    // 计算前后相邻非空bin多次采样的平均q轴电流
    float prev_iq = handle->iq_comp_accum[prev_idx] / (float)handle->raw_count_accum[prev_idx];
    float next_iq = handle->iq_comp_accum[next_idx] / (float)handle->raw_count_accum[next_idx];

    // 按比例在前后两个有效 iq 之间线性插值
    return prev_iq + (next_iq - prev_iq) * ratio;
}

/**
 * @brief 记录一次已经稳定的齿槽标定采样
 * @param handle      齿槽标定句柄
 * @param raw_count   当前编码器单圈原始计数
 * @param measured_iq 当前实际测得的 q 轴电流(A)
 *
 * 标定表最终按编码器 raw count 查表运行，因此这里不按目标角 index 写入，
 * 而是按转子实际停稳位置 raw_count 映射到固定 bin。多圈重复采样时，
 * 同一个 bin 可能被命中多次，所以分别累计命中次数和 iq 总和，最终建表时再求平均。
 */
static void coggingCalib_recordSample(cogging_calib_t *handle, uint16_t raw_count, float measured_iq)
{
    uint16_t sample_idx = coggingCalib_rawCountToIndex(raw_count);

    handle->raw_count_accum[sample_idx] += 1U;
    handle->iq_comp_accum[sample_idx] += measured_iq;
}

/**
 * @brief 根据采样累加结果生成最终齿槽补偿表
 * @param handle 齿槽标定句柄
 *
 * 对每个 raw bin：
 * - 如果有直接采样，使用该 bin 内所有 measured_iq 的平均值；
 * - 如果没有直接采样，使用前后最近有效 bin 做环形线性插值；
 * - 最后减去整表平均值，去掉恒定 q 轴电流偏置，只保留随机械角变化的周期性补偿量。
 */
static void coggingCalib_buildFinalTable(cogging_calib_t *handle)
{
    uint16_t table_idx;
    float iq_mean = 0.0f;

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

    iq_mean /= (float)COGGING_CALIB_TABLE_SIZE;
    for (table_idx = 0U; table_idx < COGGING_CALIB_TABLE_SIZE; table_idx++)
    {
        handle->iq_comp_table[table_idx] -= iq_mean;
    }
}

/**
 * @brief 初始化齿槽转矩标定器
 * @param handle            齿槽标定句柄
 * @param start_angle_rad   标定起始机械角度(rad)
 * @note 初始化后状态机会从起始角度开始，按单圈均匀采样补偿表。
 *       start_angle_rad 会被归一化到 0~2π。
 *       标定本身不依赖外部位置环，而是使用 handle 内部的 PD 直接生成 q 轴目标电流。
 */
void coggingCalib_init(cogging_calib_t *handle, float start_angle_rad)
{
    // 清空句柄数据
    memset(handle, 0, sizeof(*handle));

    // 初始化基本参数
    handle->enabled = 1U;
    handle->finished = 0U;
    handle->repeat_count = 0U;
    handle->index = 0U;
    handle->tick_count = 0U;
    handle->start_angle_rad = wrap_0_2pi(start_angle_rad);
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
 * @param raw_count         当前编码器单圈原始计数
 * @param mech_speed_rpm    当前机械转速(rpm)
 * @param measured_iq       当前实际测得的 q 轴电流(A)
 * @param target_iq         输出的标定目标 q 轴电流(A)
 * @return 1 表示标定完成；0 表示标定仍在进行
 * @note 标定流程：
 *       1) 根据 target_angle_rad 与 mech_angle_rad 的误差生成位置 PD 输出；
 *       2) 将 PD 输出作为标定期间的 target_iq，把转子拉向当前目标角；
 *       3) 转速低于阈值后进入 SETTLING，并保持 COGGING_CALIB_SETTLE_TICKS；
 *       4) 将当前 measured_iq 按 raw_count 写入固定补偿表 bin；
 *       5) 完成一圈后重复采样，达到重复次数后生成最终零均值补偿表。
 */
uint8_t coggingCalib_update(cogging_calib_t *handle, float mech_angle_rad, uint16_t raw_count, float mech_speed_rpm, float measured_iq, float *target_iq)
{
    // 标定完成后不再产生保持电流，调用方可据此退出标定模式
    if (handle->finished != 0U)
    {
        *target_iq = 0.0f;
        return 1U;
    }

    // 计算当前目标角与实际机械角之间的最短角度误差，作为位置 PD 的 P 项输入
    float position_error = wrap_neg_pi_to_pi(handle->target_angle_rad - mech_angle_rad);

    // 将机械转速从 rpm 转为 rad/s，作为 D 项的速度阻尼输入
    float speed_rad_s = mech_speed_rpm * (MATH_TWO_PI / 60.0f);

    // P项：位置误差越大，输出越大的 q 轴电流把转子拉向目标角
    handle->position_pd.error = position_error;
    handle->position_pd.p_term = handle->position_pd.kp * position_error;

    // D项：使用负机械速度作阻尼，抑制到点过程中的过冲和振荡
    handle->position_pd.derivative = -speed_rad_s;
    handle->position_pd.d_term = handle->position_pd.kd * handle->position_pd.derivative;

    // I项：积分会引入历史偏置并污染齿槽电流测量，标定阶段固定不用
    handle->position_pd.i_term = 0.0f;

    // 对位置 PD 输出限幅，避免异常角度误差或速度反馈导致过大的 q 轴电流指令
    float out_unclamped = handle->position_pd.p_term + handle->position_pd.d_term;
    handle->position_pd.out = utils_clampf(out_unclamped, handle->position_pd.out_min, handle->position_pd.out_max);

    // 将位置 PD 输出作为本周期标定电流指令，调用方会把它交给外部电流环执行
    handle->target_iq = handle->position_pd.out;
    *target_iq = handle->target_iq;

    // 速度超限时说明转子仍处于动态过程，不能把当前 measured_iq 当作静态保持电流
    // 清零 tick_count，要求当前目标点重新经历完整的连续低速稳定时间
    if (fabsf(mech_speed_rpm) > COGGING_CALIB_MAX_MECH_SPEED_RPM)
    {
        handle->tick_count = 0U;
        return 0U;
    }

    // 当前目标点尚未达到连续稳定时间，继续保持本目标角
    handle->tick_count++;

    // 低速稳定时间达标后，认为 measured_iq 可近似作为该 raw 位置的静态保持电流样本
    if (handle->tick_count >= COGGING_CALIB_SETTLE_TICKS)
    {
        coggingCalib_recordSample(handle, raw_count, measured_iq);
        handle->index++;
        handle->tick_count = 0U;

        // 当前圈还没采完：推进到下一个均匀机械角目标，下一周期重新等待稳定
        if (handle->index < COGGING_CALIB_TABLE_SIZE)
        {
            handle->target_angle_rad = wrap_0_2pi(handle->start_angle_rad + (MATH_TWO_PI / (float)COGGING_CALIB_TABLE_SIZE) * (float)handle->index);
            return 0U;
        }

        // 当前机械一圈已经采完，开始统计重复采样圈数
        handle->repeat_count++;
        if (handle->repeat_count < COGGING_CALIB_REPEAT_COUNT)
        {
            handle->index = 0U;
            handle->target_angle_rad = handle->start_angle_rad;
            return 0U;
        }

        // 所有重复圈数完成后生成最终表，并停止输出标定保持电流
        coggingCalib_buildFinalTable(handle);
        handle->state = COGGING_CALIB_STATE_DONE;
        handle->finished = 1U;
        handle->target_iq = 0.0f;
        *target_iq = 0.0f;
        return 1U;
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

    // 调试输出需要兼容标定中和标定完成两种状态：
    // - 标定中：raw_count_table/iq_comp_table 尚未最终生成，只能从累加 bin 中读取临时均值；
    // - 标定完成：直接读取最终补偿表。
    if ((handle->index > 0U) && (handle->index <= COGGING_CALIB_TABLE_SIZE))
    {
        if (handle->finished != 0U)
        {
            data[4] = (float)handle->raw_count_table[handle->index - 1U];
            data[5] = handle->iq_comp_table[handle->index - 1U];
        }
        else
        {
            // 标定进行中时，index-1 表示最近完成的目标点；这里换算到固定 raw bin 后读取临时平均值
            uint16_t debug_idx = coggingCalib_rawCountToIndex((uint16_t)(((uint32_t)(handle->index - 1U) * 4096UL) / (uint32_t)COGGING_CALIB_TABLE_SIZE));
            data[4] = (float)coggingCalib_indexToRawCount(debug_idx);
            data[5] = (handle->raw_count_accum[debug_idx] > 0UL) ? (handle->iq_comp_accum[debug_idx] / (float)handle->raw_count_accum[debug_idx]) : 0.0f;
        }
    }
    else
    {
        data[4] = 0.0f;
        data[5] = 0.0f;
    }

    *len = 6U;
}

/**
 * @brief 查询齿槽标定是否完成
 * @param handle 齿槽标定句柄
 * @return 1 表示完成；0 表示未完成或句柄无效
 */
uint8_t coggingCalib_isFinished(cogging_calib_t *handle)
{
    if (handle == NULL)
    {
        return 0U;
    }

    return handle->finished;
}

/**
 * @brief 获取齿槽补偿表长度
 * @return 补偿表点数
 */
uint16_t coggingCalib_getTableSize(void)
{
    return COGGING_CALIB_TABLE_SIZE;
}

/**
 * @brief 获取指定补偿表索引对应的编码器 raw count 网格点
 * @param handle 齿槽标定句柄
 * @param index 补偿表索引
 * @return 对应 raw count；索引越界或句柄无效时返回 0
 */
uint16_t coggingCalib_getRawCountByIndex(cogging_calib_t *handle, uint16_t index)
{
    if ((handle == NULL) || (index >= COGGING_CALIB_TABLE_SIZE))
    {
        return 0U;
    }

    return handle->raw_count_table[index];
}

/**
 * @brief 获取指定补偿表索引对应的 q 轴补偿电流
 * @param handle 齿槽标定句柄
 * @param index 补偿表索引
 * @return q 轴补偿电流(A)；索引越界或句柄无效时返回 0
 */
float coggingCalib_getIqCompByIndex(cogging_calib_t *handle, uint16_t index)
{
    if ((handle == NULL) || (index >= COGGING_CALIB_TABLE_SIZE))
    {
        return 0.0f;
    }

    return handle->iq_comp_table[index];
}

#endif /* COGGING_CALIB_ENABLE */