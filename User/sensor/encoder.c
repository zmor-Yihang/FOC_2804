#include "encoder.h"

// 当前采样状态
static uint16_t encoder_raw_count = 0U; // 当前编码器原始计数(0~4095)
static float mech_angle_single_turn = 0.0f; // 当前机械单圈角度，用于单圈位置反馈

// 机械多圈位置状态
static float mech_position_multi_turn = 0.0f;   // 当前机械多圈位置，用于多圈位置反馈
static float mech_angle_prev = 0.0f; // 上一次机械单圈角度(rad)

// PLL估计状态
static float elec_angle_pll = 0.0f;   /* PLL相位估计值(单位:电角度rad) */
static float elec_speed_pll_rad_s = 0.0f; /* PLL速度估计值(单位:电角速度rad/s) */

/**
 * @brief 将计数值换算为机械角度（rad） 0-2π，也是单圈位置
 */
static inline float encoder_convert_countToMechanicalAngle(float count)
{
#if (ENCODER_COUNT_SWAP == 0) // 不颠倒
    uint16_t mech_count = count;
#else
    uint16_t mech_count = ENCODER_CPR - count;
#endif /* ENCODER_COUNT_SWAP */

    return mech_count * (MATH_TWO_PI / (float)ENCODER_CPR);
}

/**
 * @brief 更新机械多圈位置
 * @note 单圈角度在0/2π处会回绕，将相邻两次误差映射到[-π, π]，得到采样周期内的最短角度增量再累计
 *       这样处理回绕的前提，电机在一次采样周期内不会超过180度，换算为速度就是∣ω∣< π /Ts
 */
static void encoder_update_mechanicalPosition(float mech_angle)
{
    float delta_angle = wrap_neg_pi_to_pi(mech_angle - mech_angle_prev);
    mech_position_multi_turn += delta_angle;
    mech_angle_prev = mech_angle;
}

/**
 * @brief 获取当前电角度
 * @note 编码器未通信完成时，采用预测的电角度
 */
static float encoder_get_currentElectricalAngle(void)
{
    uint16_t raw_count = 0U;
    float elec_angle = 0.0f;

    if (as5600_read_rawCountAsync(&raw_count) != 0U) // 编码器通信完成
    {
        encoder_raw_count = raw_count;
        mech_angle_single_turn = encoder_convert_countToMechanicalAngle(raw_count); // 计算当前机械角度
        elec_angle = mech_angle_single_turn * MOTOR_POLE_PAIRS;                     // 计算当前电角度
    }
    else // 编码器未通信完成，使用预测的电角度
    {
        // 用上一拍速度预测当前角度
        elec_angle = elec_angle_pll + elec_speed_pll_rad_s * ENCODER_SPEED_SAMPLE_TIME;
        mech_angle_single_turn = wrap_0_2pi(elec_angle / MOTOR_POLE_PAIRS);
    }

    encoder_update_mechanicalPosition(mech_angle_single_turn);
    return wrap_0_2pi(elec_angle);
}

/**
 * @brief 初始化编码器
 */
void encoder_init(void)
{
    uint16_t raw_count = 0U;
    float elec_angle = 0.0f;

    // 初始化编码器
    as5600_init();

    // 阻塞读取编码器角度，确保初始化的值是测量值
    as5600_read_rawCountBlock(&raw_count);

    // 给电角度和机械角度赋初值
    encoder_raw_count = raw_count;
    mech_angle_single_turn = encoder_convert_countToMechanicalAngle(raw_count);
    elec_angle = wrap_0_2pi(mech_angle_single_turn * MOTOR_POLE_PAIRS);
    mech_position_multi_turn = mech_angle_single_turn;
    mech_angle_prev = mech_angle_single_turn;
    elec_angle_pll = elec_angle;
}

/**
 * @brief PLL状态迭代，更新角度和速度
 */
void encoder_update(void)
{
    float elec_angle = encoder_get_currentElectricalAngle(); // 获取当前电角度

    float phase_error = wrap_neg_pi_to_pi(elec_angle - elec_angle_pll); // 计算当前相位误差
    float speed_integral_step = ENCODER_PLL_KI * phase_error * ENCODER_SPEED_SAMPLE_TIME;

    elec_speed_pll_rad_s += speed_integral_step;
    elec_speed_pll_rad_s = utils_clampf(elec_speed_pll_rad_s, -ENCODER_PLL_SPEED_LIMIT_RAD_S, ENCODER_PLL_SPEED_LIMIT_RAD_S);

    // 每个控制周期都使用比例校正项 + 当前速度估计推进PLL相位
    elec_angle_pll += (elec_speed_pll_rad_s + ENCODER_PLL_KP * phase_error) * ENCODER_SPEED_SAMPLE_TIME;

    elec_angle_pll = wrap_0_2pi(elec_angle_pll);
}

/**
 * @brief 获取PLL估计的电角度[0, 2π)：rad
 */
float encoder_get_pllAngle(void)
{
#if (ENCODER_PLL_ANGLE_COMP_ENABLE == 1)
    return angleUtils_compensate_delay(elec_angle_pll, elec_speed_pll_rad_s, ENCODER_PLL_ANGLE_COMP_DELAY_S);
#else
    return elec_angle_pll;
#endif /* ENCODER_PLL_ANGLE_COMP_ENABLE */
}

/**
 * @brief 获取PLL估计的转速: rpm
 */
float encoder_get_pllSpeed(void)
{
    return (elec_speed_pll_rad_s / (MATH_TWO_PI * MOTOR_POLE_PAIRS)) * 60.0f;
}

/**
 * @brief 获取当前编码器原始计数
 * @return AS5600原始计数，范围0~4095
 */
uint16_t encoder_get_rawCount(void)
{
    return encoder_raw_count;
}

/**
 * @brief 获取机械单圈角度[0, 2π)：rad
 */
float encoder_get_mechanicalAngle(void)
{
    return mech_angle_single_turn;
}

/**
 * @brief 获取机械多圈位置：rad
 */
float encoder_get_mechanicalPosition(void)
{
    return mech_position_multi_turn;
}

/**
 * @brief 重置机械多圈位置零点
 * @param position_rad 重置后的当前位置(rad)
 */
void encoder_reset_mechanicalPosition(float position_rad)
{
    mech_position_multi_turn = position_rad;         // 直接设置多圈位置，避免重置时出现大跳变
    mech_angle_prev = mech_angle_single_turn; // 设置mech_angle_prev 避免下一次更新时出现大跳变
}

/**
 * @brief 阻塞获取机械单圈角度，确保返回的是测量值
 * @note 用于零点对齐，齿槽转矩补偿
 * @return 机械单圈角度[0, 2π)：rad
 */
float encoder_get_mechanicalAngleBlock(void)
{
    uint16_t raw_count = 0U;

    as5600_read_rawCountBlock(&raw_count);

    encoder_raw_count = raw_count;
    mech_angle_single_turn = encoder_convert_countToMechanicalAngle(raw_count);
    return mech_angle_single_turn;
}