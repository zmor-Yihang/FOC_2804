#include "cogging_calibration_mode.h"

#if (COGGING_CALIB_ENABLE != 0U)

#include "../adv_alg/cogging_calibration.h"
#include <stdio.h>

// 齿槽标定模式专用 FOC 句柄
static foc_t foc_cogging_calib_handle;
// 齿槽补偿表标定器句柄
static cogging_calib_t cogging_calib_handle;

// 电流环 PI 控制器
static pid_controller_t pid_id;
static pid_controller_t pid_iq;

// 上位机调试输出缓存，只保留补偿值相关量
static float mech_angle_temp = 0.0f;
static float cogging_state_temp = 0.0f;
static float cogging_index_temp = 0.0f;
static float cogging_target_angle_temp = 0.0f;
static float cogging_repeat_temp = 0.0f;
static float cogging_table_raw_count_temp = 0.0f;
static float cogging_table_iq_temp = 0.0f;
static uint8_t cogging_final_table_printed = 0U;
static uint8_t cogging_final_table_cached = 0U;
static uint16_t cogging_final_table_size = 0U;
static uint16_t cogging_final_raw_count_table[COGGING_CALIB_TABLE_SIZE];
static float cogging_final_iq_table[COGGING_CALIB_TABLE_SIZE];
static char cogging_final_print_buffer[10192U];

/**
 * @brief 齿槽标定模式电流环回调
 * @note 在 ADC 注入转换完成中断链路中执行：
 *       1) 更新编码器角度与速度
 *       2) 完成电流采样和 Park 变换
 *       3) 调用标定状态机生成目标 iq
 *       4) 运行电流环并缓存调试输出
 */
static void cogging_calibration_mode_callback(void)
{
    // 更新编码器角度和速度估计
    encoder_update();

    // 获取控制所需角度、速度和机械角度
    float angle_el = encoder_get_pllAngle() - foc_cogging_calib_handle.angle_offset;
    float speed_feedback = encoder_get_pllSpeed();
    float mech_angle = encoder_get_mechanicalAngle();
    uint16_t raw_count = encoder_get_rawCount();

    // 获取三相电流采样值
    abc_t i_abc;
    currentSense_get_injectedValue(&i_abc);

    // 坐标变换到 dq 轴，得到当前 q 轴电流反馈
    alphabeta_t i_alphabeta = clark_transform(i_abc);
    dq_t i_dq = park_transform(i_alphabeta, angle_el);

    // 执行标定状态机：根据目标机械角输出维持该位置所需的目标 iq
    if (coggingCalib_update(&cogging_calib_handle, mech_angle, raw_count, speed_feedback, i_dq.q, &foc_cogging_calib_handle.target_iq) != 0U)
    {
        // 标定结束后清零输出，避免继续给转矩
        foc_cogging_calib_handle.target_id = 0.0f;
        foc_cogging_calib_handle.target_iq = 0.0f;
    }
    else
    {
        // 标定过程中只使用 q 轴转矩电流，d 轴恒为 0
        foc_cogging_calib_handle.target_id = 0.0f;
    }

    // 电流环根据目标 id/iq 输出电压矢量
    loopControl_run_currentLoop(&foc_cogging_calib_handle, i_dq, angle_el, speed_feedback);

    // 缓存当前机械角，供上位机显示
    mech_angle_temp = mech_angle;

    // 读取标定器内部调试数据，提取补偿表相关字段
    float calib_data[6] = {0};
    uint16_t calib_len = 0U;
    coggingCalib_getDebugData(&cogging_calib_handle, calib_data, &calib_len);
    cogging_state_temp = calib_data[0];
    cogging_index_temp = calib_data[1];
    cogging_target_angle_temp = calib_data[2];
    cogging_repeat_temp = calib_data[3];
    cogging_table_raw_count_temp = calib_data[4];
    cogging_table_iq_temp = calib_data[5];
}

/**
 * @brief 初始化齿槽标定模式
 * @note 流程：
 *       1) 初始化电流环 PI
 *       2) 初始化 FOC 句柄
 *       3) 执行零点对齐
 *       4) 以当前机械角为起点启动补偿表标定
 *       5) 注册 ADC 注入回调
 */
void coggingCalibrationMode_init(void)
{
    // 初始化 d/q 轴电流环 PI 控制器
    pid_init(&pid_id, PID_MODE_PI, CURRENT_PID_KP, CURRENT_PID_KI, 0.0f, CURRENT_PID_OUT_MIN, CURRENT_PID_OUT_MAX, PID_LIMIT_DISABLE);
    pid_init(&pid_iq, PID_MODE_PI, CURRENT_PID_KP, CURRENT_PID_KI, 0.0f, CURRENT_PID_OUT_MIN, CURRENT_PID_OUT_MAX, PID_LIMIT_DISABLE);

    // 初始化 FOC 控制句柄，标定模式只需要电流环
    foc_init(&foc_cogging_calib_handle, &pid_id, &pid_iq, NULL);
    foc_set_id(&foc_cogging_calib_handle, 0.0f);
    foc_set_iq(&foc_cogging_calib_handle, 0.0f);

    // 对齐编码器零点与电角度偏移
    zero_alignment(&foc_cogging_calib_handle);

    // 以当前机械角为单圈标定起点，并清零机械位置累计
    float start_mech_angle = encoder_get_mechanicalAngleBlock();
    encoder_reset_mechanicalPosition(0.0f);
    coggingCalib_init(&cogging_calib_handle, start_mech_angle);
    cogging_final_table_printed = 0U;
    cogging_final_table_cached = 0U;
    cogging_final_table_size = 0U;
    cogging_final_print_buffer[0] = '\0';

    // 注册电流环回调，进入标定运行
    adc_register_injectedCallback(cogging_calibration_mode_callback);
}

/**
 * @brief 输出齿槽标定调试信息
 * @note 当前发送的 6 个通道依次为：
 *       1) 标定状态
 *       2) 当前表索引
 *       3) 当前机械角(度)
 *       4) 当前目标机械角(度)
 *       5) 当前重复采样圈次
 *       6) 最近一次记录/当前平均的编码器原始计数
 *       7) 最近一次记录/当前平均的补偿 iq(A)
 */
void coggingCalibrationModeDebug_print_info(void)
{
    if (coggingCalib_isFinished(&cogging_calib_handle) == 0U)
    {
        return;
    }

    if (cogging_final_table_cached == 0U)
    {
        uint16_t idx;

        cogging_final_table_size = coggingCalib_getTableSize();
        if (cogging_final_table_size > COGGING_CALIB_TABLE_SIZE)
        {
            cogging_final_table_size = COGGING_CALIB_TABLE_SIZE;
        }

        for (idx = 0U; idx < cogging_final_table_size; ++idx)
        {
            cogging_final_raw_count_table[idx] = coggingCalib_getRawCountByIndex(&cogging_calib_handle, idx);
            cogging_final_iq_table[idx] = coggingCalib_getIqCompByIndex(&cogging_calib_handle, idx);
        }

        cogging_final_table_cached = 1U;
    }

    if (cogging_final_table_printed == 0U)
    {
        uint16_t idx;
        int print_len = 0;
        int written;

#define COGGING_FINAL_APPEND(...)                                                        \
    do                                                                                   \
    {                                                                                    \
        written = snprintf(&cogging_final_print_buffer[print_len],                       \
                           sizeof(cogging_final_print_buffer) - (size_t)print_len,       \
                           __VA_ARGS__);                                                 \
        if (written < 0)                                                                 \
        {                                                                                \
            return;                                                                      \
        }                                                                                \
        if ((size_t)written >= (sizeof(cogging_final_print_buffer) - (size_t)print_len)) \
        {                                                                                \
            print_len = (int)sizeof(cogging_final_print_buffer) - 1;                     \
        }                                                                                \
        else                                                                             \
        {                                                                                \
            print_len += written;                                                        \
        }                                                                                \
    } while (0)

        COGGING_FINAL_APPEND("/* COGGING_TABLE_BEGIN raw_count=%u iq_count=%u */\r\n",
                             (unsigned int)cogging_final_table_size,
                             (unsigned int)cogging_final_table_size);
        COGGING_FINAL_APPEND("#include \"cogging_comp.h\"\r\n\r\n");
        COGGING_FINAL_APPEND("const uint16_t g_cogging_comp_raw_count_table[COGGING_COMP_TABLE_SIZE] = {\r\n");

        for (idx = 0U; idx < cogging_final_table_size; ++idx)
        {
            if ((idx % 16U) == 0U)
            {
                COGGING_FINAL_APPEND("    ");
            }

            COGGING_FINAL_APPEND("%u%s",
                                 (unsigned int)cogging_final_raw_count_table[idx],
                                 (idx + 1U) < cogging_final_table_size ? "," : "");

            if ((((idx + 1U) % 16U) == 0U) || ((idx + 1U) >= cogging_final_table_size))
            {
                COGGING_FINAL_APPEND("\r\n");
            }
            else
            {
                COGGING_FINAL_APPEND(" ");
            }
        }

        COGGING_FINAL_APPEND("};\r\n\r\n");
        COGGING_FINAL_APPEND("const float g_cogging_comp_iq_table[COGGING_COMP_TABLE_SIZE] = {\r\n");

        for (idx = 0U; idx < cogging_final_table_size; ++idx)
        {
            if ((idx % 8U) == 0U)
            {
                COGGING_FINAL_APPEND("    ");
            }

            COGGING_FINAL_APPEND("%.6ff%s",
                                 (double)cogging_final_iq_table[idx],
                                 (idx + 1U) < cogging_final_table_size ? "," : "");

            if ((((idx + 1U) % 8U) == 0U) || ((idx + 1U) >= cogging_final_table_size))
            {
                COGGING_FINAL_APPEND("\r\n");
            }
            else
            {
                COGGING_FINAL_APPEND(" ");
            }
        }

        COGGING_FINAL_APPEND("};\r\n");
        COGGING_FINAL_APPEND("/* COGGING_TABLE_END raw_count=%u iq_count=%u */\r\n",
                             (unsigned int)cogging_final_table_size,
                             (unsigned int)cogging_final_table_size);

#undef COGGING_FINAL_APPEND

        cogging_final_table_printed = 1U;
        usart_send_data((uint8_t *)cogging_final_print_buffer, (uint16_t)print_len);
    }
}

#endif /* COGGING_CALIB_ENABLE */