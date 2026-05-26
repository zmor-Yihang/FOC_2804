#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

// 直流母线电压
#define U_DC                                      12.0

// 数学常量
#define MATH_PI                                   3.14159265358979323846f       // 圆周率 PI
#define MATH_TWO_PI                               (2.0f * MATH_PI)              // 2*PI
#define MATH_INV_SQRT3                            0.57735026919f                // 1/sqrt(3)
#define MATH_SQRT3_BY_2                           0.86602540378f                // sqrt(3)/2

// 电机参数
#define MOTOR_POLE_PAIRS                          7                              // 电机极对数
#define MOTOR_RS_Ω                                1.6f                           // 定子电阻
#define MOTOR_LD_H                                0.86e-3f                       // d轴电感(H)
#define MOTOR_LQ_H                                0.86e-3f                       // q轴电感(H)
#define MOTOR_PSI_F                               0.0035f                        // 永磁体磁链(Wb)
#define MOTOR_PHASE_SWAP                          1                              // 1：启用相序翻转

// 控制参数
#define FOC_CURRENT_LOOP_FREQ_HZ                  10000.0f                       // 电流环执行频率(Hz)
#define FOC_SPEED_LOOP_DIVIDER                    10U                            // 速度环相对电流环的分频系数
#define FOC_DECOUPLING_ENABLE                     0                              // 1:启用前馈解耦 0:关闭前馈解耦
#define FOC_ELEC_ANGLE_TRIM_RAD                   0.00f                          // 电角度手动微调量(rad)，正常情况下保持为0，仅在排查固定偏差时临时微调

// 编码器参数
#define ENCODER_COUNT_SWAP                        0                              // 1：启用编码器计数翻转
#define ENCODER_PLL_KP                            1776.0f                        // 编码器速度 PLL 比例增益  200Hz带宽
#define ENCODER_PLL_KI                            1.58e6f                        // 编码器速度 PLL 积分增益
#define ENCODER_PLL_SPEED_LIMIT_RPM               5000.0f                        // 编码器 PLL 机械转速限幅(rpm)
#define ENCODER_PLL_ANGLE_COMP_ENABLE             1                              // 1：启用拍延时补偿
#define ENCODER_PLL_ANGLE_COMP_DELAY_S            1.5e-4f                        // 输出补偿延时(s)

// 电流环控制参数
#define CURRENT_PID_KP                            5.0f                           // 电流环PI比例系数  带宽1000Hz
#define CURRENT_PID_KI                            2670.0f                        // 电流环PI积分系数
#define CURRENT_PID_OUT_MIN                       (-U_DC / 2.0f)                 // 电流环输出下限
#define CURRENT_PID_OUT_MAX                       (U_DC / 2.0f)                  // 电流环输出上限

// 速度环参数
#define SPEED_PID_KP                              0.004f                         // 速度环PI比例系数  // 按 δ = 16 整定的，带宽1000 / 16 = 62.5Hz
#define SPEED_PID_KI                              2.5f                           // 速度环PI积分系数
#define SPEED_PID_OUT_MIN                         -0.8f                          // 速度环输出下限
#define SPEED_PID_OUT_MAX                         0.8f                           // 速度环输出上限

// 位置环参数：输入机械位置rad，PID直接输出q轴目标电流A
#define FOC_POSITION_LOOP_DIVIDER                 10U                            // 位置环相对电流环的分频系数
#define POSITION_PID_KP                           2.5f                           // 位置环P系数
#define POSITION_PID_KI                           0.15f                          // 位置环I系数，用于消除恒定负载下的稳态误差
#define POSITION_PID_KD                           0.04f                          // 位置环D系数，D项使用PLL速度反馈，提供阻尼作用
#define POSITION_PID_D_FILTER_ALPHA               0.2f                           // 位置环D项一阶滤波系数(alpha)，1.0表示不过滤
#define POSITION_PID_OUT_MIN                      -0.8f                          // 位置环输出q轴电流下限(A)
#define POSITION_PID_OUT_MAX                      0.8f                           // 位置环输出q轴电流上限(A)

// 弱磁控制参数
#define FLUX_WEAK_U_REF_RATIO                     0.95f                          // 弱磁电压参考比例
#define FLUX_WEAK_KP                              0.15f                          // 弱磁PI比例系数
#define FLUX_WEAK_KI                              3000.0f                        // 弱磁PI积分系数
#define FLUX_WEAK_ID_MIN                          -0.8f                          // 弱磁d轴最小电流(A)
#define FLUX_WEAK_VOLTAGE_FILTER_CONST            0.05f                          // 弱磁电压反馈一阶滤波系数(alpha)

// 电阻辨识参数
#define RES_MEAS_TARGET_CURRENT                   0.6f                           // 辨识注入电流 (A)
#define RES_MEAS_RAMP_RATE                        1.0f                           // 电流爬升速率 (A/s)
#define RES_MEAS_TS_S                             (1.0f / FOC_CURRENT_LOOP_FREQ_HZ) // 执行周期 (s)
#define RES_MEAS_SETTLE_TICKS                     2000U                          // 稳态等待拍数 (200ms @10kHz)
#define RES_MEAS_SAMPLE_COUNT                     5000U                          // 采样累积拍数 (500ms @10kHz)

// MXLEMMING 观测器参数
#define MXLEMMING_OBSERVER_CURRENT_PID_KP         5.0f                           // 无感模式电流环PI比例系数
#define MXLEMMING_OBSERVER_CURRENT_PID_KI         2670.0f                        // 无感模式电流环PI积分系数
#define MXLEMMING_OBSERVER_SPEED_PID_KP           0.004f                         // 无感模式速度环PI比例系数
#define MXLEMMING_OBSERVER_SPEED_PID_KI           2.5f                           // 无感模式速度环PI积分系数
#define MXLEMMING_OBSERVER_SPEED_PID_OUT_MIN      -0.8f                          // 无感模式速度环输出下限(A)
#define MXLEMMING_OBSERVER_SPEED_PID_OUT_MAX      0.8f                           // 无感模式速度环输出上限(A)
#define MXLEMMING_OBSERVER_TS_S                   (1.0f / FOC_CURRENT_LOOP_FREQ_HZ) // 无感模式观测器执行周期(s)
#define MXLEMMING_OBSERVER_PLL_KP                 1776.0f                        // 无感模式PLL比例增益
#define MXLEMMING_OBSERVER_PLL_KI                 1.58e6f                        // 无感模式PLL积分增益
#define MXLEMMING_OBSERVER_PLL_SPEED_LIMIT_RPM    5000.0f                        // 无感模式机械转速限幅(rpm)

// 无感磁链观测器参数（Ortega）
#define FLUX_OBSERVER_GAMMA                       2.0e7f                         // 观测器非线性增益
#define FLUX_OBSERVER_CURRENT_PID_KP              5.0f                           // 无感模式电流环PI比例系数
#define FLUX_OBSERVER_CURRENT_PID_KI              2670.0f                        // 无感模式电流环PI积分系数
#define FLUX_OBSERVER_SPEED_PID_KP                0.004f                         // 无感模式速度环PI比例系数
#define FLUX_OBSERVER_SPEED_PID_KI                2.5f                           // 无感模式速度环PI积分系数
#define FLUX_OBSERVER_SPEED_PID_OUT_MIN           -0.8f                          // 无感模式速度环输出下限(A)
#define FLUX_OBSERVER_SPEED_PID_OUT_MAX           0.8f                           // 无感模式速度环输出上限(A)

#define FLUX_OBSERVER_TS_S                        (1.0 / FOC_CURRENT_LOOP_FREQ_HZ) // 观测器执行周期(s)
#define FLUX_OBSERVER_PLL_KP                      1776.0f                        // PLL比例增益
#define FLUX_OBSERVER_PLL_KI                      1.58e6f                        // PLL积分增益
#define FLUX_OBSERVER_PLL_SPEED_LIMIT_RPM         5000.0f                        // 机械转速限幅(rpm)

// 改进非线性磁链观测器参数（论文 Eq.(15)）
#define IMPROVED_FLUX_OBSERVER_GAIN               3.53e7f                         // 观测器幅值搜索增益 λ
#define IMPROVED_FLUX_OBSERVER_PHASE_GAIN_K       1.0f                           // 正交相位搜索增益 k
#define IMPROVED_FLUX_OBSERVER_CURRENT_PID_KP     5.0f                           // 无感模式电流环PI比例系数
#define IMPROVED_FLUX_OBSERVER_CURRENT_PID_KI     2670.0f                        // 无感模式电流环PI积分系数
#define IMPROVED_FLUX_OBSERVER_SPEED_PID_KP       0.004f                         // 无感模式速度环PI比例系数
#define IMPROVED_FLUX_OBSERVER_SPEED_PID_KI       2.5f                           // 无感模式速度环PI积分系数
#define IMPROVED_FLUX_OBSERVER_SPEED_PID_OUT_MIN  -0.8f                          // 无感模式速度环输出下限(A)
#define IMPROVED_FLUX_OBSERVER_SPEED_PID_OUT_MAX  0.8f                           // 无感模式速度环输出上限(A)
#define IMPROVED_FLUX_OBSERVER_TS_S               (1.0f / FOC_CURRENT_LOOP_FREQ_HZ) // 观测器执行周期(s)
#define IMPROVED_FLUX_OBSERVER_PLL_KP             1776.0f                        // PLL比例增益
#define IMPROVED_FLUX_OBSERVER_PLL_KI             1.58e6f                        // PLL积分增益
#define IMPROVED_FLUX_OBSERVER_PLL_SPEED_LIMIT_RPM 5000.0f                       // 机械转速限幅(rpm)

// 龙伯格扰动观测器参数
#define LUENBERGER_DOB_ENABLE                     1U                             // 1:启用速度环负载扰动补偿 0:关闭
#define LUENBERGER_DOB_TS_S                       ((1.0f / FOC_CURRENT_LOOP_FREQ_HZ) * (float)FOC_SPEED_LOOP_DIVIDER) // 观测器执行周期(s)
#define LUENBERGER_DOB_J_KGM2                     3.7e-6f                        // 转动惯量(kg·m²)，需按实际负载修正
#define LUENBERGER_DOB_B_NM_S_RAD                 0.0f                           // 粘性摩擦系数(N·m·s/rad)，未知可先设0
#define LUENBERGER_DOB_KT_NM_A                    (1.5f * (float)MOTOR_POLE_PAIRS * MOTOR_PSI_F) // q轴转矩常数(N·m/A)
#define LUENBERGER_DOB_BANDWIDTH_HZ               30.0f                          // 观测器带宽(Hz)
#define LUENBERGER_DOB_ZETA                       1.0f                           // 观测器阻尼系数
#define LUENBERGER_DOB_COMP_GAIN                  1.0f                           // 补偿开启比例，调试稳定后可逐步增大
#define LUENBERGER_DOB_IQ_COMP_MAX                (0.2f * SPEED_PID_OUT_MAX)     // q轴补偿电流限幅(A)

// 齿槽转矩补偿参数
#define COGGING_COMP_ENABLE                       0U                             // 1:启用齿槽转矩补偿前馈 0:关闭齿槽转矩补偿前馈
#define COGGING_CALIB_ENABLE                      0U                             // 1:编译离线标定代码 0:只使用已有补偿表前馈
#define COGGING_COMP_TABLE_SIZE                   512U                           // 单圈齿槽补偿表点数
#define COGGING_CALIB_TABLE_SIZE                  COGGING_COMP_TABLE_SIZE        // 补偿表点数
#define COGGING_CALIB_SETTLE_TICKS                200U                           // 每个采样点到位后的稳定等待拍数
#define COGGING_CALIB_REPEAT_COUNT                10U                            // 整张单圈补偿表重复采样次数
#define COGGING_CALIB_MAX_MECH_SPEED_RPM          5.0f                           // 允许记录数据时的最大机械转速(rpm)
#define COGGING_CALIB_POSITION_KP                 0.8f                           // 标定用位置 PD 的比例系数
#define COGGING_CALIB_POSITION_KD                 0.02f                          // 标定用位置 PD 的微分系数
#define COGGING_CALIB_POSITION_OUT_MIN            -0.8f                          // 标定用 q 轴目标电流下限(A)
#define COGGING_CALIB_POSITION_OUT_MAX            0.8f                           // 标定用 q 轴目标电流上限(A)

// 数据类型
typedef struct
{
    float a;
    float b;
    float c;
} abc_t;       // 三相坐标系

typedef struct
{
    float alpha;
    float beta;
} alphabeta_t; // 两相静止坐标系

typedef struct
{
    float d;
    float q;
} dq_t;        // 两相旋转坐标系

#endif /* __USER_CONFIG_H__ */
