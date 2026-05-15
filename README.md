# FOC_2804 — 基于 STM32G431 的磁场定向控制（FOC）电机驱动

本项目是一款基于 **STM32G431RB** 微控制器的无刷电机（BLDC/PMSM）FOC 驱动方案，支持 **有感电流闭环、有感速度闭环、有感位置闭环、无感速度闭环** 四种运行模式。硬件上使用 **AS5600 磁编码器** 进行转子位置检测，电流采样通过片内 ADC 实现双电阻采样。

---

## 主要特性

- **电流环频率**：10 kHz
- **速度环频率**：1 kHz
- **位置环频率**：1 kHz
- **控制模式**：
  - 有感电流闭环（`currentClosed`）
  - 有感速度闭环（`speedClosed`）
  - 无感速度闭环（`fluxObserverClosed`，基于非线性磁链观测器）
  - 有感位置闭环（`positionClosed`，位置 PID 直接输出 q 轴电流）
- **调制方式**：零序注入SVPWM
- **解耦控制**：支持 D/Q 轴前馈解耦
- **拍延时补偿与角度插值**：针对 AS5600 I2C 通信与 PWM 更新延迟，结合 PLL 估计做电角度前向插值
- **预测齿槽转矩补偿**：支持离线标定与角度查表前馈补偿
- **速度环扰动观测器**：集成龙伯格 DOB，支持负载扰动估计与 q 轴电流前馈补偿
- **弱磁控制**：支持弱磁扩展转速（待完善）
- **调试输出**：支持 VOFA+ 波形调试

---

## 项目结构

```
FOC_2804/
├── User/
│   ├── app/              # 应用层：main 入口、用户配置
│   ├── bsp/              # 板级支持包：ADC、GPIO、TIM、USART、I2C、时钟
│   ├── sensor/           # 传感器驱动：AS5600 编码器、电流采样
│   ├── alg/              # 基础算法：Clarke/Park 变换、SVPWM、PID
│   ├── foc/              # FOC 核心：控制对象、环路调度、零点校准、栅极驱动
│   ├── motor/            # 应用级闭环：电流闭环、速度闭环、位置闭环、无感闭环
│   ├── adv_alg/          # 高级算法：磁链观测器、弱磁控制、齿槽补偿、龙伯格 DOB
│   └── utils/            # 工具库：快速三角函数、角度工具、斜坡、FIFO、打印
├── Drivers/              # STM32 HAL 库及 CMSIS
├── docs/                 # 调试记录与开发文档
├── 芯片手册/             # 芯片数据手册与参考手册
└── STM32G431RBTX_FLASH.ld  # 链接器脚本
```
---

## 目录说明

| 目录 | 说明 |
|------|------|
| `User/app` | `main.c` 程序入口，`user_config.h` 全局配置 |
| `User/bsp` | 底层外设初始化：时钟、GPIO、TIM（PWM）、ADC、USART、I2C |
| `User/sensor` | AS5600 编码器读取、电流采样及转换 |
| `User/alg` | Clarke/Park 坐标变换、SVPWM、PID 控制器 |
| `User/foc` | FOC 核心结构体、电流/速度环调度、零点校准、栅极驱动使能 |
| `User/motor` | 四种应用级闭环控制的封装（电流/速度/位置/无感） |
| `User/adv_alg` | 磁链观测器（用于无感控制）、弱磁控制、齿槽转矩补偿、龙伯格扰动观测器 |
| `User/utils` | 快速 sin/cos、角度归一化、斜坡函数、VOFA 打印、FIFO |

---

## 快速开始

### 1. 环境准备

- **IDE**：VS Code + EIDE 插件 + Cortex-Debug 插件
- **编译器**：GCC
- **调试器**：OpenOCD
- **串口助手**：VOFA+

### 2. 编译与烧录

1. 使用 VS Code 打开 `FOC_2804.code-workspace`
2. 在 EIDE 中选择对应的编译器工具链
3. 点击 **Build** 编译项目
4. 点击 **Download** 烧录固件

### 3. 选择运行模式

在 `User/app/main.c` 中按需要选择一种运行模式：

```c
// 电流闭环（仅控制 Id/Iq）
currentClosed_init(0.0f, 0.5f);

// 速度闭环（有感，编码器提供速度）
speedClosed_init(3000);

// 位置闭环（有感，位置 PID 直接输出 q 轴电流）
positionClosed_init(0.0f);

// 无感速度闭环（磁链观测器）
fluxObseverClosed_init(30);
```

同时，在 `while(1)` 中选择对应的调试输出：

```c
currentClosedDebug_print_info();       // 电流闭环调试
speedClosedDebug_print_info();         // 速度闭环调试
positionClosedDebug_print_info();      // 位置闭环调试
fluxObseverClosedDebug_print_info();   // 无感闭环调试
```

---

## 参数配置

电机与控制参数集中在 `User/app/user_config.h`：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `MOTOR_POLE_PAIRS` | 7 | 电机极对数 |
| `MOTOR_RS_Ω` | 1.5 Ω | 定子电阻 |
| `MOTOR_LD_H` / `MOTOR_LQ_H` | 0.86 mH | D/Q 轴电感 |
| `MOTOR_PSI_F` | 0.0035 Wb | 永磁体磁链 |
| `FOC_CURRENT_LOOP_FREQ_HZ` | 10000 | 电流环频率 |
| `FOC_SPEED_LOOP_DIVIDER` | 10 | 速度环分频系数 |
| `FOC_DECOUPLING_ENABLE` | 0 | 前馈解耦开关 |

### 核心算法说明

#### 1. 拍延时补偿与角度插值

AS5600 通过 I2C 读取时存在通信与采样延迟，PWM 更新也存在固有拍延时。为减少转子角度滞后对 FOC 定向精度的影响，编码器模块在 `encoder_get_pllAngle()` 中基于 PLL 速度估计做前向角度补偿：

```text
angle_comp = angle_pll + speed_pll * delay
```

这类处理本质上是把“当前时刻的角度”前推到“控制真正生效的时刻”，用于减小 Park 变换和 PWM 更新之间的相位误差。

#### 2. 预测齿槽转矩补偿

齿槽转矩会在低速和匀速工况下引入周期性转矩脉动。项目中通过离线标定得到 `raw_count -> iq_comp` 的补偿表，并在运行时按编码器原始计数做线性插值：

```text
iq_cogging = table(raw_count)
```

查表值以 q 轴电流形式注入到速度环参考值中，用于抵消周期性机械扰动。这样做的优点是针对性强，且不会把明显周期性的扰动全部留给速度环或 DOB 去处理。

#### 3. 无感速度闭环

无感模式通过磁链观测器替代编码器角度与速度反馈。观测器由定子电压、电流和磁链状态构成，先估计反电动势对应的磁链矢量，再通过 PLL 提取角度与速度：

```text
xhat_dot = y + correction(eta, s)
theta_est = atan2(eta_beta, eta_alpha)
```

其中 `fluxObserver_estimate()` 负责更新磁链状态，`fluxObserver_get_angle()` 和 `fluxObserver_get_speed()` 分别输出平滑角度与速度。无感模式适合在编码器不可用或需要去传感器时使用，但对参数和电压观测质量更敏感。

#### 4. 速度环龙伯格扰动观测器

速度闭环中可通过 `LUENBERGER_DOB_ENABLE` 启用负载扰动前馈补偿。观测器基于机械模型：

```text
omega_dot = -(B / J) * omega + (Kt / J) * iq + d
```

其中 `d_hat` 估计的是机械加速度维度的总扰动项，最终换算为 q 轴补偿电流：

```text
iq_comp = -(J / Kt) * d_hat * comp_gain
```

它与速度 PI 叠加后共同决定 `target_iq`，用于提升负载突变时的抗扰性能。相关参数位于 `User/app/user_config.h`：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `LUENBERGER_DOB_ENABLE` | 1 | 速度环负载扰动补偿开关 |
| `LUENBERGER_DOB_TS_S` | `1 / 10000 * 10` | 观测器执行周期，与速度环周期一致 |
| `LUENBERGER_DOB_J_KGM2` | `3.7e-6` | 转动惯量，需按实际电机和负载修正 |
| `LUENBERGER_DOB_B_NM_S_RAD` | `0.0` | 粘性摩擦系数，未知时可先设 0 |
| `LUENBERGER_DOB_KT_NM_A` | `1.5 * pole_pairs * psi_f` | q 轴转矩常数 |
| `LUENBERGER_DOB_BANDWIDTH_HZ` | `30.0` | 观测器带宽，越高响应越快但越容易放大测速噪声 |
| `LUENBERGER_DOB_ZETA` | `1.0` | 观测器阻尼系数 |
| `LUENBERGER_DOB_COMP_GAIN` | `1.0` | 补偿比例，调试时可从较小值逐步增大 |
| `LUENBERGER_DOB_IQ_COMP_MAX` | `0.2 * SPEED_PID_OUT_MAX` | q 轴补偿电流限幅 |

在 `speed_closed` 中，DOB 与速度环同周期更新，输出 `dob_iq_comp_temp` 与速度 PI、齿槽补偿共同叠加到 `target_iq`。调试输出末尾包含 `dob_iq_comp_temp`、`dob_d_hat_temp`、`dob_omega_hat_temp`，可用于观察补偿电流、扰动估计和速度估计。

#### 5. 位置环参数

位置闭环使用 PID 控制器直接输出 q 轴电流，D 项使用 PLL 速度反馈提供阻尼。相关参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `FOC_POSITION_LOOP_DIVIDER` | 10 | 位置环相对电流环的分频系数 |
| `POSITION_PID_KP` | 2.5 | 位置环 P 系数 |
| `POSITION_PID_KI` | 0.15 | 位置环 I 系数，用于消除恒定负载下的稳态误差 |
| `POSITION_PID_KD` | 0.04 | 位置环 D 系数，使用 PLL 速度反馈提供阻尼 |
| `POSITION_PID_D_FILTER_ALPHA` | 0.2 | D 项一阶滤波系数，1.0 表示不过滤 |
| `POSITION_PID_OUT_MIN` / `MAX` | ±0.8 A | 位置环输出 q 轴电流限幅 |

---

## 开发计划

完善弱磁控制
开发直接转矩控制（DTC）
