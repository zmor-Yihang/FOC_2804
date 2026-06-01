# FOC_2804 — STM32G431 FOC 电机驱动

基于 **STM32G431RB** 的 BLDC/PMSM 磁场定向控制工程。

---

## 目录结构

```text
FOC_2804/
├── User/
│   ├── app/        # 程序入口与全局配置
│   ├── bsp/        # ADC、GPIO、TIM、USART、I2C、时钟、AS5600 底层接口
│   ├── sensor/     # 编码器、电流采样
│   ├── alg/        # Clarke/Park、PID、SVPWM
│   ├── foc/        # FOC 控制对象、环路调度、零点对齐、栅极驱动
│   ├── motor/      # 各运行模式封装
│   ├── adv_alg/    # 磁链观测器、弱磁、DOB、齿槽补偿、参数辨识
│   ├── test/       # 外设与同步测试
│   └── utils/      # 角度、快速三角函数、打印、FIFO、宏工具
├── Drivers/        # STM32 HAL、CMSIS、启动文件、系统调用
├── docs/           # 调试记录与开发文档
│   └── videos/     # 不同运行模式的演示视频
├── .vscode/        # 调试和任务配置
├── FOC_2804.code-workspace
└── STM32G431RBTX_FLASH.ld
```

---

## 快速开始

### 1. 环境准备

- VS Code EIDE 插件
- Cortex-Debug 插件
- arm-none-eabi-gcc 工具链
- OpenOCD 或兼容调试器

### 2. 编译与烧录

1. 使用 VS Code 打开 `FOC_2804.code-workspace`
2. 在 EIDE 中选择对应的 GCC 工具链
3. 执行 Build
4. 执行 Download 或通过 Cortex-Debug 烧录调试

### 3. 切换运行模式

运行模式集中在 `User/app/main.c`。同一时间只保留一个初始化入口和一个调试输出入口。

```c
// 电流闭环
currentClosed_init(0.0f, 0.5f);
currentClosedDebug_print_info();

// 有感速度闭环
speedClosed_init(2);
speedClosedDebug_print_info();

// 有感位置闭环
positionClosed_init(0.0f);
positionClosedDebug_print_info();

// 弱磁速度闭环
speedWeakClosed_init(1000);
speedWeakClosedDebug_print_info();

// Ortega 无感速度闭环
fluxObseverClosed_init(500);
fluxObseverClosedDebug_print_info();

// MXLEMMING 无感速度闭环
mxlemmingObserverClosed_init(500);
mxlemmingObserverClosedDebug_print_info();

// 改进非线性磁链观测器无感速度闭环
improvedFluxObserverClosed_init(20);
improvedFluxObserverClosedDebug_print_info();

// 电阻辨识
resistanceMeasureMode_init();
resistanceMeasureModeDebug_print_info();

// 电感辨识
inductanceMeasureMode_init();
inductanceMeasureModeDebug_print_info();

// 齿槽转矩标定
coggingCalibrationMode_init();
coggingCalibrationModeDebug_print_info();
```

---

## 核心参数

参数集中在 `User/app/user_config.h`。

### 电机与采样

| 参数 | 当前值 | 说明 |
|------|--------|------|
| `U_DC` | 12.0 V | 直流母线电压 |
| `MOTOR_POLE_PAIRS` | 7 | 电机极对数 |
| `MOTOR_RS_Ω` | 1.6 Ω | 定子电阻 |
| `MOTOR_LD_H` / `MOTOR_LQ_H` | 0.67 mH | d/q 轴电感 |
| `MOTOR_PSI_F` | 0.0035 Wb | 永磁体磁链 |
| `MOTOR_PHASE_SWAP` | 1 | 启用相序翻转 |
| `FOC_CURRENT_LOOP_FREQ_HZ` | 10000 Hz | 电流环频率 |
| `FOC_SPEED_LOOP_DIVIDER` | 10 | 速度环分频 |
| `FOC_POSITION_LOOP_DIVIDER` | 10 | 位置环分频 |

### 编码器与角度补偿

| 参数 | 当前值 | 说明 |
|------|--------|------|
| `ENCODER_COUNT_SWAP` | 0 | 编码器计数方向翻转开关 |
| `ENCODER_PLL_KP` | 1776.0 | 编码器 PLL 比例增益 |
| `ENCODER_PLL_KI` | 1.58e6 | 编码器 PLL 积分增益 |
| `ENCODER_PLL_SPEED_LIMIT_RPM` | 5000 rpm | PLL 速度限幅 |
| `ENCODER_PLL_ANGLE_COMP_ENABLE` | 1 | 启用拍延时补偿 |
| `ENCODER_PLL_ANGLE_COMP_DELAY_S` | 1.5e-4 s | 角度前向补偿延时 |
| `FOC_ELEC_ANGLE_TRIM_RAD` | 0.0 rad | 电角度手动微调 |

### 环路控制

| 参数 | 当前值 | 说明 |
|------|--------|------|
| `CURRENT_PID_KP` / `CURRENT_PID_KI` | 5.0 / 1860.5 | 电流环 PI |
| `CURRENT_PID_OUT_MIN` / `MAX` | ±6 V | 电流环输出限幅 |
| `SPEED_PID_KP` / `SPEED_PID_KI` | 0.004 / 2.5 | 速度环 PI |
| `SPEED_PID_OUT_MIN` / `MAX` | ±0.8 A | 速度环输出 q 轴电流限幅 |
| `POSITION_PID_KP` / `KI` / `KD` | 2.5 / 0.15 / 0.04 | 位置环 PID |
| `POSITION_PID_D_FILTER_ALPHA` | 0.2 | 位置环 D 项滤波 |
| `POSITION_PID_OUT_MIN` / `MAX` | ±0.8 A | 位置环输出 q 轴电流限幅 |
| `FOC_DECOUPLING_ENABLE` | 0 | D/Q 轴前馈解耦开关 |

### 高级功能

| 功能 | 关键参数 | 当前状态 |
|------|----------|----------|
| 龙伯格扰动观测器 | `LUENBERGER_DOB_*` | 默认启用，补偿限幅为 `0.2 * SPEED_PID_OUT_MAX` |
| 齿槽转矩补偿 | `COGGING_COMP_*` | 默认关闭 |
| 齿槽离线标定 | `COGGING_CALIB_*` | 默认不编译标定代码 |
| 电阻辨识 | `RES_MEAS_*` | 独立运行模式 |
| 电感辨识 | `IND_MEAS_*` | 独立运行模式 |
| 弱磁控制(有 bug，待完善) | `FLUX_WEAK_*` | 已接入弱磁速度闭环 |

---

## 运行效果视频


---
## 开发计划

- 完善弱磁控制
- 开发直接转矩控制（DTC）