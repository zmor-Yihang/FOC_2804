# FOC 控制系统延迟补偿与 Park 变换易错点

> 本文档系统梳理 FOC 中"角度延迟补偿时间 `T_comp` 如何选取"以及"Park / 反 Park 角度为何不能共用"两个高频踩坑点，并结合本工程（STM32G431、中心对齐 PWM、编码器 PLL）给出具体结论与正确写法。
> 配套阅读：`docs/角度延时对电流矢量的影响分析.md`（讲现象与物理后果）。

---

## 一、统一的时间定义

所有补偿时间本质上都来自同一个公式：

$$
T_{\mathrm{comp}}=t_{\mathrm{target}}-t_{\mathrm{sample}}
$$

- `t_sample`：ADC 采样电流、读取角度的时刻；
- `t_target`：本次计算得到的电压矢量应当对应的时刻；
- `T_s`：控制周期，通常等于 PWM 周期；
- `t_target` 可取 PWM 新指令的**开始生效时刻**，也可取该指令的**等效作用中心时刻**。两种定义使补偿时间相差约 `0.5T_s`。

### 为什么"等效作用中心"重要

若在 `t=T_s` 装载一组新的 PWM 占空比，并在区间 `[T_s,2T_s]` 内保持不变。它虽从 `T_s` 开始生效，但按平均电压建模，等效作用时刻在该区间中心：

$$
t_{\mathrm{center}}=\frac{T_s+2T_s}{2}=1.5T_s
$$

若角度在 `t=0` 采样：
- 预测到 PWM 开始生效时刻 → 补偿 `1.0T_s`；
- 预测到 PWM 等效作用中心 → 补偿 `1.5T_s`。

这就是 `1.0T_s` 与 `1.5T_s` 的本质区别。

---

## 二、常见补偿档位对照

| 控制时序 | 新 PWM 开始生效 | 补偿到生效起点 | 补偿到作用中心 |
|---|---:|---:|---:|
| 当前周期即可生效 | `t_k` 附近 | 约 `0` | 约 `0.5T_s` |
| 下一周期生效 | `t_k+T_s` | `1.0T_s` | `1.5T_s` |
| 下下周期生效 | `t_k+2T_s` | `2.0T_s` | `2.5T_s` |

统一公式：

$$
T_{\mathrm{comp}}=N_dT_s+\frac{T_s}{2}
$$

其中 `N_d` 是从采样时刻到 PWM 开始生效之间的拍数：
- 当前周期生效：`N_d=0` → `T_comp=0.5T_s`；
- 下一周期生效：`N_d=1` → `T_comp=1.5T_s`；
- 下下周期生效：`N_d=2` → `T_comp=2.5T_s`。

> 注意：`0.5T_s / 1.0T_s / 1.5T_s / 2.0T_s` 中混用了两种定义——`1.0T_s`、`2.0T_s` 是"开始生效时刻"定义，`0.5T_s`、`1.5T_s` 是"作用中心"定义。两者不能直接并列比较。

---

## 三、20 kHz / 10 kHz 下的数值对照

本工程 `FOC_CURRENT_LOOP_FREQ_HZ = 10000`，中心对齐 PWM，故：

$$
T_s = \frac{1}{10\,\mathrm{kHz}} = 100\,\mu s
$$

| 补偿档位 | 补偿时间 |
|---:|---:|
| `0.5T_s` | `50μs` |
| `1.0T_s` | `100μs` |
| `1.5T_s` | `150μs` |
| `2.0T_s` | `200μs` |
| `2.5T_s` | `250μs` |

角度超前量统一为：

$$
\Delta\theta_e=\hat\omega_eT_{\mathrm{comp}}
$$

例如 24 r/min、7 极对时 `ω_e ≈ 17.59 rad/s`，各档对应电角度超前约 `0.0252° / 0.0504° / 0.0756° / 0.1008°`。**转速越高、`ω_e` 越大，相同的 `T_comp` 产生的角度误差越大**，这就是"拍延时陷阱"在高速区恶化的原因。

---

## 四、本工程实际属于哪一档

### 4.1 时序链路

| 项 | 配置 |
|---|---|
| PWM 模式 | 中心对齐（`CENTERALIGNED1`），10 kHz，`T_s=100μs` |
| ARR | `TIM1_PERIOD = 170e6/(10000·2)-1 = 8499` |
| ADC 采样触发 | TIM2 TRGO = UPDATE 事件（PWM 周期边界）|
| ISR 执行 | ADC 注入完成中断内立即做 Park+PI+反Park+`tim_set_pwmDuty` |
| PWM 装载 | CCR 预装载，新比较值在**下一 Update 事件**才生效 |
| 新占空比作用区间 | `[t_s+T_s, t_s+2T_s]`，整周期恒定 |

### 4.2 结论

- 新 PWM **开始生效**：`t_s + T_s` ⇒ 对应 `1.0T_s`（`z⁻¹` 一拍延时）；
- 新 PWM **平均作用中心**：区间中点 `t_s + 1.5T_s` ⇒ 对应 `1.5T_s`。

代码中 `ENCODER_PLL_ANGLE_COMP_DELAY_S = 1.5e-4 s = 150μs = 1.5T_s`，正是**"作用中心"定义**。对本工程这种"单更新点、新占空比保持一整周期"的中心对齐 PWM，`1.5T_s` 是比 `1.0T_s` 更贴合平均电压模型的选择，**不需要改成 `1T_s`**。

### 4.3 什么时候该写哪种

- 本拍采样、下拍 PWM 生效，论文/注释中若采用作用中心模型：
  > 控制量存在一个采样周期的更新延时。考虑新电压矢量在下一 PWM 周期内的平均作用时刻，预测时域取 `T_comp = 1.5T_s`，反 Park 变换角度修正为 `θ_e,invPark[k] = θ̂_e[k] + 1.5T_s·ω̂_e[k]`。
- 若仅表述传统"一拍延时补偿"：取 `T_comp = T_s`，并注明"预测到下一拍开始生效时刻"。

---

## 五、核心易错点：Park 与反 Park 不能共用同一超前角

### 5.1 原则

电流是**过去/此刻**采样到的物理量，Park 变换应使用**采样时刻**的角度：

$$
\theta_{\mathrm{Park}}=\hat\theta_e(t_{\mathrm{sample}})
$$

控制器算出的电压将在**未来**生效，反 Park 应使用**预测角度**：

$$
\theta_{\mathrm{invPark}}=\hat\theta_e(t_{\mathrm{sample}})+\hat\omega_eT_{\mathrm{comp}}
$$

### 5.2 为什么 Park 必须用原始角

Park 变换的职责是把静止坐标系电流投影到"角度为 `θ(t)` 的 d/q 旋转轴"上。`i_α,i_β` 物理上只属于采样那一刻，因此 `θ(t)` 必须是采样时刻的转子角 `θ_s`。

若误用未来角 `θ_s+δ`，得到的 `(i_d,i_q)` 等于真值绕原点旋转了 `-δ`：

$$
i_{d,\text{meas}}\approx i_d+i_q\sin\delta,\quad
i_{q,\text{meas}}\approx i_q-i_d\sin\delta
$$

**数值示例**：`θ_s=0`、真实电流 `i_d=0,i_q=10A`、`δ=30°`（夸张）：
- 正确（Park 用 `θ_s=0`）：`i_d=0, i_q=10` ✓
- 错误（Park 用 `θ_s+30°`）：`i_d≈5A, i_q≈8.66A` ✗

明明是纯 q 轴电流，却"测"出了 5A 的 d 轴分量。低速时 `δ` 小可忽略，**高速时 `δ` 变大，PI 看到假 `i_d` 会去纠正 `v_d`，制造 d/q 耦合——这正是"软件 I_d 始终在 0 附近、掩盖真实助磁"的假象来源**（详见 `docs/角度延时对电流矢量的影响分析.md`）。

### 5.3 为什么反 Park 反过来用未来角

反 Park 输出的 `v_α,v_β` 不是此刻生效，而是写入影子寄存器、到 `t_s+T_comp` 才变成真实电压作用到电机上。要在这个**未来时刻**把 `(v_d,v_q)` 指向正确物理方向，就必须用未来转子角 `θ_s+ω·T_comp`。

### 5.4 正确的程序结构

```c
/* —— 采样时刻的原始角度（Park 用）—— */
float theta_raw = wrap_0_2pi(phasePll_get_phase(&encoder_pll) - offset);

/* —— 反 Park 用的预测角（提前 T_comp）—— */
float omega_e   = speed_feedback * (MATH_TWO_PI / 60.0f) * MOTOR_POLE_PAIRS;
float theta_inv = wrap_0_2pi(theta_raw + omega_e * ENCODER_PLL_ANGLE_COMP_DELAY_S);

dq_t       i_dq        = park_transform(i_alphabeta, theta_raw);   /* Park：原始角 */
ud = pi_d(target_id - i_dq.d);
uq = pi_q(target_iq - i_dq.q);
alphabeta_t v_alphabeta = ipark_transform(v_dq, theta_inv);       /* 反Park：预测角 */
svpwm(v_alphabeta);
```

要点：
- Park 用 `theta_raw`（采样时刻）；
- 反 Park 用 `theta_inv`（预测 `T_comp` 之后）；
- **两者之差 = `ω_e·T_comp`**，这就是"补偿"真正生效的地方。

---

## 六、本工程当前代码的状态与建议

### 当前实现（`speed_closed.c` / `loop_control.c`）

```c
float angle_el = encoder_get_pllAngle() - offset;   /* 已 +ω·T_comp */
i_dq = park_transform(i_alphabeta, angle_el);        /* Park 用了预测角 ← 不符合原则 */
v_alphabeta = ipark_transform(v_dq, angle_el);       /* 反Park 用预测角 ← 这点对 */
```

- `ENCODER_PLL_ANGLE_COMP_DELAY_S = 1.5e-4 s`：取值 `1.5T_s`（作用中心定义），**对中心对齐单更新 PWM 是合适的**，无需改为 `1.0T_s`。
- 反 Park 用了预测角 ✓：电压正确应用到了未来转子角，应用方向正确。
- Park 也用了预测角 ✗：把"当前采样电流"投影到了未来坐标系，引入 d/q 耦合误差，制造软件 I_d 假象。

### 建议改造

把 `encoder_get_pllAngle()` 拆为两路：
1. 原始角 `phasePll_get_phase()` 供 Park 使用；
2. `phasePll_get_phase() + ω·T_comp` 供反 Park 使用。

`ENCODER_PLL_ANGLE_COMP_DELAY_S` 保持 `1.5e-4` 不变即可。改造后 `i_d` 测量值才反映真实的物理 d 轴电流，不再被"提前旋转"。

---

## 七、中心对齐 PWM 下还需逐项核对

仅知道"中心对齐 PWM"还不能确定 `T_comp` 是 `0.5/1.0/1.5T_s`，必须同时确认：

$$
\boxed{\text{ADC 采样点 + ISR 完成时间 + PWM 影子装载点}}
$$

例如：
- ADC 在 `CTR=ZERO` 采样；
- ISR 在 `CTR=PRD` 前完成；
- CMPA 在 `CTR=PRD` 装载；

那么新指令可能只延迟半个 PWM 周期开始生效。本工程采用 TRGO=UPDATE 触发 + 单更新点，新占空比延后一个完整周期生效，故落在 `N_d=1` 这一档。

---

## 八、速查结论

| 问题 | 结论 |
|---|---|
| 本工程 `T_comp` 取值 | `1.5e-4 s = 1.5T_s`（作用中心），合适 |
| 是否改为 `1.0T_s` | 不需要；`1.0T_s` 是开始生效定义，少补 `0.5T_s` |
| Park 用什么角 | 采样时刻原始角 `θ_s` |
| 反 Park 用什么角 | 预测角 `θ_s + ω·T_comp` |
| 两者能否共用 | 不能；共用会把电流投影到未来坐标系，制造 I_d 假象 |
| 高速下最该查什么 | 角度补偿时间 + Park/反Park 是否分离 + PLL 速度估计准确性 |
