#include "tim.h"

TIM_HandleTypeDef htim2; // TIM2 HAL句柄：管理PA5/TIM2_CH1，并作为主定时器提供同步触发
TIM_HandleTypeDef htim3; // TIM3 HAL句柄：管理PA6/TIM3_CH1和PA7/TIM3_CH2，并作为从定时器运行

/**
 * @brief 初始化三相中心对齐PWM及ADC触发时基
 * @note TIM2先以TRGO_ENABLE触发TIM3同步启动，随后将TRGO切换为UPDATE事件，用于触发ADC注入组采样。
 */
void tim_init(void) {
    GPIO_InitTypeDef        gpio_init_struct   = {0}; // GPIO复用输出配置，清零可避免未赋值字段带入随机值
    TIM_OC_InitTypeDef      tim_oc_init_struct = {0}; // PWM输出比较通道配置
    TIM_MasterConfigTypeDef master_config      = {0}; // TIM2主定时器TRGO配置
    TIM_SlaveConfigTypeDef  slave_config       = {0}; // TIM3从定时器触发源和工作模式配置

    __HAL_RCC_GPIOA_CLK_ENABLE(); // 打开GPIOA外设时钟，使PA5、PA6、PA7配置寄存器可访问
    __HAL_RCC_TIM2_CLK_ENABLE();  // 打开TIM2外设时钟，TIM2作为三相PWM的主定时器
    __HAL_RCC_TIM3_CLK_ENABLE();  // 打开TIM3外设时钟，TIM3作为三相PWM的从定时器

    // PA5配置为TIM2_CH1复用输出
    gpio_init_struct.Pin       = GPIO_PIN_5;                // 选择PA5，对应TIM2_CH1输出引脚
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;           // 复用推挽输出，由定时器直接驱动高、低电平
    gpio_init_struct.Pull      = GPIO_NOPULL;               // 不启用内部上拉或下拉，避免影响外部驱动器输入
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH; // 使用最高GPIO翻转速度，减小PWM边沿延迟
    gpio_init_struct.Alternate = GPIO_AF1_TIM2;             // 将PA5复用功能连接到AF1中的TIM2_CH1
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);                // 将以上配置写入GPIOA相关寄存器

    // PA6、PA7沿用上面的推挽、无上下拉和高速配置，仅修改引脚与复用功能
    gpio_init_struct.Pin       = GPIO_PIN_6 | GPIO_PIN_7; // 同时选择PA6和PA7，分别对应TIM3_CH1和TIM3_CH2
    gpio_init_struct.Alternate = GPIO_AF2_TIM3;           // 将PA6、PA7连接到AF2中的TIM3通道
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);              // 将复用配置写入GPIOA相关寄存器

    // TIM2基础计数器配置：主定时器，输出PA5/TIM2_CH1
    htim2.Instance               = TIM2;                           // 句柄绑定TIM2寄存器基地址
    htim2.Init.Prescaler         = TIM1_PRESCALER;                 // 设置PSC预分频值，实际分频系数为PSC+1，当前为1分频
    htim2.Init.Period            = TIM1_PERIOD;                    // 设置ARR周期值，当前按170MHz时钟和10kHz中心对齐PWM计算
    htim2.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1; // 中心对齐模式1：计数器在0和ARR之间往返计数
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;         // tDTS采样时钟不分频；该字段不负责PWM计数频率分频
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;  // 启用ARR预装载，新周期值在更新事件时统一生效
    HAL_TIM_PWM_Init(&htim2);                                      // 初始化TIM2基础计数器和PWM功能

    // TIM2启动阶段使用CEN作为TRGO，CEN由0置1时触发TIM3启动
    master_config.MasterOutputTrigger = TIM_TRGO_ENABLE;            // MMS选择ENABLE：TRGO跟随TIM2的CEN使能信号
    master_config.MasterSlaveMode     = TIM_MASTERSLAVEMODE_ENABLE; // 使能主从同步模式，建立TIM2到从定时器的同步链路
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &master_config);  // 将TRGO和主从模式配置写入TIM2

    // TIM3基础计数器配置：从定时器，输出PA6/TIM3_CH1和PA7/TIM3_CH2
    htim3.Instance               = TIM3;                           // 句柄绑定TIM3寄存器基地址
    htim3.Init.Prescaler         = TIM1_PRESCALER;                 // 使用与TIM2相同的PSC，保证两个定时器计数速度一致
    htim3.Init.Period            = TIM1_PERIOD;                    // 使用与TIM2相同的ARR，保证两个定时器PWM周期一致
    htim3.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1; // 使用与TIM2相同的中心对齐计数模式
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;         // tDTS采样时钟不分频，不改变计数器输入时钟
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;  // 启用ARR预装载，使周期更新在更新事件边界生效
    HAL_TIM_PWM_Init(&htim3);                                      // 初始化TIM3基础计数器和PWM功能，此时尚未启动计数

    // TIM3从模式配置：接收TIM2的内部TRGO后启动计数
    slave_config.SlaveMode    = TIM_SLAVEMODE_TRIGGER; // 触发模式：TRGI出现有效边沿后设置CEN，此后TIM3连续运行
    slave_config.InputTrigger = TIM_TS_ITR1;           // 选择内部触发ITR1；在本芯片映射中ITR1连接TIM2_TRGO
    HAL_TIM_SlaveConfigSynchro(&htim3, &slave_config); // 将从模式和输入触发源写入TIM3

    // 三路PWM输出比较通道使用相同的模式、极性和初始占空比
    tim_oc_init_struct.OCMode     = TIM_OCMODE_PWM1;     // PWM模式1，根据CNT与CCR的比较结果生成有效脉冲
    tim_oc_init_struct.OCPolarity = TIM_OCPOLARITY_HIGH; // 输出有效电平为高电平，不反相PWM波形
    tim_oc_init_struct.OCFastMode = TIM_OCFAST_DISABLE;  // 关闭快速模式，保持标准比较输出时序
    tim_oc_init_struct.Pulse      = TIM1_PERIOD / 2;     // 初始CCR设为ARR的一半，三相初始占空比约为50%

    HAL_TIM_PWM_ConfigChannel(&htim2, &tim_oc_init_struct, TIM_CHANNEL_1); // 配置PA5对应的TIM2_CH1
    HAL_TIM_PWM_ConfigChannel(&htim3, &tim_oc_init_struct, TIM_CHANNEL_1); // 配置PA6对应的TIM3_CH1
    HAL_TIM_PWM_ConfigChannel(&htim3, &tim_oc_init_struct, TIM_CHANNEL_2); // 配置PA7对应的TIM3_CH2

    // 仅使能TIM3两个通道的引脚输出，不设置TIM3_CR1.CEN，因此TIM3计数器仍保持停止
    TIM3->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E); // 设置CC1E和CC2E，允许CH1、CH2比较信号输出到PA6、PA7

    // 两个定时器均未计数，可直接将初始计数位置统一为0
    TIM2->CNT = 0; // TIM2从中心对齐周期的底部开始计数
    TIM3->CNT = 0; // TIM3从与TIM2相同的计数位置等待触发

    // 启动TIM2_CH1：HAL同时使能CH1输出和TIM2计数器；CEN上升沿经TRGO触发TIM3硬件启动
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // TIM2、TIM3从CNT=0开始同步输出三相PWM

    // TIM3完成启动后，TRGO不再负责启动同步，改由TIM2更新事件触发ADC注入组转换
    TIM2->CR2 = (TIM2->CR2 & ~TIM_CR2_MMS) | TIM_TRGO_UPDATE; // 仅改写MMS字段为UPDATE，保留CR2中的其他配置位
}

/**
 * @brief 更新三相PWM占空比
 * @param duty1 逻辑A相占空比，调用方应保证范围为[0.0, 1.0]
 * @param duty2 逻辑B相占空比，调用方应保证范围为[0.0, 1.0]
 * @param duty3 逻辑C相占空比，调用方应保证范围为[0.0, 1.0]
 * @note 本函数不执行限幅；CCR已启用预装载，新比较值在对应定时器的更新事件时生效。
 */
void tim_set_pwmDuty(float duty1, float duty2, float duty3) {
    float duty_a = duty1; // 保存逻辑A相占空比
    float duty_b = duty2; // 保存逻辑B相占空比
    float duty_c = duty3; // 保存逻辑C相占空比

    uint32_t compare1 = (uint32_t)(duty_a * TIM1_PERIOD); // 将A相归一化占空比换算为CCR整数值，小数部分直接截断
    uint32_t compare2 = (uint32_t)(duty_b * TIM1_PERIOD); // 将B相归一化占空比换算为CCR整数值，小数部分直接截断
    uint32_t compare3 = (uint32_t)(duty_c * TIM1_PERIOD); // 将C相归一化占空比换算为CCR整数值，小数部分直接截断

#if (MOTOR_PHASE_SWAP == 0)
    // 正常相序：逻辑A、B、C依次映射到物理输出PA5、PA6、PA7
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, compare1); // 逻辑A相 -> PA5/TIM2_CH1
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, compare2); // 逻辑B相 -> PA6/TIM3_CH1
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, compare3); // 逻辑C相 -> PA7/TIM3_CH2
#else
    // 翻转相序：交换逻辑A、B的物理输出，逻辑C保持不变
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, compare2); // 逻辑B相 -> PA5/TIM2_CH1
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, compare1); // 逻辑A相 -> PA6/TIM3_CH1
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, compare3); // 逻辑C相 -> PA7/TIM3_CH2
#endif
}
