#include "tim.h"

TIM_HandleTypeDef htim2; // TIM2句柄
TIM_HandleTypeDef htim3; // TIM3句柄

void tim_init(void) {
    GPIO_InitTypeDef        gpio_init_struct = {0};
    TIM_OC_InitTypeDef      tim_oc_init_struct = {0};
    TIM_MasterConfigTypeDef master_config = {0};
    TIM_SlaveConfigTypeDef  slave_config = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    // GPIO配置
    gpio_init_struct.Pin = GPIO_PIN_5;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init_struct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);

    gpio_init_struct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio_init_struct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);

    // TIM2
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = TIM1_PRESCALER;
    htim2.Init.Period = TIM1_PERIOD;
    htim2.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim2);

    // 使用 TRGO_ENABLE，CEN置位瞬间就产生触发
    master_config.MasterOutputTrigger = TIM_TRGO_ENABLE;
    master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &master_config);

    // TIM3
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = TIM1_PRESCALER;
    htim3.Init.Period = TIM1_PERIOD;
    htim3.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim3);

    slave_config.SlaveMode = TIM_SLAVEMODE_TRIGGER;
    slave_config.InputTrigger = TIM_TS_ITR1; // TIM3的ITR1 = TIM2
    HAL_TIM_SlaveConfigSynchro(&htim3, &slave_config);

    // PWM通道配置
    tim_oc_init_struct.OCMode = TIM_OCMODE_PWM1;
    tim_oc_init_struct.OCPolarity = TIM_OCPOLARITY_HIGH;
    tim_oc_init_struct.OCFastMode = TIM_OCFAST_DISABLE;
    tim_oc_init_struct.Pulse = TIM1_PERIOD / 2;

    HAL_TIM_PWM_ConfigChannel(&htim2, &tim_oc_init_struct, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim3, &tim_oc_init_struct, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim3, &tim_oc_init_struct, TIM_CHANNEL_2);

    // TIM3 只开通道输出，不启动计数器（不设CEN）
    TIM3->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E);
    // 此时 TIM3 的 CEN=0，等待 TIM2 触发

    // 确保两个计数器都从0开始
    TIM2->CNT = 0;
    TIM3->CNT = 0;

    // 启动 TIM2 → CEN=1 → TRGO_ENABLE触发 → TIM3硬件自动CEN=1
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    // 同步启动后，将TRGO改为UPDATE用于ADC触发
    TIM2->CR2 = (TIM2->CR2 & ~TIM_CR2_MMS) | TIM_TRGO_UPDATE;
}

void tim_set_pwmDuty(float duty1, float duty2, float duty3) {
    float duty_a = duty1;
    float duty_b = duty2;
    float duty_c = duty3;

    uint32_t compare1 = (uint32_t)(duty_a * TIM1_PERIOD);
    uint32_t compare2 = (uint32_t)(duty_b * TIM1_PERIOD);
    uint32_t compare3 = (uint32_t)(duty_c * TIM1_PERIOD);

#if (MOTOR_PHASE_SWAP == 0)
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, compare1); // 写入A相CCR（PA5/TIM2_CH1）
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, compare2); // 写入B相CCR（PA6/TIM3_CH1）
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, compare3); // 写入C相CCR（PA7/TIM3_CH2）
#else
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, compare2); // 写入A相CCR（PA5/TIM2_CH1）
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, compare1); // 写入B相CCR（PA6/TIM3_CH1）
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, compare3); // 写入C相CCR（PA7/TIM3_CH2）
#endif
}
