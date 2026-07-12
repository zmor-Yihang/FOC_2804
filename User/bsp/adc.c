#include "adc.h"

// ADC1 句柄
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

// 规则组 DMA 缓冲区
static uint16_t adc_regular_buf[2] = {0};

// 注入组数据缓冲区
static uint16_t adc_injected_buf[2] = {0};

// AB相电流零点偏移
static adc_offset_t adc_offset = {0};

// 注入组中断回调函数指针
static adc_injectedCallback_p adc_injected_callback = NULL;

// 注入组转换完成中断触发计数器
static volatile uint32_t adc_injected_irq_count = 0;

// 注入组用户回调实际执行计数器
static volatile uint32_t adc_injected_callback_count = 0;

/**
 * @brief  将 ADC 原始值转换为原始电压 (V)
 */
static inline float adc_raw_to_voltage(uint16_t adc_raw) {
    return (float)adc_raw * ADC_VREF / ADC_RESOLUTION; // ADC原始值 × 参考电压 / 最大计数值 = 实际电压
}

/**
 * @brief  零电流偏置标定
 *         在电机驱动未使能、无电流流过分流电阻时调用
 *         使用规则组 DMA 连续采样, 累加平均得到零点偏移
 * @note   标定结果 = 实际ADC电压 - 理论中点(1.65V) 的残余偏移
 *         后续电流计算时会减去该偏移
 */
static void adc_calibrate_offset(void) {
    float sum_ia = 0.0f; // A相累加器
    float sum_ib = 0.0f; // B相累加器

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_regular_buf, 2); // 启动规则组DMA采样
    HAL_Delay(ADC_CALIB_SETTLE_MS);                            // 等待电路稳定

    for (uint16_t i = 0; i < ADC_CALIB_SAMPLES; i++) {
        float vout_a = adc_raw_to_voltage(adc_regular_buf[0]); // 读取A相电压
        float vout_b = adc_raw_to_voltage(adc_regular_buf[1]); // 读取B相电压

        sum_ia += (vout_a - (ADC_VREF * 0.5f)); // 累加A相对理论中点电压的偏差
        sum_ib += (vout_b - (ADC_VREF * 0.5f)); // 累加B相对理论中点电压的偏差

        HAL_Delay(ADC_CALIB_DELAY_MS); // 采样间隔
    }

    HAL_ADC_Stop_DMA(&hadc1); // 停止DMA采样

    adc_offset.ia_offset = sum_ia / (float)ADC_CALIB_SAMPLES; // A相零点偏移 = 平均值
    adc_offset.ib_offset = sum_ib / (float)ADC_CALIB_SAMPLES; // B相零点偏移 = 平均值
}

/**
 * @brief  ADC1 初始化
 */
void adc_init(void) {
    // 使能外设时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();   // PA0(IN1), PA1(IN2) 模拟输入
    __HAL_RCC_DMAMUX1_CLK_ENABLE(); // DMA请求路由器
    __HAL_RCC_DMA1_CLK_ENABLE();    // DMA1_Channel3 (ADC1规则组)
    __HAL_RCC_ADC12_CLK_ENABLE();   // ADC1

    // PA0/IN1, PA1/IN2 模拟输入
    GPIO_InitTypeDef gpio_init = {0};
    gpio_init.Pin = GPIO_PIN_0 | GPIO_PIN_1; // PA0 + PA1 引脚
    gpio_init.Mode = GPIO_MODE_ANALOG;       // 模拟输入模式
    gpio_init.Pull = GPIO_NOPULL;            // 无上下拉
    HAL_GPIO_Init(GPIOA, &gpio_init);

    hdma_adc1.Instance = DMA1_Channel3;
    hdma_adc1.Init.Request = DMA_REQUEST_ADC1;                    // ADC1作为DMA请求源
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;              // 外设(ADC) -> 内存 方向
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;                  // 外设地址不自增
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;                      // 内存地址自增
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; // 外设半字对齐 (16bit)
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;    // 内存半字对齐 (16bit)
    hdma_adc1.Init.Mode = DMA_CIRCULAR;                           // 循环模式, 不断刷新缓冲区
    hdma_adc1.Init.Priority = DMA_PRIORITY_VERY_HIGH;             // 最高优先级, 保证实时性
    HAL_DMA_Init(&hdma_adc1);

    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1); // 将DMA句柄绑定到ADC1

    ADC_MultiModeTypeDef     multimode = {0};
    ADC_ChannelConfTypeDef   regular = {0};
    ADC_InjectionConfTypeDef injected = {0};

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4; // APB时钟4分频
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;           // 12位分辨率
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;           // 右对齐
    hadc1.Init.GainCompensation = 0;                      // 无增益补偿
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;            // 扫描模式, 多通道依次采样
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;        // 单次转换结束标志
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = ENABLE; // 连续转换模式
    hadc1.Init.NbrOfConversion = 2;         // 规则组通道数: IA + IB
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;                // 软件触发规则组
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE; // 规则组无外部触发边沿
    hadc1.Init.DMAContinuousRequests = ENABLE;                       // 持续DMA请求
    hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;                   // 溢出时覆盖旧数据
    hadc1.Init.OversamplingMode = ENABLE;                            // 启用过采样提高精度
    HAL_ADC_Init(&hadc1);

    multimode.Mode = ADC_MODE_INDEPENDENT; // 独立模式, 不使用双ADC
    HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode);

    // 规则通道1: PA0/IN1 (IA相)
    regular.Channel = ADC_CHANNEL_1;                  // PA0 -> ADC通道1
    regular.Rank = ADC_REGULAR_RANK_1;                // 规则组第1个转换
    regular.SamplingTime = ADC_SAMPLETIME_12CYCLES_5; // 12.5个周期采样时间
    regular.SingleDiff = ADC_SINGLE_ENDED;            // 单端输入
    regular.OffsetNumber = ADC_OFFSET_NONE;           // 无偏移补偿
    regular.Offset = 0;
    HAL_ADC_ConfigChannel(&hadc1, &regular);

    // 规则通道2: PA1/IN2 (IB相)
    regular.Channel = ADC_CHANNEL_2;   // PA1 -> ADC通道2
    regular.Rank = ADC_REGULAR_RANK_2; // 规则组第2个转换
    HAL_ADC_ConfigChannel(&hadc1, &regular);

    // 注入通道1: PA0/IN1 (IA相, TIM2_CC2中心点触发)
    injected.InjectedChannel = ADC_CHANNEL_1;
    injected.InjectedRank = ADC_INJECTED_RANK_1;
    injected.InjectedSamplingTime = ADC_SAMPLETIME_12CYCLES_5;
    injected.InjectedSingleDiff = ADC_SINGLE_ENDED;
    injected.InjectedOffsetNumber = ADC_OFFSET_NONE;
    injected.InjectedOffset = 0;
    injected.InjectedNbrOfConversion = 2; // 注入组2个通道
    injected.InjectedDiscontinuousConvMode = DISABLE;
    injected.AutoInjectedConv = DISABLE; // 不自动注入
    injected.QueueInjectedContext = DISABLE;
    injected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T2_TRGO;             // TIM2捕获比较事件触发
    injected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING; // 上升沿触发
    injected.InjecOversamplingMode = ENABLE;                                    // 注入组也启用过采样
    HAL_ADCEx_InjectedConfigChannel(&hadc1, &injected);

    // 注入通道2: PA1/IN2 (IB相)
    injected.InjectedChannel = ADC_CHANNEL_2;    // PA1 -> ADC通道2
    injected.InjectedRank = ADC_INJECTED_RANK_2; // 注入组第2个转换
    HAL_ADCEx_InjectedConfigChannel(&hadc1, &injected);

    // ADC 中断配置 (注入组转换完成中断, 用于FOC电流环回调)
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 4, 0); // 抢占优先级0, 子优先级0
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);         // 使能ADC1/ADC2全局中断

    // ADC 内部校准, 消除ADC本身偏移误差
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

    // 零电流偏置标定, 消除INA240+PCB+参考电压残余偏移
    adc_calibrate_offset();

    // 启动注入组中断采样, 等待TIM2_Update触发, 进入FOC后由PWM驱动
    HAL_ADCEx_InjectedStart_IT(&hadc1);
}

/**
 * @brief  获取标定得到的零点偏移量
 */
void adcDebug_get_offset(adc_offset_t *offsets) {
    offsets->ia_offset = adc_offset.ia_offset; // 输出A相零点偏移
    offsets->ib_offset = adc_offset.ib_offset; // 输出B相零点偏移
}

/**
 * @brief  获取最近一次注入组采样的原始值 (非阻塞)
 */
void adc_get_injectedRaw(adc_rawValues_t *values) {
    values->ia_raw = adc_injected_buf[0];
    values->ib_raw = adc_injected_buf[1];
}

/**
 * @brief  规则组采样获取当前原始值 (阻塞式, 调试用)
 */
void adcDebug_get_regularRaw(adc_rawValues_t *values) {
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_regular_buf, 2); // 启动规则组DMA采样
    HAL_Delay(10);                                             // 等待10ms让ADC完成多次转换
    values->ia_raw = adc_regular_buf[0];
    values->ib_raw = adc_regular_buf[1];
    HAL_ADC_Stop_DMA(&hadc1); // 停止DMA
}

uint32_t adcDebug_get_injectedIrqCount(void) {
    return adc_injected_irq_count;
}

uint32_t adcDebug_get_injectedCallbackCount(void) {
    return adc_injected_callback_count;
}

/**
 * @brief  注册注入组采样完成回调 (此回调中执行 FOC 计算)
 */
void adc_register_injectedCallback(adc_injectedCallback_p callback) {
    adc_injected_callback = callback; // 保存用户回调函数指针
}

/**
 * @brief  ADC1/ADC2 中断入口
 */
void ADC1_2_IRQHandler(void) {
    HAL_ADC_IRQHandler(&hadc1); // 交给HAL库处理ADC中断
}

/**
 * @brief  注入组转换完成回调
 *         由 TIM2_CC2 触发, 在 PWM 周期中固定时刻采样
 *         采样完成后调用用户注册的回调函数 (FOC 控制循环)
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) // 确认是ADC1的中断
    {
        adc_injected_irq_count++;
        adc_injected_buf[0] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1); // 读取IA相注入通道值
        adc_injected_buf[1] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2); // 读取IB相注入通道值

        if ((adc_injected_callback != NULL) && ((adc_injected_irq_count % ADC_INJECTED_CALLBACK_PRESCALER) == 0U)) // 按分频系数执行用户回调
        {
            adc_injected_callback_count++;
            adc_injected_callback(); // 执行用户FOC电流环回调函数
        }
    }
}
