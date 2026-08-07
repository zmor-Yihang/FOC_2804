#ifndef __ADC_H
#define __ADC_H

#include "stm32g4xx_hal.h"

#define ADC_VREF 3.3f          // ADC 参考电压
#define ADC_RESOLUTION 4096.0f // 12位 ADC 分辨率

/*
 * 过采样配置
 *
 * 硬件行为：OVSR 把 N 次转换结果累加，OVSS 右移 log2(N) 位才能把量程还原回 12 位。
 * 因此比率和右移必须成对出现；只要有一边漏配，raw 就会超出 0..4095，
 * 下游电流换算得到的是纯增益误差（零点仍准、斜率翻倍），示波器上很难察觉。
 *
 * 这里只声明 log2(N)，ratio 与 shift 从同一个数派生，两者无法各自漂移。
 */
#define ADC_OVS_CAT_(a, b) a##b
#define ADC_OVS_CAT(a, b)  ADC_OVS_CAT_(a, b) // 两级间接：先展开参数再拼接

#define ADC_OVS_POW2_1 2
#define ADC_OVS_POW2_2 4
#define ADC_OVS_POW2_3 8
#define ADC_OVS_POW2_4 16
#define ADC_OVS_POW2_5 32
#define ADC_OVS_POW2_6 64
#define ADC_OVS_POW2_7 128
#define ADC_OVS_POW2_8 256

#define ADC_OVS_RATIO(log2n) ADC_OVS_CAT(ADC_OVERSAMPLING_RATIO_, ADC_OVS_CAT(ADC_OVS_POW2_, log2n)) // log2(N) -> HAL 比率宏
#define ADC_OVS_SHIFT(log2n) ADC_OVS_CAT(ADC_RIGHTBITSHIFT_, log2n)                                 // log2(N) -> HAL 右移宏

#define ADC_OVS_LOG2_REGULAR 4  // 规则组 16× 过采样：零偏标定与调试轮询，慢速通道优先精度
#define ADC_OVS_LOG2_INJECTED 2 // 注入组 4× 过采样：FOC 电流环，采样窗口受 PWM 低边导通时间约束

#define ADC_CALIB_SAMPLES 256  // 零点标定采样次数
#define ADC_CALIB_DELAY_MS 1   // 每次标定采样间隔 (ms)
#define ADC_CALIB_SETTLE_MS 20 // 标定前等待模拟前端稳定 (ms)

#define ADC_INJECTED_CALLBACK_PRESCALER 2U // 注入组用户回调分频系数：1=不分频，2=2分频，3=3分频...

// AB相原始采样值
typedef struct {
    uint16_t ia_raw;
    uint16_t ib_raw;
} adc_rawValues_t;

// AB相零点偏移（ADC 码值域：零电流时的码值均值，已含采样电路中点与运放/PCB 残余偏移）
typedef struct {
    float ia_offset_counts;
    float ib_offset_counts;
} adc_offset_t;

// 注入组中断回调函数指针类型
typedef void (*adc_injectedCallback_p)(void);

void adc_init(void);
void adc_get_injectedRaw(adc_rawValues_t *values);
void adc_register_injectedCallback(adc_injectedCallback_p callback);

// 调试接口（统一使用 adc_dbg_ 前缀）
void     adcDebug_get_regularRaw(adc_rawValues_t *values); // 规则组阻塞式采样，仅调试用
void     adcDebug_get_offset(adc_offset_t *offsets);       // 获取ADC零点偏移
uint32_t adcDebug_get_injectedIrqCount(void);              // 注入组中断触发计数器
uint32_t adcDebug_get_injectedCallbackCount(void);         // 注入组用户ADC回调执行计数器，触发中断不一定执行回调

#endif /* __ADC_H */
