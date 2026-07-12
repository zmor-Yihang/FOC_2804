#ifndef __ADC_H
#define __ADC_H

#include "stm32g4xx_hal.h"

#define ADC_VREF 3.3f          // ADC 参考电压
#define ADC_RESOLUTION 4096.0f // 12位 ADC 分辨率

#define ADC_CALIB_SAMPLES 256  // 零点标定采样次数
#define ADC_CALIB_DELAY_MS 1   // 每次标定采样间隔 (ms)
#define ADC_CALIB_SETTLE_MS 20 // 标定前等待模拟前端稳定 (ms)

#define ADC_INJECTED_CALLBACK_PRESCALER 2U // 注入组用户回调分频系数：1=不分频，2=2分频，3=3分频...

// AB相原始采样值
typedef struct {
    uint16_t ia_raw;
    uint16_t ib_raw;
} adc_rawValues_t;

// AB相零点偏移
typedef struct {
    float ia_offset;
    float ib_offset;
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
