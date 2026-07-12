#include "current_sense.h"

/*
 * @brief  将 ADC 原始采样值转换为三相电流值
 * @param  raw      两路相电流 ADC 原始值（A/B 相）
 * @param  offsets  对应通道零偏（单位：V）
 * @param  currents 输出三相电流（单位：A）
 */
static void currentSense_convert_rawToCurrent(const adc_rawValues_t *raw, const current_sense_offset_t *offsets, abc_t *currents) {
    // ADC 原始码值转采样电压：Vout = raw * Vref / resolution
    float vout_a = (float)raw->ia_raw * ADC_VREF / ADC_RESOLUTION; // A 相采样放大器输出电压
    float vout_b = (float)raw->ib_raw * ADC_VREF / ADC_RESOLUTION; // B 相采样放大器输出电压

    // 将采样电压还原为硬件测得相电流（未做控制坐标变换）
    float ia_hw = (vout_a - CURRENT_SENSE_REF_VOLTAGE - offsets->ia_offset) * CURRENT_SENSE_SCALE;
    float ib_hw = (vout_b - CURRENT_SENSE_REF_VOLTAGE - offsets->ib_offset) * CURRENT_SENSE_SCALE;

    // 三相平衡约束：Ia + Ib + Ic = 0
    float ic_hw = -(ia_hw + ib_hw);

#if (MOTOR_PHASE_SWAP == 0)
    // 标准相序映射
    currents->a = ia_hw;
    currents->b = ib_hw;
    currents->c = ic_hw;
#else
    // 硬件接线 A/B 相互换时的补偿映射
    currents->a = ib_hw;
    currents->b = ia_hw;
    currents->c = ic_hw;
#endif
}

/*
 * @brief  控制接口：读取注入通道 ADC 并输出三相电流
 * @param  currents 输出三相电流（单位：A）
 */
void currentSense_get_injectedValue(abc_t *currents) {
    adc_rawValues_t        raw;
    current_sense_offset_t offsets;

    adc_get_injectedRaw(&raw);     // 读取注入组采样值（通常与 PWM 同步）
    adcDebug_get_offset(&offsets); // 读取当前零偏
    currentSense_convert_rawToCurrent(&raw, &offsets, currents);
}

/*
 * @brief  调试接口：读取常规通道 ADC 并输出三相电流
 * @param  currents 输出三相电流（单位：A）
 */
void currentSenseDebug_get_regularValue(abc_t *currents) {
    adc_rawValues_t        raw;
    current_sense_offset_t offsets;

    adcDebug_get_regularRaw(&raw); // 读取常规组采样值
    adcDebug_get_offset(&offsets); // 读取当前零偏
    currentSense_convert_rawToCurrent(&raw, &offsets, currents);
}

/*
 * @brief  调试接口：导出当前电流采样零偏
 * @param  offsets 输出零偏结构体
 */
void currentSenseDebug_get_offset(current_sense_offset_t *offsets) {
    adcDebug_get_offset(offsets);
}
