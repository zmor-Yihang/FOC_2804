#include "current_sense.h"

/*
 * @brief  将 ADC 原始采样值转换为三相电流值
 * @param  raw      两路相电流 ADC 原始值（A/B 相）
 * @param  offsets  对应通道零电流码值（单位：count）
 * @param  currents 输出三相电流（单位：A）
 */
static void currentSense_convert_rawToCurrent(adc_rawValues_t *raw, current_sense_offset_t *offsets, abc_t *currents) {
    // 码值域直接去偏后一次换算到电流：零点已包含采样中点，无需再减参考电压
    float ia_hw = ((float)raw->ia_raw - offsets->ia_offset_counts) * CURRENT_SENSE_COUNTS_TO_AMPS;
    float ib_hw = ((float)raw->ib_raw - offsets->ib_offset_counts) * CURRENT_SENSE_COUNTS_TO_AMPS;

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
