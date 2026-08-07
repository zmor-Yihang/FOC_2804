#ifndef __CURRENT_SENSE_H__
#define __CURRENT_SENSE_H__

#include "../alg/clark_park.h"
#include "../app/user_config.h"
#include "../bsp/adc.h"

#define CURRENT_SENSE_SHUNT_RES 0.01f                                                   // 分流电阻(Ω)
#define CURRENT_SENSE_AMP_GAIN 50.0f                                                    // 电流采样运放增益(V/V)
#define CURRENT_SENSE_SCALE (1.0f / (CURRENT_SENSE_AMP_GAIN * CURRENT_SENSE_SHUNT_RES)) // 电流换算系数(A/V)

/*
 * 码值 -> 电流 的合并系数(A/count)：(Vref/分辨率) × (1/(增益×分流电阻))
 * 采样中点不出现在这里：零点由标定得到的码值承载，减去它即完成去偏。
 */
#define CURRENT_SENSE_COUNTS_TO_AMPS (ADC_VREF / ADC_RESOLUTION * CURRENT_SENSE_SCALE)

typedef adc_offset_t current_sense_offset_t;

void currentSense_get_injectedValue(abc_t *currents);
void currentSenseDebug_get_regularValue(abc_t *currents);
void currentSenseDebug_get_offset(current_sense_offset_t *offsets);

#endif /* __CURRENT_SENSE_H__ */
