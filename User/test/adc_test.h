#ifndef __ADC_TEST_H__
#define __ADC_TEST_H__

#include <stdio.h>
#include "../bsp/adc.h"
#include "../sensor/current_sense.h"
#include "../utils/print.h"
#include "stm32g4xx_hal.h"

void adc_test_init(void);
void adc_test_poll(void);

#endif /* __ADC_TEST_H__ */