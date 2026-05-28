#ifndef __MAIN_H__
#define __MAIN_H__

#include "stm32g4xx_hal.h"
#include <stdio.h>

#include "../alg/clark_park.h"
#include "../bsp/adc.h"
#include "../bsp/clock.h"
#include "../bsp/gpio.h"
#include "../bsp/usart.h"

#include "../bsp/tim.h"
#include "../sensor/encoder.h"

#include "../motor/current_closed.h"
#include "../motor/speed_closed.h"
#include "../motor/speed_weak_closed.h"
#include "../motor/flux_observer_closed.h"
#include "../motor/position_closed.h"
#include "../motor/cogging_calibration_mode.h"
#include "../motor/mxlemming_observer_closed.h"
#include "../motor/improved_flux_observer_closed.h"
#include "../motor/resistance_measure_mode.h"
#include "../motor/inductance_measure_mode.h"





#endif /* __MAIN_H__ */
