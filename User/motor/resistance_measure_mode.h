#ifndef __RESISTANCE_MEASURE_MODE_H__
#define __RESISTANCE_MEASURE_MODE_H__

#include "../adv_alg/resistance_measure.h"
#include "../app/user_config.h"
#include "../foc/foc.h"
#include "../sensor/current_sense.h"
#include "../sensor/encoder.h"
#include "../utils/print.h"

void resistanceMeasureMode_init(void);
void resistanceMeasureModeDebug_print_info(void);

#endif /* __RESISTANCE_MEASURE_MODE_H__ */