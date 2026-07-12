#ifndef __INDUCTANCE_MEASURE_MODE_H__
#define __INDUCTANCE_MEASURE_MODE_H__

#include "../adv_alg/inductance_measure.h"
#include "../app/user_config.h"
#include "../foc/foc.h"
#include "../foc/gate_drive.h"
#include "../sensor/current_sense.h"
#include "../sensor/encoder.h"
#include "../utils/print.h"

void inductanceMeasureMode_init(void);
void inductanceMeasureModeDebug_print_info(void);

#endif /* __INDUCTANCE_MEASURE_MODE_H__ */