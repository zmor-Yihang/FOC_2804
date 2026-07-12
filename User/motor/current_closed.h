#ifndef __CURRENT_CLOSED_H__
#define __CURRENT_CLOSED_H__

#include "../bsp/gpio.h"
#include "../foc/foc.h"
#include "../sensor/current_sense.h"
#include "../sensor/encoder.h"
#include "../utils/print.h"

void currentClosed_init(float id, float iq);
void currentClosedDebug_print_info(void);

#endif /*__CURRENT_CLOSED_H__*/
