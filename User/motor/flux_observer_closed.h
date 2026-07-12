#ifndef __FLUX_OBSERVER_CLOSED_H__
#define __FLUX_OBSERVER_CLOSED_H__

#include "../adv_alg/flux_observer.h"
#include "../app/user_config.h"
#include "../foc/foc.h"
#include "../sensor/current_sense.h"
#include "../sensor/encoder.h"
#include "../utils/print.h"

void fluxObseverClosed_init(float speed_rpm);
void fluxObseverClosedDebug_print_info(void);

#endif /* __FLUX_OBSERVER_CLOSED_H__ */
