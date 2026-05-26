#ifndef __IMPROVED_FLUX_OBSERVER_CLOSED_H__
#define __IMPROVED_FLUX_OBSERVER_CLOSED_H__

#include "../foc/foc.h"
#include "../adv_alg/improved_flux_observer.h"
#include "../sensor/current_sense.h"
#include "../sensor/encoder.h"
#include "../utils/print.h"
#include "../app/user_config.h"

void improvedFluxObserverClosed_init(float speed_rpm);
void improvedFluxObserverClosedDebug_print_info(void);

#endif /* __IMPROVED_FLUX_OBSERVER_CLOSED_H__ */