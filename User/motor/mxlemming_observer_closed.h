#ifndef __MXLEMMING_OBSERVER_CLOSED_H__
#define __MXLEMMING_OBSERVER_CLOSED_H__

#include "../foc/foc.h"
#include "../adv_alg/mxlemming_observer.h"
#include "../sensor/current_sense.h"
#include "../sensor/encoder.h"
#include "../utils/print.h"
#include "../app/user_config.h"

void mxlemmingObserverClosed_init(float speed_rpm);
void mxlemmingObserverClosedDebug_print_info(void);

#endif /* __MXLEMMING_OBSERVER_CLOSED_H__ */