#ifndef __SPEED_WEAK_CLOSED_H__
#define __SPEED_WEAK_CLOSED_H__

#include "../app/user_config.h"
#include "../foc/foc.h"
#include "../sensor/current_sense.h"
#include "../sensor/encoder.h"
#include "../utils/print.h"

void speedWeakClosed_init(float speed_rpm);
void speedWeakClosedDebug_print_info(void);

#endif /* __SPEED_WEAK_CLOSED_H__ */
