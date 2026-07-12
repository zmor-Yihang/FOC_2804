#ifndef __SPEED_CLOSED_H__
#define __SPEED_CLOSED_H__

#include "../adv_alg/cogging_comp.h"
#include "../adv_alg/luenberger_dob.h"
#include "../app/user_config.h"
#include "../foc/foc.h"
#include "../sensor/current_sense.h"
#include "../sensor/encoder.h"
#include "../utils/print.h"

void speedClosed_init(float speed_rpm);
void speedClosedDebug_print_info(void);

#endif /* __SPEED_CLOSED_H__ */
