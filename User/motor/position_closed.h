#ifndef __POSITION_CLOSED_H__
#define __POSITION_CLOSED_H__

#include "../app/user_config.h"
#include "../foc/foc.h"
#include "../sensor/current_sense.h"
#include "../sensor/encoder.h"
#include "../utils/print.h"

void positionClosed_init(float position_rad);
void positionClosedDebug_print_info(void);

#endif /* __POSITION_CLOSED_H__ */
