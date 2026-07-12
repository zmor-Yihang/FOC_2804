#ifndef __AS5600_TEST_H__
#define __AS5600_TEST_H__

#include <stdio.h>
#include "../alg/clark_park.h"
#include "../app/user_config.h"
#include "../bsp/as5600.h"
#include "../foc/foc.h"
#include "../foc/gate_drive.h"
#include "../sensor/encoder.h"
#include "../utils/angle_utils.h"
#include "../utils/print.h"
#include "stm32g4xx_hal.h"

void as5600_test_init(void);
void as5600_test_poll(void);

#endif /* __AS5600_TEST_H__ */
