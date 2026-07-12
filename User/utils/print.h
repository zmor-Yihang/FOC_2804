#ifndef __PRINT_H__
#define __PRINT_H__

#include <stdio.h>
#include <string.h>
#include "../bsp/usart.h"
#include "stm32g4xx_hal.h"

void vofa_send(float *data, uint16_t num);

#endif /* __PRINT_H__ */
