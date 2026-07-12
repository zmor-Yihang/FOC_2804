#ifndef __GPIO_H__
#define __GPIO_H__

#include "stm32g4xx_hal.h"

// PA4  Mosfet驱动使能引脚
#define MOSFET_ENA_PORT GPIOA
#define MOSFET_ENA_PIN GPIO_PIN_4

void gpio_init(void);
void gpio_m1_enable(void);
void gpio_m1_disable(void);

#endif /* __GPIO_H__ */
