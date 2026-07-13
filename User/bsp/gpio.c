#include "gpio.h"

void gpio_init(void) {
    GPIO_InitTypeDef gpio_init_struct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE(); // 防止 MOSFET 开启

    HAL_GPIO_WritePin(MOSFET_ENA_PORT, MOSFET_ENA_PIN, GPIO_PIN_RESET);

    gpio_init_struct.Pin   = MOSFET_ENA_PIN;
    gpio_init_struct.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull  = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(MOSFET_ENA_PORT, &gpio_init_struct);
}

void gpio_m1_enable(void) {
    HAL_GPIO_WritePin(MOSFET_ENA_PORT, MOSFET_ENA_PIN, GPIO_PIN_SET);
}

void gpio_m1_disable(void) {
    HAL_GPIO_WritePin(MOSFET_ENA_PORT, MOSFET_ENA_PIN, GPIO_PIN_RESET);
}
