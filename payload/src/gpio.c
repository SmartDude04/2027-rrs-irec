#include "gpio.h"

void LED_Init() {
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef LED_cfg = {
        .Pin = GPIO_PIN_13,
        .Pull = GPIO_PULLUP,
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Speed = GPIO_SPEED_FREQ_MEDIUM
    };
    HAL_GPIO_Init(GPIOC, &LED_cfg);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}

