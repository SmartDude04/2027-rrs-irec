#include "gpio.h"

void LED_Init() {
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef LED_cfg = {
        .Pin = LED_PIN,
        .Pull = GPIO_PULLUP,
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Speed = GPIO_SPEED_FREQ_MEDIUM
    };
    HAL_GPIO_Init(LED_PORT, &LED_cfg);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
}