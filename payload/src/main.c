#include "stm32f4xx_hal.h"

void gpio_init();

int main() {
    HAL_Init();
    gpio_init();

    while (1) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(1000);
        
    }
}

void gpio_init() {
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef gpio_cfg;
    gpio_cfg.Pin = GPIO_PIN_13;
    gpio_cfg.Pull = GPIO_PULLUP;
    gpio_cfg.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_cfg.Speed = GPIO_SPEED_HIGH;

    HAL_GPIO_Init(GPIOC, &gpio_cfg);
}

void SysTick_Handler(void) {
    HAL_IncTick();
}