#ifndef __GPIO_H__
#define __GPIO_H__

#include "stm32f4xx_hal.h"

#define LED_PIN GPIO_PIN_13
#define LED_PORT GPIOC

#define BTN_PIN GPIO_PIN_0
#define BTN_PORT GPIOA


void LED_Init(void);
void ARM_Btn_Init(void);

#endif