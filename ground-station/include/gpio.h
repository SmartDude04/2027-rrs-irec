#ifndef __GPIO_H__
#define __GPIO_H__

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_dma.h"

#define LED_PIN GPIO_PIN_13
#define LED_PORT GPIOC

#define RADIO_SPI_PORT GPIOA
#define RADIO_MOSI_PIN GPIO_PIN_7
#define RADIO_MISO_PIN GPIO_PIN_6
#define RADIO_SCK_PIN  GPIO_PIN_5
#define RADIO_NSS_PIN  GPIO_PIN_4


void LED_Init(void);

#endif