#ifndef __SPI_H__
#define __SPI_H__

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"
//#include "stm32f4xx_hal_dma.h"

extern SPI_HandleTypeDef hspi1;

void SPI_Init();

#endif