#ifndef __SPI_H__
#define __SPI_H__

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_dma.h"

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

void SPI_Init();

#endif