#ifndef __I2C_DMA_H__
#define __I2C_DMA_H__

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_dma.h"

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

void I2C_Init(I2C_HandleTypeDef *hi2c);

#endif