#ifndef __I2C_DMA_H__
#define __I2C_DMA_H__

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_dma.h"

extern I2C_HandleTypeDef hi2c2;

extern DMA_HandleTypeDef hdma_i2c2_rx;
extern DMA_HandleTypeDef hdma_i2c2_tx;
void I2C_Init(I2C_HandleTypeDef *hi2c);

#endif