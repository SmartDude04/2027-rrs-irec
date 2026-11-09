#ifndef AIRBRAKES_I2C_DMA_H
#define AIRBRAKES_I2C_DMA_H

#include "stm32h7xx_hal.h"

extern I2C_HandleTypeDef i2c_handle;

/**
 * Initializes I2C with PB6 as SCA and PB7 as SCL
 */
void I2C1_DMA_Init();

#endif