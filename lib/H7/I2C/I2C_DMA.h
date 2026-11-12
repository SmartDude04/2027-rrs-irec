#ifndef AIRBRAKES_I2C_DMA_H
#define AIRBRAKES_I2C_DMA_H

#include "stm32h7xx_hal.h"

extern I2C_HandleTypeDef i2c1_handle;
extern I2C_HandleTypeDef i2c2_handle;
extern I2C_HandleTypeDef i2c3_handle;

/**
 * Initializes I2C1 with PB6 as SCL and PB7 as SDA
 * Initializes I2C2 with PB10 as SCL and PB11 as SDA
 * Initializes I2C3 with PA8 as SCL and PC9 as SDA
 */
void I2C_DMA_Init();

#endif