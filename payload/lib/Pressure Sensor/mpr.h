#ifndef __MPR_H__
#define __MPR_H__

#include "I2C_DMA.h"
#include "stdbool.h"
#include "stm32f4xx_hal.h"

#define MPR_ADDR     0x18

struct mpr {
    volatile bool busy;
    volatile bool new_data;
    volatile int32_t raw;
    float temp;
};

HAL_StatusTypeDef MPR_StartRead(struct mpr *dev);
void MPR_GetPressure(struct mpr *dev);

#endif
