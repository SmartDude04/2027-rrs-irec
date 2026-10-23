#ifndef __ICM_H__
#define __ICM_H__

#include "I2C_DMA.h"
#include "stdbool.h"
#include "stm32f4xx_hal.h"

#define ICM_ADDR     0x69<<1 // correct for 7-bit addressing

#define ICM_accelXH  0x2D //registers for accel data
#define ICM_accelXL  0x2E
#define ICM_accelYH  0x2F
#define ICM_accelYL  0x30
#define ICM_accelZH  0x31
#define ICM_accelZL  0x32


#define PWR_MGMT_1   0x06 // basic chip level registers
#define PWR_MGMT_2   0x07
#define ICM_WHO_AM_I 0x00
#define ACCEL_CONFIG 0x14
#define ICM_BANK_CONFIG 0x7F



struct ICM {
    volatile bool busy;
    volatile bool new_data;
    volatile int16_t rawX;
    volatile float accelX;
    volatile int16_t rawY;
    volatile float accelY;
    volatile int16_t rawZ;
    volatile float accelZ;
    float offX;
    float offY;
    float offZ;
};

HAL_StatusTypeDef ICM_StartAccel(struct ICM *dev);
void ICM_GetAccel(struct ICM *dev);
void ICM_init(struct ICM *dev);



#endif