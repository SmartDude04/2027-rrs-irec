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

#define ICM_gyroXH   0x33 //registers for gyro data
#define GYRO_CONFIG 0x1B

#define PWR_MGMT_1   0x06 // basic chip level registers
#define PWR_MGMT_2   0x07
#define ICM_WHO_AM_I 0x00
#define ACCEL_CONFIG 0x14
#define ICM_BANK_CONFIG 0x7F


/**
 * @brief Structure holding all relevant data for the ICM-20948 device
 * 
 * @param aBusy Flag indicating if the accelerometer is busy reading data
 * @param gBusy Flag indicating if the gyroscope is busy reading data
 * @param new_accel_data Flag indicating if new accelerometer data is available
 * @param new_gyro_data Flag indicating if new gyroscope data is available
 * @param rawX Raw accelerometer X-axis data
 * @param accelX Real accelerometer X-axis data, converted to g's
 * @param rawY Raw accelerometer Y-axis data
 * @param accelY Real accelerometer Y-axis data, converted to g's
 * @param rawZ Raw accelerometer Z-axis data
 * @param accelZ Real accelerometer Z-axis data, converted to g's
 * @param rawGyroX Raw gyroscope X-axis data
 * @param gyroX Real gyroscope X-axis data, converted to degrees or radians
 * @param rawGyroY Raw gyroscope Y-axis data
 * @param gyroY Real gyroscope Y-axis data, converted to degrees or radians
 * @param rawGyroZ Raw gyroscope Z-axis data
 * @param gyroZ Real gyroscope Z-axis data, converted to degrees or radians
 */
struct ICM {
    volatile bool aBusy;
    volatile bool gBusy;
    volatile bool new_accel_data;
    volatile bool new_gyro_data;
    volatile int16_t rawX;
    volatile float accelX;
    volatile int16_t rawY;
    volatile float accelY;
    volatile int16_t rawZ;
    volatile float accelZ;
    volatile int16_t rawGyroX;
    volatile float gyroX;
    volatile int16_t rawGyroY;
    volatile float gyroY;
    volatile int16_t rawGyroZ;
    volatile float gyroZ;

};

HAL_StatusTypeDef ICM_StartAccel(struct ICM *dev);
HAL_StatusTypeDef ICM_StartGyro(struct ICM *dev);   
void ICM_GetAccel(struct ICM *dev);
void ICM_GetGyro(struct ICM *dev,bool degrees);
void ICM_init(struct ICM *dev);



#endif