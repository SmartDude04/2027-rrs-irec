#ifndef __MPR_H__
#define __MPR_H__

#include "I2C_DMA.h"
#include "stdbool.h"
#include "stm32f4xx_hal.h"



#define MPR_I2C_ADDR     0x18 << 1 // I2C address of the MPR sensor
#define MPR_SCL      1u << 8 //GPIO that is SCL = PB8
#define MPR_SDA      1u << 4 //GPIO that is SDA = PA4
#define MPR_TIMEOUT_MS     20u
#define MPR_PRESSURE_MIN   0.0f          //0 psi
#define MPR_PRESSURE_MAX   25.0f         //25 psi

#define MPR_OUTPUT_MIN     0x19999Au     // Transfer function A: 10% of 2^24
#define MPR_OUTPUT_MAX     0xE66666u     // Transfer function A: 90% of 2^24


typedef struct  {
    uint8_t status;
    uint32_t raw_count;  // 24-bit sensor output stored in a 32-bit integer
    float pressure;      // Units match the configured pressure limits
    bool busy;          // Indicates if the sensor is currently busy
} MPR_Data;

typedef enum {
    MPR_OK = 0,         // Successful read
    MPR_I2C_ERROR = 1,  // I2C communication error
    MPR_TIMEOUT = 2,    // Timeout waiting for sensor to be ready
    MPR_SENSOR_ERROR = 3, // Sensor error indicated by status register
    MPR_POWER_ERROR = 5   // Sensor is powered down or in an error state
} MPR_Result;


MPR_Result MPR_Read(MPR_Data *result);

#endif
