#ifndef AIRBRAKES_ICM_20948_IMU_H
#define AIRBRAKES_ICM_20948_IMU_H

// Documentation available at: https://cdn.sparkfun.com/assets/7/f/e/c/d/DS-000189-ICM-20948-v1.3.pdf

#include "stm32h7xx_hal.h"

#define ICM_20948_I2C_ADDRESS 0x69

extern uint8_t icm_20948_rx_buffer[12];

typedef enum
{
    IMU_UNINITIALIZED,
    IMU_IDLE, // No work is being done
    IMU_RX_PENDING, // Between the I2C request being made and the callback signaling it is complete
    IMU_DATA_READY // I2C request is complete and data is in *mprls_pressure_rx_buffer
} IMU_STATUS;
extern IMU_STATUS imu_status;

typedef struct
{
    double acceleration_x_meters_per_second_squared;
    double acceleration_y_meters_per_second_squared;
    double acceleration_z_meters_per_second_squared;
    double gyro_x_degrees_per_second;
    double gyro_y_degrees_per_second;
    double gyro_z_degrees_per_second;
} IMU_Data;


/**
 * Initializes the ICM 20948 IMU on I2C2 with SCL & SDA on PB10 & PB11. Polling rate of 1.125 KHz
 *
 * @return Status of initialization
 */
HAL_StatusTypeDef imu_init();

/**
 * Requests data for acceleration and gyro data into icm_20948_rx_buffer
 * @return Status of the read. Does not guarantee the IMU received the request, just that the read is starting
 */
HAL_StatusTypeDef read_acceleration_gyro();

/**
 * Using data in icm_20948_rx_buffer, converts raw data to acceleration and gyroscope data
 * @return An IMU_Data struct containing acceleration and gyroscope data
 */
IMU_Data imu_data_from_rx_buffer();


#endif