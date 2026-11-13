#ifndef AIRBRAKES_MPRLS_PRESSURE_H
#define AIRBRAKES_MPRLS_PRESSURE_H

#include <stdint.h>

#include "stm32h7xx_hal.h"

#define MPRLS_PRESSURE_I2C_ADDRESS 0x18
#define LAUNCH_PRESSURE_HPA 1013.25f

typedef enum
{
    MPRLS_UNINITIALIZED,
    MPRLS_IDLE, // No work is being done
    MPRLS_TX_PENDING, // Between starting the I2C command to make a measurement and the callback signaling the I2C command is complete
    MPRLS_MEASURING, // Between the I2C request for a measurement being sent and PB8 going high 
    MPRLS_MEASURED, // Between PB8 going high and the I2C request being made
    MPRLS_RX_PENDING, // Between the I2C request being made and the callback signaling it is complete
    MPRLS_DATA_READY // I2C request is complete and data is in *mprls_pressure_rx_buffer
} MPRLS_STATUS;

extern MPRLS_STATUS mprls_status;

extern uint8_t mprls_pressure_rx_buffer[4];

/**
 * Initializes the MPRLS pressure sensor. Specifically, sets PB8 as input to be called when data is ready
 */
void mprls_init();

/**
 * Requests the MPRLS pressure sensor to make an environmental measurement. Between calling this function and reading
 * the data, 5 ms must pass *or* the EOC is set to high
 * 
 * @return Status of the request. Does not guarantee that the microcontroller received the data, only that a request
 * was sent
 */
HAL_StatusTypeDef request_mprls_read();

/**
 * Starts a read from the DMA. The HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) interrupt will be triggered
 * when the memory is ready to be accessed
 * 
 * @return Status of the I2C read. Does not guarantee data will appear in the interrupt, only that a read request was
 * successful
 */
HAL_StatusTypeDef start_mprls_dma_read();

/**
 * Using data in mprls_pressure_rx_buffer, returns the pressure in PSI
 * @return The pressure, in PSI
 */
float pressure_psi_from_rx_buffer();

/**
 * Converts the pressure in PSI to hPa
 */
float pressure_to_hpa(float pressure_psi);

/**
 * Converts a pressure reading to a relative altitude above the ground
 * 
 * @param pressure_hpa Height, in meters, of the launch site above sea level
 * @return Altitude of the rocket above the launch site, in meters
 */
float altitude_meters(float pressure_hpa);

/**
 * Converts a pressure and temperature reading to an estimated air density
 * 
 * @param pressure_hpa Pressure, in hPa, of the air
 * @return Estimated air density in kilograms per meter cubed
 */
float air_density_kg_per_meter_cubed(float pressure_hpa);

/**
 * Converts a pressure and air density reading to a speed of sound estimated
 * 
 * @param pressure_hpa Pressure of the air, in hectopascals
 * @param density_kg_meters_cubed Air density in kilograms per meter cubed
 * @return The current estimated speed of sound
 */
float speed_of_sound_meters_per_second(float pressure_hpa, float density_kg_meters_cubed);


#endif