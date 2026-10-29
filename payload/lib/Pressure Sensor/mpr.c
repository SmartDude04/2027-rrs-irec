#include "mpr.h"

MPR_Result MPR_Read(MPR_Data *result)
{
    
    const uint8_t command[] = {0xAA, 0x00, 0x00}; //sends the command to start reading pressure
    if (HAL_I2C_Master_Transmit(&hi2c1, MPR_I2C_ADDR, (uint8_t *)command, sizeof(command), MPR_TIMEOUT_MS) != HAL_OK) {;
        return MPR_I2C_ERROR;
    }

    uint32_t start = HAL_GetTick(); //starting "time"
    uint8_t status = 0;

    do {
        if (HAL_I2C_Master_Receive(&hi2c1, MPR_I2C_ADDR, &status, 1, MPR_TIMEOUT_MS) != HAL_OK) {
            return MPR_I2C_ERROR;
        }

        if ((status & (1u << 2))) {;
            return MPR_SENSOR_ERROR;
        }

        if ((status & (1u << 6)) != 0u) {
            return MPR_POWER_ERROR;
        }

        HAL_Delay(1);
    } while ((HAL_GetTick() - start) < MPR_TIMEOUT_MS); //runs until timeout

    if (status & (1u << 5)) {
        result->busy = true;
        return;
    }

    uint8_t bytes[4];
    if (HAL_I2C_Master_Receive(&hi2c1, MPR_I2C_ADDR, bytes, 4,
                               MPR_TIMEOUT_MS) != HAL_OK) {
        return MPR_I2C_ERROR;
    }

    result->status = bytes[0];
    result->raw_count = ((uint32_t)bytes[1] << 16)
                      | ((uint32_t)bytes[2] << 8)
                      |  (uint32_t)bytes[3];

    if ((result->status & (1u << 5)) != 0u) {
        return MPR_BUSY;
    }
    if ((result->status & (1u << 2)) != 0u || (result->status & 1u) != 0u) {
        return MPR_SENSOR_ERROR;
    }

    result->pressure =
        ((float)(result->raw_count - MPR_OUTPUT_MIN)
        / (float)(MPR_OUTPUT_MAX - MPR_OUTPUT_MIN))
        * (MPR_PRESSURE_MAX - MPR_PRESSURE_MIN)
        + MPR_PRESSURE_MIN;

    return MPR_OK;
}