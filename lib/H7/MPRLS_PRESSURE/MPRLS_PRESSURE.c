#include "MPRLS_PRESSURE.h"

#include <math.h>
#include <stm32h7xx_hal.h>

#include "I2C_DMA.h"
#include "ICM_20948_IMU.h"

#define STANDARD_TEMPERATURE_KELVIN 288.16f
#define HEIGHT_TEMP_SCALE_FACTOR 0.0065f
#define STANDARD_PRESSURE_PASCALS 101325f
#define STANDARD_DENSITY_KG_M3 1.225f
#define POWER 5.2561f
#define SPECIFIC_GAS_CONSTANT_DRY_AIR 287.05f
#define RATIO_SPECIFIC_HEATS_AIR 1.4f
#define TEMPERATURE_LAPSE_RATE 0.0065f

uint8_t mprls_pressure_rx_buffer[4] = {0};
MPRLS_STATUS mprls_status = MPRLS_UNINITIALIZED;

static uint8_t measurement_command[3] = {0xAA, 0x00, 0x00};

void mprls_init()
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    GPIO_InitTypeDef pb8_init = {0};
    pb8_init.Pin = GPIO_PIN_8;
    pb8_init.Mode = GPIO_MODE_IT_RISING;
    pb8_init.Pull = GPIO_PULLDOWN;
    pb8_init.Speed = GPIO_SPEED_FREQ_MEDIUM;
    
    HAL_GPIO_Init(GPIOB, &pb8_init);   
    
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    
    mprls_status = MPRLS_IDLE;
}

HAL_StatusTypeDef request_mprls_read()
{
    if (mprls_status != MPRLS_IDLE)
    {
        return HAL_ERROR;
    }
    
    i2c1_handle.Devaddress = MPRLS_PRESSURE_I2C_ADDRESS << 1;
    
    const HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit_DMA(&i2c1_handle, MPRLS_PRESSURE_I2C_ADDRESS << 1, measurement_command, sizeof(measurement_command));
    if (ret == HAL_OK)
    {
        mprls_status = MPRLS_TX_PENDING;
    }
    return ret;
}

HAL_StatusTypeDef start_mprls_dma_read()
{
    if (mprls_status != MPRLS_MEASURED)
    {
        return HAL_ERROR;
    }
    
    i2c1_handle.Devaddress = MPRLS_PRESSURE_I2C_ADDRESS << 1;
    
    const HAL_StatusTypeDef ret = HAL_I2C_Master_Receive_DMA(&i2c1_handle, MPRLS_PRESSURE_I2C_ADDRESS << 1, mprls_pressure_rx_buffer, 4);
    if (ret == HAL_OK)
    {
        mprls_status = MPRLS_RX_PENDING;
    }
    return ret;
}

float pressure_psi_from_rx_buffer()
{
    const float output_min = 0x19999Au;
    const float output_max = 0xE66666u;
    const float pressure_min = 0.0f;
    const float pressure_max = 25.0f;
    
    const uint32_t pressure_raw = (mprls_pressure_rx_buffer[1] << 16) + (mprls_pressure_rx_buffer[2] << 8) + mprls_pressure_rx_buffer[3];
    
    return ((float) pressure_raw - output_min) / (output_max - output_min) * (pressure_max - pressure_min) + pressure_min;
}

float pressure_to_hpa(const float pressure_psi)
{
    return pressure_psi * 68.9476f;
}

float altitude_meters(const float pressure_hpa)
{
    // From this formula: https://en.wikipedia.org/wiki/Pressure_altitude
    return 0.3048f * 145366.45f * (1 - powf(pressure_hpa / LAUNCH_PRESSURE_HPA, 0.190284f));
}

// float air_density_kg_per_meter_cubed(const float pressure_hpa)
// {
//     const float pressure_pascals = pressure_hpa * 100.0f;
//     
//     return pressure_pascals / (SPECIFIC_GAS_CONSTANT_DRY_AIR * STANDARD_TEMPERATURE_KELVIN);
// }

float air_density_kg_per_meter_cubed(const float altitude_meters)
{
    return STANDARD_DENSITY_KG_M3 * powf(1.0f - TEMPERATURE_LAPSE_RATE * altitude_meters / STANDARD_TEMPERATURE_KELVIN, 4.2561f);
}

float speed_of_sound_meters_per_second(const float pressure_hpa, const float density_kg_meters_cubed)
{
    const float pressure_pascals = pressure_hpa * 100;
    return sqrtf(RATIO_SPECIFIC_HEATS_AIR * pressure_pascals / density_kg_meters_cubed);
}

void EXTI9_5_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
}