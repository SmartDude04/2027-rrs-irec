#include "MPRLS_PRESSURE.h"

#include <math.h>
#include <stm32h7xx_hal.h>

double mprls_pressure_hpa = -1.0;
double mprls_altitude_meters = -1.0;
uint8_t mprls_pressure_rx_buffer[4] = {0};
MPRLS_STATUS mprls_status = UNINITIALIZED;

extern I2C_HandleTypeDef i2c1_handle;

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
    
    mprls_status = IDLE;
}

HAL_StatusTypeDef request_mprls_read()
{
    if (mprls_status != IDLE)
    {
        return HAL_ERROR;
    }
    
    i2c1_handle.Devaddress = MPRLS_PRESSURE_I2C_ADDRESS << 1;
    
    const HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit_DMA(&i2c1_handle, MPRLS_PRESSURE_I2C_ADDRESS << 1, measurement_command, sizeof(measurement_command));
    if (ret == HAL_OK)
    {
        mprls_status = TX_PENDING;
    }
    return ret;
}

HAL_StatusTypeDef start_mprls_dma_read()
{
    if (mprls_status != MEASURED)
    {
        return HAL_ERROR;
    }
    
    i2c1_handle.Devaddress = MPRLS_PRESSURE_I2C_ADDRESS << 1;
    
    const HAL_StatusTypeDef ret = HAL_I2C_Master_Receive_DMA(&i2c1_handle, MPRLS_PRESSURE_I2C_ADDRESS << 1, mprls_pressure_rx_buffer, 4);
    if (ret == HAL_OK)
    {
        mprls_status = RX_PENDING;
    }
    return ret;
}

double pressure_hpa_to_altitude_meters(const double pressure_hpa)
{
    return 0.3048 * 145366.45 * (1 - pow(pressure_hpa / 1013.25, 0.190284)) - LAUNCH_SITE_ALTITUDE_ABOVE_SEA_LEVEL;
}

void EXTI9_5_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
}