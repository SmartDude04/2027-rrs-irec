#include "main.h"

#include "ICM_20948_IMU.h"
#include "MPRLS_PRESSURE.h"
#include "I2C_DMA.h"

volatile float pressure_hpa;
volatile float altitude_meters;
volatile float air_density_kg_m_cubed;
volatile IMU_Data imu_data;

/**
 * Called when a write is finished from HAL_I2C_Master_Transmit_DMA()
 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Devaddress == MPRLS_PRESSURE_I2C_ADDRESS << 1)
    {
        if (mprls_status == MPRLS_TX_PENDING)
        {
            mprls_status = MPRLS_MEASURING;
        }
    }
}

/**
 * Called when DMA memory is ready from HAL_I2C_Master_Receive_DMA() read
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &i2c1_handle && hi2c->Devaddress == MPRLS_PRESSURE_I2C_ADDRESS << 1)
    {
        if (mprls_status == MPRLS_RX_PENDING)
        {
            mprls_status = MPRLS_DATA_READY;
        }
    }
}

/**
 * Called when DMA memory is ready from HAL_I2C_Mem_Read_DMA() read
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &i2c2_handle && hi2c->Devaddress == ICM_20948_I2C_ADDRESS << 1)
    {
        if (imu_status == IMU_RX_PENDING)
        {
            imu_status = IMU_DATA_READY;
        }
    }
}

/**
 * Called during GPIO interrupt
 */
void HAL_GPIO_EXTI_Callback(const uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_8)
    {
        if (mprls_status == MPRLS_MEASURING)
        {
            mprls_status = MPRLS_MEASURED;
        }
    }
}

/**
 * Called when a timer interrupt is set to fire
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
    {
        if (imu_status == IMU_DATA_READY)
        {
            imu_status = IMU_IDLE;
        }
    }
}