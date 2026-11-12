#include "main.h"

#include "stm32h7xx_hal.h"

#include "usb_cdc.h"
#include "servo.h"
#include "I2C_DMA.h"
#include "../lib/ICM_20948_IMU/ICM_20948_IMU.h"
#include "../lib/MPRLS_PRESSURE/MPRLS_PRESSURE.h"

static void run_mprls();
static void run_imu();

int main(void)
{
    HAL_Init();
    USB_CDC_INIT();
    Servo_Init_TIM3_PA6();
    I2C_DMA_Init();
    mprls_init();
    imu_init();
    
    
    while (1)
    {
        run_mprls();
        run_imu();
        
        HAL_Delay(1);
    }
}

void run_mprls()
{
    if (mprls_status == MPRLS_IDLE)
    {
        const HAL_StatusTypeDef ret = request_mprls_read();
        if (ret != HAL_OK)
        {
            HAL_Delay(10);
            USB_CDC_PRINTF("request_mprls_read() failed with code %d\n", ret);
            while (1);
        }
    }
    if (mprls_status == MPRLS_MEASURED)
    {
        const HAL_StatusTypeDef ret = start_mprls_dma_read();
        if (ret != HAL_OK)
        {
            HAL_Delay(10);
            USB_CDC_PRINTF("start_mplrs_dma_read() failed with code %d\n", ret);
            while (1);
        }
    }
    if (mprls_status == MPRLS_DATA_READY)
    {
        const uint32_t pressure = (mprls_pressure_rx_buffer[1] << 16) + (mprls_pressure_rx_buffer[2] << 8) + mprls_pressure_rx_buffer[3];
        // USB_CDC_PRINTF("Pressure: %d\n", pressure);
        mprls_status = MPRLS_IDLE;
    }
}

void run_imu()
{
    if (imu_status == IMU_IDLE)
    {
        const HAL_StatusTypeDef ret = read_acceleration_gyro();
        if (ret != HAL_OK)
        {
            USB_CDC_PRINTF("Error reading acceleration/gyro: %d\n", ret);
            while (1);
        }
    }
    if (imu_status == IMU_DATA_READY)
    {
        const int16_t ax_raw = (int16_t) (((uint16_t) icm_20948_rx_buffer[0] << 8) + icm_20948_rx_buffer[1]);
        const float ax = (float) ax_raw / 4096.0f;
        USB_CDC_PRINTF("ax: %f\n", ax);
        imu_status = IMU_IDLE;
    }
}

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