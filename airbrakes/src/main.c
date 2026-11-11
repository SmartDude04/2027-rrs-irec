#include "main.h"

#include "stm32h7xx_hal.h"

#include "usb_cdc.h"
#include "servo.h"
#include "I2C_DMA.h"
#include "../lib/MPRLS_PRESSURE/MPRLS_PRESSURE.h"

void run_mprls();

int main(void)
{
    HAL_Init();
    USB_CDC_INIT();
    Servo_Init_TIM3_PA6();
    I2C1_DMA_Init();
    mprls_init();
    
    HAL_Delay(3000);
    
    while (1)
    {
        run_mprls();
    }
}

void run_mprls()
{
    if (mprls_status == IDLE)
    {
        const HAL_StatusTypeDef ret = request_mprls_read();
        if (ret != HAL_OK)
        {
            HAL_Delay(10);
            USB_CDC_PRINTF("request_mprls_read() failed with code %d", ret);
            while (1);
        }
    }
    if (mprls_status == MEASURED)
    {
        const HAL_StatusTypeDef ret = start_mprls_dma_read();
        if (ret != HAL_OK)
        {
            HAL_Delay(10);
            USB_CDC_PRINTF("start_mplrs_dma_read() failed with code %d", ret);
            while (1);
        }
    }
    if (mprls_status == DATA_READY)
    {
        const uint32_t pressure = (mprls_pressure_rx_buffer[1] << 16) + (mprls_pressure_rx_buffer[2] << 8) + mprls_pressure_rx_buffer[3];
        USB_CDC_PRINTF("Pressure: %d\n", pressure);
        mprls_status = IDLE;
    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Devaddress == MPRLS_PRESSURE_I2C_ADDRESS << 1)
    {
        if (mprls_status == TX_PENDING)
        {
            mprls_status = MEASURING;
        }
    }
}

/**
 * Called when DMA memory is ready from HAL_I2C_Master_Receive_DMA() read
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Devaddress == MPRLS_PRESSURE_I2C_ADDRESS << 1)
    {
        if (mprls_status == RX_PENDING)
        {
            mprls_status = DATA_READY;
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
        if (mprls_status == MEASURING)
        {
            mprls_status = MEASURED;
        }
    }
}