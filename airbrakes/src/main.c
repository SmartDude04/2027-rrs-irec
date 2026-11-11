#include "main.h"

#include "stm32h7xx_hal.h"

#include "usb_cdc.h"
#include "servo.h"
#include "I2C_DMA.h"
#include "../lib/MPRLS_PRESSURE/MPRLS_PRESSURE.h"

static volatile int call_read_pressure = 1;
static volatile int request_pressure = 0;
static volatile int data_ready = 0;

int main(void)
{
    HAL_Init();
    USB_CDC_INIT();
    Servo_Init_TIM3_PA6();
    I2C1_DMA_Init();
    mprls_init();
    
    while (1)
    {
        USB_CDC_PRINTF("Starting request\n");
        HAL_Delay(10);
        const HAL_StatusTypeDef res = start_mprls_dma_read();
        if (res != HAL_OK)
        {
            USB_CDC_PRINTF("Error\n");
            while (1);
        }
        
        if (data_ready)
        {
            USB_CDC_PRINTF("Data ready\n");
            data_ready = 0;
        }
        
        HAL_Delay(1000);
    }
}

/**
 * Called when DMA memory is ready from HAL_I2C_Master_Receive_DMA() read
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Devaddress == MPRLS_PRESSURE_I2C_ADDRESS << 1)
    {
        data_ready = 1;
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    
}


// /**
//  * Called during GPIO interrupt
//  */
// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
// {
//     if (GPIO_Pin == GPIO_PIN_8)
//     {
//         request_pressure = 1;
//     }
// }