#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#include "stm32h7xx_hal.h"

#include "usb_cdc.h"
#include "usbd_cdc.h"
#include "usbd_def.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"

static void SystemClock_Config(void);
static void MPU_Config(void);
static void USB_CDC_Error_Handler(void);

int initialized = 0;

void USB_CDC_INIT(void)
{
    MPU_Config();
    HAL_Init();
    SystemClock_Config();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    MX_USB_DEVICE_Init();
    
    initialized = 1;
}

uint8_t USB_CDC_PRINTF(const char *format, ...)
{
    if (!initialized)
    {
        return USBD_FAIL;
    }
    
    static char buffer[256];
    
    // Get the current CDC state
    extern USBD_HandleTypeDef hUsbDeviceFS;
    const USBD_CDC_HandleTypeDef *cdc = (USBD_CDC_HandleTypeDef *) hUsbDeviceFS.pClassData;

    // If USB is not ready to send, return failure
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED || cdc == NULL) {
        return USBD_FAIL;
    }

    // Further, if a previous transmission is still printing, fail
    if (cdc->TxState != 0) {
        return USBD_BUSY;
    }

    // Access the additional arguments declared in the ...'s
    va_list args;
    va_start(args, format);
    const int length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    // Check length of string including args is small enough to fit in buff
    if (length < 0 || (size_t) length >= sizeof(buffer)) {
        return USBD_FAIL;
    }

    // If there is nothing to send, then just don't send anything
    if (length == 0) {
        return USBD_OK;
    }

    // Finally, transmit the message
    return CDC_Transmit_FS((uint8_t *) buffer, (uint16_t) length);
}

void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct;

    /* Disables the MPU */
    HAL_MPU_Disable();

    /** Initializes and configures the Region and the memory to be protected
    */
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress = 0x0;
    MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
    MPU_InitStruct.SubRegionDisable = 0x87;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    /* Enables the MPU */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /** Supply configuration update enable
    */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    /** Configure the main internal regulator output voltage
    */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        USB_CDC_Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                                |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        USB_CDC_Error_Handler();
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
static void USB_CDC_Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    
    }
}
