#include "main.h"

#include "stm32h7xx_hal.h"

#include "usb_cdc.h"

int main(void)
{
  HAL_Init();
  USB_CDC_INIT();
  
  int i = 0;
  
  while (1)
  {
    USB_CDC_PRINTF("Hello, world %d!\n", ++i);
    HAL_Delay(1000);
  }
}