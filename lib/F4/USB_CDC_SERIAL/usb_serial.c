#include "usb_serial.h"
#include "usb_device.h"
#include "usbd_cdc_if.h" 
#include <string.h>      
#include <stdio.h>
#include <stdarg.h>

void SerialWrite(const char *format, ...)
{
  char buffer[128];
  va_list args;

  if (format == NULL || hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
  {
    return;
  }

  va_start(args, format);
  int length = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  if (length >= 0 && (size_t)length < sizeof(buffer))
  {
    CDC_Transmit_FS((uint8_t *)buffer, (uint16_t)length);
  }
}