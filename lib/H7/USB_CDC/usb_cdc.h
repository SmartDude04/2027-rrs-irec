#ifndef AIRBRAKES_USB_CDC_H
#define AIRBRAKES_USB_CDC_H

#include <stdint.h>

/**
 * Initializes USB CDC communication to open a virtual COM port with a connected device
 */
void USB_CDC_INIT();

/**
 * Prints a formatted string to the USB CDC COM port opened using USB_CDC_INIT
 * 
 * @param format String to send, with standard printf-style format specifiers
 * @param ... Additional arguments matching, in order, the format specifiers in `format`
 * @return USB CDC status, typically either USBD_FAIL, USBD_BUSY, or USBD_OK
 */
uint8_t USB_CDC_PRINTF(const char *format, ...);

#endif