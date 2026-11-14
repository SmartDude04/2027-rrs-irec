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
 * NOTE: This function is NON-BLOCKING. This means the function will return before the text is printed to the COM port.
 * If you attempt to print too quickly back-to-back, the second print statement may fail because the first is still
 * printing. To resolve this, add a small delay (~10 ms) after print statements if it is possible they could conflict.
 * 
 * @param format String to send, with standard printf-style format specifiers
 * @param ... Additional arguments matching, in order, the format specifiers in `format`
 * @return USB CDC status, typically either USBD_FAIL, USBD_BUSY, or USBD_OK
 */
uint8_t USB_CDC_PRINTF(const char *format, ...);

#endif