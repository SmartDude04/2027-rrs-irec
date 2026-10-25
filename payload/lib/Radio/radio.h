#ifndef __RADIO_H__
#define __RADIO_H__

#include "lora_sx1276.h"
#include "SPI.h"

enum state {
    IDLE,
    ARMED,
    ASCENT,
    DESCENT
};

void radio_config(lora_sx1276 *lora);
void radio_test(lora_sx1276 *lora);


#endif