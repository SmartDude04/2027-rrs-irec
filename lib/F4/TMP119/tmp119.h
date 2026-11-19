#ifndef __TMP119_H__
#define __TMP119_H__

#include "I2C_DMA.h"
#include "stdbool.h"
#include "stm32f4xx_hal.h"

#define TMP119_ADDR1     0x48<<1
#define TMP119_ADDR2     0x49<<1
#define TMP119_REG_TEMP 0x00
#define TMP119_REG_ID   0x0f
#define TMP119_REG_CFG  0x01

struct tmp119 {
    volatile bool busy;
    volatile bool new_data;
    volatile int16_t raw;
    float temp;
    uint8_t i2c_addr;   
};

HAL_StatusTypeDef TMP119_StartRead(struct tmp119 *dev);
void TMP119_GetTemperature(struct tmp119 *dev);
void TMP119_SetLowPower(struct tmp119 *dev,bool lowPower);
void TMP119_setAddr(struct tmp119 *dev,bool addr2);


#endif