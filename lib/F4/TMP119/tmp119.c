#include "tmp119.h"

uint8_t rx[2];

HAL_StatusTypeDef TMP119_StartRead(struct tmp119 *dev) {
    HAL_StatusTypeDef ret = HAL_BUSY;
    if (!dev->busy) {
        ret = HAL_I2C_Mem_Read_DMA(&hi2c1, TMP119_ADDR, TMP119_REG_TEMP, I2C_MEMADD_SIZE_8BIT, rx, 2);
        dev->busy = true;
    }
    return ret;
}

void TMP119_GetTemperature(struct tmp119 *dev) {
    dev->raw = (int16_t)(((uint16_t)rx[0] << 8) | rx[1]);
    dev->temp = (float)dev->raw * 0.0078125f;
    dev->new_data = true;
    dev->busy = false;
}