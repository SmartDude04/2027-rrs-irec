#include "mpr.h"

uint8_t rx[4];

HAL_StatusTypeDef MPR_StartRead(struct mpr *dev) {
    HAL_StatusTypeDef ret = HAL_BUSY;
    if (!dev->busy) {
        ret = HAL_I2C_Mem_Read_DMA(&hi2c1, TMP119_ADDR, TMP119_REG_TEMP, I2C_MEMADD_SIZE_8BIT, rx, 2);
        dev->busy = true;
    }
    return ret;
}