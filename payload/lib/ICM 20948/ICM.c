#include "ICM.h"

uint8_t rx[2];
uint8_t value = 0;

HAL_StatusTypeDef ICM_StartAccel(struct ICM *dev) {
    HAL_StatusTypeDef ret = HAL_OK;
    if (!dev->busy) {
        ret = HAL_I2C_Mem_Read_DMA(&hi2c1, ICM_ADDR, ICM_accelXH, I2C_MEMADD_SIZE_8BIT, rx, 1);
        dev->busy = true;
    }
    return ret;
}

void ICM_GetAccel(struct ICM *dev) {
    dev->rawX = (int16_t)(((uint16_t)rx[0] << 8) | rx[1]);
    dev->accelX = (float)dev->rawX / 16384.0f;
    //SerialWrite("Accel X: %f\tRawAccel: %d\n", dev->accelX, dev->rawX);
    //dev->accelY = (int16_t)(((uint16_t)rx[2] << 8) | rx[3]);
    //dev->accelZ = (int16_t)(((uint16_t)rx[4] << 8) | rx[5]);
    dev->new_data = true;
    dev->busy = false;
}

void ICM_init(struct ICM *dev) {
    dev->busy = false;
    dev->new_data = false;
    HAL_I2C_Mem_Write(&hi2c1, ICM_ADDR, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, 0x01, 1, 1000);
    HAL_I2C_Mem_Write(&hi2c1, ICM_ADDR, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, 0x00, 1, 1000);
    HAL_I2C_Mem_Write(&hi2c1, ICM_ADDR, PWR_MGMT_2, I2C_MEMADD_SIZE_8BIT, 0x00, 1, 1000);

}