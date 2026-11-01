#include "ICM.h"

uint8_t acc[6];

uint8_t value = 0;
uint16_t AccelSensitivity = 16384; // 2g range by default, 4g = 8192, 8g = 4096, 16g = 2048
HAL_StatusTypeDef ICM_StartAccel(struct ICM *dev) {
    HAL_StatusTypeDef ret = HAL_OK;
    if (!dev->busy) {
       ret = HAL_I2C_Mem_Read_DMA(&hi2c1, ICM_ADDR, ICM_accelXH,I2C_MEMADD_SIZE_8BIT, (uint8_t *)acc, 6);
        
        dev->busy = true;
    }
    return ret;
}

void ICM_GetAccel(struct ICM *dev) {
    dev->rawX = (int16_t)(((uint16_t)acc[0] << 8) | acc[1]);
    dev->rawY = (int16_t)(((uint16_t)acc[2] << 8) | acc[3]);
    dev->rawZ = (int16_t)(((uint16_t)acc[4] << 8) | acc[5]);
    dev->accelX = (float)dev->rawX / AccelSensitivity - dev->offX;
dev->accelY = (float)dev->rawY / AccelSensitivity - dev->offY;
dev->accelZ = (float)dev->rawZ / AccelSensitivity - dev->offZ;
    dev->new_data = true;
    dev->busy = false;
}

void ICM_init(struct ICM *dev) {
    dev->busy = false;
    dev->new_data = false;
    dev->busy = true;
    HAL_I2C_Mem_Write(&hi2c1, ICM_ADDR, ICM_BANK_CONFIG, I2C_MEMADD_SIZE_8BIT,(uint8_t[]){0x00}, 1, 1000); // ensure on bank 0
    HAL_I2C_Mem_Write(&hi2c1, ICM_ADDR, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, (uint8_t[]){0x80}, 1, 1000); // reset device before starting everything
    HAL_Delay(150); // delay for reset to happen
    HAL_I2C_Mem_Write(&hi2c1, ICM_ADDR, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, (uint8_t[]){0x01}, 1, 1000);
    HAL_I2C_Mem_Write(&hi2c1, ICM_ADDR, PWR_MGMT_2, I2C_MEMADD_SIZE_8BIT, (uint8_t[]){0x00}, 1, 1000);
    HAL_I2C_Mem_Write(&hi2c1, ICM_ADDR, ICM_BANK_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t[]){0x20}, 1, 1000); // direct to bank 2 adr 0x20
    HAL_I2C_Mem_Write(&hi2c1, ICM_ADDR, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t[]){0x00}, 1, 1000);
    HAL_I2C_Mem_Write(&hi2c1, ICM_ADDR, ICM_BANK_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t[]){0x00}, 1, 1000); // direct back to bank 0 
    dev->busy = false; 
    //calibration code
    int samples =10000;
    float sumX = 0;
    float sumY = 0;
    float sumZ = 0;
    for (int i = 0; i < samples; i++) {
        ICM_StartAccel(dev);
        while (dev->busy) {
        }
        sumX += dev->accelX;
        sumY += dev->accelY;
        sumZ += dev->accelZ;

    }
    float accXOffset =sumX / samples;
    float accYOffset = sumY / samples;
    float accZOffset = sumZ / samples- 1.0f;
    SerialWrite("Calibration complete. Offsets: X=%f, Y=%f, Z=%f\n", accXOffset, accYOffset, accZOffset);
    dev->busy = false;
}