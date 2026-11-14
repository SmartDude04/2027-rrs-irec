#include "ICM.h"

uint8_t acc[6];
uint8_t gyro[6];
uint8_t value = 0;
uint16_t accelSensitivity = 16384; // 2g range by default, 4g = 8192, 8g = 4096, 16g = 2048
uint16_t gyroSensitivity = 131; // 250dps range by default, 500dps = 65.5, 1000dps = 32.8, 2000dps = 16.4

/**
 * @brief Start reading the Acceleration data direct from memory using DMA
 * 
 * @param dev Device Struct
 * @return HAL_StatusTypeDef Returns if the I2C read was successful or not. HAL_OK if successful, HAL_ERROR if not.
 */
HAL_StatusTypeDef ICM_StartAccel(struct ICM *dev) {
    HAL_StatusTypeDef ret = HAL_OK;
    if (!dev->aBusy) {
       ret = HAL_I2C_Mem_Read_DMA(&hi2c1, ICM_ADDR, ICM_accelXH,I2C_MEMADD_SIZE_8BIT, (uint8_t *)acc, 6);
       
        dev->aBusy = true;
    }
    return ret;
}
/**
 * @brief Start reading the Gyro data direct from memory using DMA
 * 
 * @param dev Device Struct
 * @return HAL_StatusTypeDef  Returns if the I2C read was successful or not. HAL_OK if successful, HAL_ERROR if not.
 */
HAL_StatusTypeDef ICM_StartGyro(struct ICM *dev) {
    HAL_StatusTypeDef ret = HAL_OK;
    if (!dev->gBusy) {
        ret = HAL_I2C_Mem_Read_DMA(&hi2c1, ICM_ADDR, ICM_gyroXH,I2C_MEMADD_SIZE_8BIT, (uint8_t *)gyro, 6);
       
        dev->gBusy = true;
    }
    return ret;
}
/**
 * @brief Get the Acceleration objects in X,Y, Z, and updates dev struct with new data.
 * 
 * @param dev Device Struct
 */
void ICM_GetAccel(struct ICM *dev) {
    dev->rawX = (int16_t)(((uint16_t)acc[0] << 8) | acc[1]);
    dev->rawY = (int16_t)(((uint16_t)acc[2] << 8) | acc[3]);
    dev->rawZ = (int16_t)(((uint16_t)acc[4] << 8) | acc[5]);
    dev->accelX = (float)dev->rawX / accelSensitivity;
    dev->accelY = (float)dev->rawY / accelSensitivity;
    dev->accelZ = (float)dev->rawZ / accelSensitivity;
    dev->new_accel_data = true;

    dev->aBusy = false;
}
/**
 * @brief Get the Gyro objects in X,Y, Z, and updates dev struct with new data.
 * 
 * @param dev Device Struct
 * @param degrees Toggle if output is in degrees (true) or radians (false)
 */
void ICM_GetGyro(struct ICM *dev, bool degrees) {
    float conversion= 3.14159/180;
    dev->rawGyroX = (int16_t)(((uint16_t)gyro[0] << 8) | gyro[1]);
    dev->rawGyroY = (int16_t)(((uint16_t)gyro[2] << 8) | gyro[3]);
    dev->rawGyroZ = (int16_t)(((uint16_t)gyro[4] << 8) | gyro[5]);
    if(degrees){
        conversion = 1;
    }
    dev->gyroX = ((float)dev->rawGyroX / gyroSensitivity)*conversion; 
    dev->gyroY = ((float)dev->rawGyroY / gyroSensitivity)*conversion;
    dev->gyroZ = ((float)dev->rawGyroZ / gyroSensitivity)*conversion;
    dev->new_gyro_data = true;
   
    dev->gBusy = false;
}

/** 
 * @brief Initializes the ICM device and configures registers
 */
void ICM_init(struct ICM *dev) {
    dev->aBusy = false;
    dev->gBusy = false;
    dev->new_accel_data = false;
    dev->new_gyro_data = false;
    HAL_I2C_Mem_Write(&hi2c1, ICM_ADDR, ICM_BANK_CONFIG, I2C_MEMADD_SIZE_8BIT,(uint8_t[]){0x00}, 1, 1000); // ensure on bank 0
    HAL_I2C_Mem_Write(&hi2c1, ICM_ADDR, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, (uint8_t[]){0x80}, 1, 1000); // reset device before starting everything
    HAL_Delay(150); // delay for reset to happen
    HAL_I2C_Mem_Write(&hi2c1, ICM_ADDR, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, (uint8_t[]){0x01}, 1, 1000);
    HAL_I2C_Mem_Write(&hi2c1, ICM_ADDR, PWR_MGMT_2, I2C_MEMADD_SIZE_8BIT, (uint8_t[]){0x00}, 1, 1000);
    HAL_I2C_Mem_Write(&hi2c1, ICM_ADDR, ICM_BANK_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t[]){0x20}, 1, 1000); // direct to bank 2 adr 0x20
    HAL_I2C_Mem_Write(&hi2c1, ICM_ADDR, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t[]){0x00}, 1, 1000);
    HAL_I2C_Mem_Write(&hi2c1, ICM_ADDR, GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t[]){0x00}, 1, 1000);
    HAL_I2C_Mem_Write(&hi2c1, ICM_ADDR, ICM_BANK_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t[]){0x00}, 1, 1000); // direct back to bank 0 

    
}