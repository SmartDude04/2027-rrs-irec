#include "ICM_20948_IMU.h"

#include "I2C_DMA.h"
#include "usb_cdc.h"

#define REG_BANK_SEL 0x7F
#define PWR_MGMT_1 0x06
#define PWR_MGMT_2 0x07
#define ACCEL_CONFIG_BANK_2 0x14
#define GYRO_CONFIG_BANK_2 0x01
#define ACCEL_XOUT_H 0x2D

uint8_t icm_20948_rx_buffer[12] = {0};
IMU_STATUS imu_status = IMU_UNINITIALIZED;

HAL_StatusTypeDef imu_init()
{
    if (imu_status != IMU_UNINITIALIZED)
    {
        return HAL_ERROR;
    }
    
    USB_CDC_PRINTF("Here 1\n");
    HAL_StatusTypeDef status;
    
    // Switch to bank 0
    status = HAL_I2C_Mem_Write(&i2c2_handle, ICM_20948_I2C_ADDRESS << 1, REG_BANK_SEL, I2C_MEMADD_SIZE_8BIT,(uint8_t[]){0x00}, 1, 1000);
    if (status != HAL_OK) { return status; }
    USB_CDC_PRINTF("Here 2\n");
    
    // Reset the internal registers
    status = HAL_I2C_Mem_Write(&i2c2_handle, ICM_20948_I2C_ADDRESS << 1, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, (uint8_t[]){0b10000000}, 1, 1000);
    if (status != HAL_OK) { return status; }
    USB_CDC_PRINTF("Here 3\n");
    
    // Delay for reset to happen
    HAL_Delay(150);
    
    // Auto select the best available clock source 
    status = HAL_I2C_Mem_Write(&i2c2_handle, ICM_20948_I2C_ADDRESS << 1, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, (uint8_t[]){0x01}, 1, 1000);
    if (status != HAL_OK) { return status; }
    USB_CDC_PRINTF("Here 4\n");
    
    // Ensure gyro and accelerometer are enabled
    status = HAL_I2C_Mem_Write(&i2c2_handle, ICM_20948_I2C_ADDRESS << 1, PWR_MGMT_2, I2C_MEMADD_SIZE_8BIT, (uint8_t[]){0x00}, 1, 1000);
    if (status != HAL_OK) { return status; }
    
    // Move to bank 2
    status = HAL_I2C_Mem_Write(&i2c2_handle, ICM_20948_I2C_ADDRESS << 1, REG_BANK_SEL, I2C_MEMADD_SIZE_8BIT, (uint8_t[]){0b00100000}, 1, 1000);
    if (status != HAL_OK) { return status; }
    
    // Set acceleration limit to plus.minus 8G
    status = HAL_I2C_Mem_Write(&i2c2_handle, ICM_20948_I2C_ADDRESS << 1, ACCEL_CONFIG_BANK_2, I2C_MEMADD_SIZE_8BIT, (uint8_t[]){0b00000100, 0x00}, 2, 1000);
    if (status != HAL_OK) { return status; }
    
    // Set gyro limit to plus.minus 1000 dps
    status = HAL_I2C_Mem_Write(&i2c2_handle, ICM_20948_I2C_ADDRESS << 1, GYRO_CONFIG_BANK_2, I2C_MEMADD_SIZE_8BIT, (uint8_t[]){0b00000100, 0x00}, 2, 1000);
    if (status != HAL_OK) { return status; }
    
    // Switch back to bank 0
    status = HAL_I2C_Mem_Write(&i2c2_handle, ICM_20948_I2C_ADDRESS << 1, REG_BANK_SEL, I2C_MEMADD_SIZE_8BIT, (uint8_t[]){0x00}, 1, 1000);
    
    if (status == HAL_OK)
    {
        imu_status = IMU_IDLE;
    }
    
    return status;
}

HAL_StatusTypeDef read_acceleration_gyro()
{
    i2c2_handle.Devaddress = ICM_20948_I2C_ADDRESS << 1;
    const HAL_StatusTypeDef ret = HAL_I2C_Mem_Read_DMA(&i2c2_handle, ICM_20948_I2C_ADDRESS << 1, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, icm_20948_rx_buffer, sizeof(icm_20948_rx_buffer));
    if (ret == HAL_OK)
    {
        imu_status = IMU_RX_PENDING;
    }
    return ret;
}