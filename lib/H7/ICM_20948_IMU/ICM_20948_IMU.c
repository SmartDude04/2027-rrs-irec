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

static TIM_HandleTypeDef tim4_handle;
HAL_StatusTypeDef tim4_init();

HAL_StatusTypeDef imu_init()
{
    if (imu_status != IMU_UNINITIALIZED)
    {
        return HAL_ERROR;
    }
    
    HAL_StatusTypeDef status;
    
    // Switch to bank 0
    status = HAL_I2C_Mem_Write(&i2c2_handle, ICM_20948_I2C_ADDRESS << 1, REG_BANK_SEL, I2C_MEMADD_SIZE_8BIT,(uint8_t[]){0x00}, 1, 1000);
    if (status != HAL_OK) { return status; }
    
    // Reset the internal registers
    status = HAL_I2C_Mem_Write(&i2c2_handle, ICM_20948_I2C_ADDRESS << 1, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, (uint8_t[]){0b10000000}, 1, 1000);
    if (status != HAL_OK) { return status; }
    
    // Delay for reset to happen
    HAL_Delay(150);
    
    // Auto select the best available clock source 
    status = HAL_I2C_Mem_Write(&i2c2_handle, ICM_20948_I2C_ADDRESS << 1, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, (uint8_t[]){0x01}, 1, 1000);
    if (status != HAL_OK) { return status; }
    
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
    if (status != HAL_OK) { return status; }
    
    status = tim4_init();
    if (status == HAL_OK)
    {
        imu_status = IMU_IDLE;
    }
    
    return status;
}

HAL_StatusTypeDef tim4_init()
{
    __HAL_RCC_TIM4_CLK_ENABLE();
    
    tim4_handle.Instance = TIM4;
    tim4_handle.Init.Prescaler = 63; // Sets the timer to have a realized frequency of 1 MHz
    tim4_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    tim4_handle.Init.Period = 9999; // 100 Hz
    tim4_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    tim4_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    
    const HAL_StatusTypeDef status = HAL_TIM_Base_Init(&tim4_handle);
    if (status != HAL_OK)
    {
        return status;
    }
    
    HAL_NVIC_SetPriority(TIM4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
    
    // Initialization can leave an update flag pending
    __HAL_TIM_CLEAR_FLAG(&tim4_handle, TIM_FLAG_UPDATE);
    return HAL_TIM_Base_Start_IT(&tim4_handle);
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

IMU_Data imu_data_from_rx_buffer()
{
    IMU_Data data;
    
    const int16_t raw_accel_x = (int16_t)(((uint16_t) icm_20948_rx_buffer[0] << 8) + icm_20948_rx_buffer[1]);
    const int16_t raw_accel_y = (int16_t)(((uint16_t) icm_20948_rx_buffer[2] << 8) + icm_20948_rx_buffer[3]);
    const int16_t raw_accel_z = (int16_t)(((uint16_t) icm_20948_rx_buffer[4] << 8) + icm_20948_rx_buffer[5]);
    const int16_t raw_gyro_x = (int16_t)(((uint16_t) icm_20948_rx_buffer[6] << 8) + icm_20948_rx_buffer[7]);
    const int16_t raw_gyro_y = (int16_t)(((uint16_t) icm_20948_rx_buffer[8] << 8) + icm_20948_rx_buffer[9]);
    const int16_t raw_gyro_z = (int16_t)(((uint16_t) icm_20948_rx_buffer[10] << 8) + icm_20948_rx_buffer[11]);
    
    // Raw is converted using conversion from sections 3.1 and 3.2
    // https://cdn.sparkfun.com/assets/7/f/e/c/d/DS-000189-ICM-20948-v1.3.pdf
    data.acceleration_x_meters_per_second_squared = 9.80665f * ((float) raw_accel_x) / 4096.0f;
    data.acceleration_y_meters_per_second_squared = 9.80665f * ((float) raw_accel_y) / 4096.0f;
    data.acceleration_z_meters_per_second_squared = 9.80665f * ((float) raw_accel_z) / 4096.0f;
    data.gyro_x_degrees_per_second = ((float) raw_gyro_x) / 32.8f;
    data.gyro_y_degrees_per_second = ((float) raw_gyro_y) / 32.8f;
    data.gyro_z_degrees_per_second = ((float) raw_gyro_z) / 32.8f;
    
    return data;
}

void TIM4_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&tim4_handle);
}