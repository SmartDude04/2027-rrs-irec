#include "main.h"

#include "stm32h7xx_hal.h"

#include "usb_cdc.h"
#include "servo.h"
#include "I2C_DMA.h"
#include "ICM_20948_IMU.h"
#include "MPRLS_PRESSURE.h"

static void run_mprls();
static void run_imu();

static volatile uint8_t new_pressure = 0;
static volatile uint8_t new_imu = 0;
static volatile double pressure_hpa;
static volatile double altitude_m;
static volatile IMU_Data imu_data;

int main(void)
{
    HAL_Init();
    USB_CDC_INIT();
    Servo_Init_TIM3_PA6();
    I2C_DMA_Init();
    mprls_init();
    imu_init();
    
    
    while (1)
    {
        run_mprls();
        run_imu();
        
        if (new_pressure)
        {
            USB_CDC_PRINTF("Pressure (hPa): %lf\tAltitude (m): %lf\n", pressure_hpa, altitude_m);
            new_pressure = 0;
        }
        if (new_imu)
        {
            USB_CDC_PRINTF("ax: %lf ay: %lf ax: %lf gx %lf gy %lf gz %lf\n",
                imu_data.acceleration_x_meters_per_second_squared,
                imu_data.acceleration_y_meters_per_second_squared,
                imu_data.acceleration_z_meters_per_second_squared,
                imu_data.gyro_x_degrees_per_second,
                imu_data.gyro_y_degrees_per_second,
                imu_data.gyro_z_degrees_per_second);
            new_imu = 0;
        }
        
        HAL_Delay(1);
    }
}

void run_mprls()
{
    if (mprls_status == MPRLS_IDLE)
    {
        const HAL_StatusTypeDef ret = request_mprls_read();
        if (ret != HAL_OK)
        {
            HAL_Delay(10);
            USB_CDC_PRINTF("request_mprls_read() failed with code %d\n", ret);
            while (1);
        }
    }
    if (mprls_status == MPRLS_MEASURED)
    {
        const HAL_StatusTypeDef ret = start_mprls_dma_read();
        if (ret != HAL_OK)
        {
            HAL_Delay(10);
            USB_CDC_PRINTF("start_mplrs_dma_read() failed with code %d\n", ret);
            while (1);
        }
    }
    if (mprls_status == MPRLS_DATA_READY)
    {
        pressure_hpa = pressure_psi_to_hpa(pressure_psi_from_rx_buffer());
        altitude_m = pressure_hpa_to_altitude_meters(pressure_hpa);
        new_pressure = 1;
        mprls_status = MPRLS_IDLE;
    }
}

void run_imu()
{
    if (imu_status == IMU_IDLE)
    {
        const HAL_StatusTypeDef ret = read_acceleration_gyro();
        if (ret != HAL_OK)
        {
            USB_CDC_PRINTF("Error reading acceleration/gyro: %d\n", ret);
            while (1);
        }
    }
    if (imu_status == IMU_DATA_READY)
    {
        imu_data = imu_data_from_rx_buffer();
        new_imu = 1;
        imu_status = IMU_IDLE;
    }
}

/**
 * Called when a write is finished from HAL_I2C_Master_Transmit_DMA()
 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Devaddress == MPRLS_PRESSURE_I2C_ADDRESS << 1)
    {
        if (mprls_status == MPRLS_TX_PENDING)
        {
            mprls_status = MPRLS_MEASURING;
        }
    }
}

/**
 * Called when DMA memory is ready from HAL_I2C_Master_Receive_DMA() read
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &i2c1_handle && hi2c->Devaddress == MPRLS_PRESSURE_I2C_ADDRESS << 1)
    {
        if (mprls_status == MPRLS_RX_PENDING)
        {
            mprls_status = MPRLS_DATA_READY;
        }
    }
}

/**
 * Called when DMA memory is ready from HAL_I2C_Mem_Read_DMA() read
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &i2c2_handle && hi2c->Devaddress == ICM_20948_I2C_ADDRESS << 1)
    {
        if (imu_status == IMU_RX_PENDING)
        {
            imu_status = IMU_DATA_READY;
        }
    }
}

/**
 * Called during GPIO interrupt
 */
void HAL_GPIO_EXTI_Callback(const uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_8)
    {
        if (mprls_status == MPRLS_MEASURING)
        {
            mprls_status = MPRLS_MEASURED;
        }
    }
}