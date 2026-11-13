#include "main.h"

#include "stm32h7xx_hal.h"

#include "usb_cdc.h"
#include "servo.h"
#include "I2C_DMA.h"
#include "ICM_20948_IMU.h"
#include "MPRLS_PRESSURE.h"
#include "finite_state_machine.h"

static void run_mprls();
static void run_imu();

volatile float cur_pressure_hpa;
volatile float cur_altitude_meters;
volatile float cur_air_density_kg_m_cubed;
volatile float cur_speed_of_sound_meters_per_second;
volatile IMU_Data cur_imu_data;

static volatile uint8_t new_pressure = 0;
static volatile uint8_t new_imu = 0;

Airbrakes_Finite_State_Machine airbrakes_state = DISARMED;

int main(void)
{
    // TODO: Add better error handling all around, and especially in init functions
    HAL_Init();
    USB_CDC_INIT();
    Servo_Init_TIM3_PA6();
    I2C_DMA_Init();
    mprls_init();
    const HAL_StatusTypeDef imu_init_status = imu_init();
    if (imu_init_status != HAL_OK)
    {
        HAL_Delay(3000);
        USB_CDC_PRINTF("Failed to initialize IMU: Failed with code %d\n", imu_init_status);
        while (1);
    }
    
    
    while (1)
    {
        // Servo_Move_Degrees(0);
        // HAL_Delay(1000);
        // Servo_Move_Degrees(30);
        // HAL_Delay(1000);
        // continue;
        run_mprls();
        run_imu();
        
        if (new_pressure)
        {
            HAL_Delay(10);
            USB_CDC_PRINTF("Pressure (hPa): %lf\tAltitude (m): %lf\t Air density (kg/m^3): %lf\t Speed of sound (m/s): %lf\n", cur_pressure_hpa, cur_altitude_meters, cur_air_density_kg_m_cubed, cur_speed_of_sound_meters_per_second);
            new_pressure = 0;
        }
        if (new_imu)
        {
            HAL_Delay(10);
            USB_CDC_PRINTF("ax: %lf ay: %lf ax: %lf gx %lf gy %lf gz %lf temp: %lf\n",
                cur_imu_data.acceleration_x_meters_per_second_squared,
                cur_imu_data.acceleration_y_meters_per_second_squared,
                cur_imu_data.acceleration_z_meters_per_second_squared,
                cur_imu_data.gyro_x_degrees_per_second,
                cur_imu_data.gyro_y_degrees_per_second,
                cur_imu_data.gyro_z_degrees_per_second,
                cur_imu_data.temperature_celsius);
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
        cur_pressure_hpa = pressure_to_hpa(pressure_psi_from_rx_buffer());
        cur_altitude_meters = altitude_meters(cur_pressure_hpa);
        cur_air_density_kg_m_cubed = air_density_kg_per_meter_cubed(cur_pressure_hpa, cur_imu_data.temperature_celsius);
        cur_speed_of_sound_meters_per_second = speed_of_sound_meters_per_second(cur_pressure_hpa, cur_air_density_kg_m_cubed);
        
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
        cur_imu_data = imu_data_from_rx_buffer();
        new_imu = 1;
    }
}