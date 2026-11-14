#include "main.h"

#include "stm32h7xx_hal.h"

#include "usb_cdc.h"
#include "servo.h"
#include "I2C_DMA.h"
#include "ICM_20948_IMU.h"
#include "MPRLS_PRESSURE.h"
#include "finite_state_machine.h"
#include "../lib/kalman/kalman.h"

static void run_kalman();
static void run_mprls();
static void run_imu();

volatile float cur_pressure_hpa;
volatile float cur_altitude_meters;
volatile float cur_air_density_kg_m_cubed;
volatile float cur_speed_of_sound_meters_per_second;
volatile IMU_Data cur_imu_data;

static volatile uint8_t new_pressure = 0;
static volatile uint8_t new_imu = 0;
static uint8_t kalman_initialized = 0;

static volatile float airbrake_active_area = 0;
static uint32_t cycle_count_at_last_predict;

Airbrakes_Finite_State_Machine airbrakes_state = DISARMED;

void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CYCCNT = 0;

    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

int main(void)
{
    // TODO: Add better error handling all around, and especially in init functions
    HAL_Init();
    USB_CDC_INIT();
    Servo_Init_TIM3_PA6();
    I2C_DMA_Init();
    mprls_init();
    DWT_Init();
    const HAL_StatusTypeDef imu_init_status = imu_init();
    if (imu_init_status != HAL_OK)
    {
        HAL_Delay(3000);
        USB_CDC_PRINTF("Failed to initialize IMU: Failed with code %d\n", imu_init_status);
        while (1);
    }
       
    HAL_Delay(3000);
    
    while (1)
    {
        // Servo_Move_Degrees(0);
        // HAL_Delay(1000);
        // Servo_Move_Degrees(30);
        // HAL_Delay(1000);
        // continue;
        
        run_mprls();
        run_imu();
        
        run_kalman();
        
        USB_CDC_PRINTF("altitude: %f\t velocity: %f\t theta: %f\n", ekf_state.altitude, ekf_state.velocity, ekf_state.theta);
        // if (new_pressure)
        // {
        //     HAL_Delay(10);
        //     USB_CDC_PRINTF("Pressure (hPa): %lf\tAltitude (m): %lf\t Air density (kg/m^3): %lf\t Speed of sound (m/s): %lf\n", cur_pressure_hpa, cur_altitude_meters, cur_air_density_kg_m_cubed, cur_speed_of_sound_meters_per_second);
        //     new_pressure = 0;
        // }
        // if (new_imu)
        // {
        //     HAL_Delay(10);
        //     USB_CDC_PRINTF("ax: %lf ay: %lf ax: %lf gx %lf gy %lf gz %lf temp: %lf\n",
        //         cur_imu_data.acceleration_x_meters_per_second_squared,
        //         cur_imu_data.acceleration_y_meters_per_second_squared,
        //         cur_imu_data.acceleration_z_meters_per_second_squared,
        //         cur_imu_data.gyro_x_degrees_per_second,
        //         cur_imu_data.gyro_y_degrees_per_second,
        //         cur_imu_data.gyro_z_degrees_per_second,
        //         cur_imu_data.temperature_celsius);
        //     new_imu = 0;
        // }
        //
        // HAL_Delay(1);
    }
}

static void run_kalman()
{
    if (!kalman_initialized)
    {
        if (new_imu && new_pressure)
        {
            ekf_init(cur_altitude_meters, 0.0f);
            kalman_initialized = 1;
            cycle_count_at_last_predict = DWT->CYCCNT;

            new_imu = 0;
            new_pressure = 0;
        }
        
        return;
    }

    const uint32_t prev_cycle_count = cycle_count_at_last_predict;
    cycle_count_at_last_predict = DWT->CYCCNT;

    // Before launch, track the barometer without predicting a falling rocket.
    // This is because the EKF does not predict state on the pad
    if (airbrakes_state == DISARMED || airbrakes_state == PRE_LAUNCH)
    {
        if (new_pressure)
        {
            ekf_init(cur_altitude_meters, 0.0f);
            new_pressure = 0;
        }

        new_imu = 0;
        return;
    }

    const float time_since_last_predict = (float) (cycle_count_at_last_predict - prev_cycle_count) / (float) SystemCoreClock;

    ekf_predict(airbrake_active_area, time_since_last_predict);

    if (new_imu)
    {
        // Do not update during the boost phase; the model is not built for that
        if (airbrakes_state == COAST || airbrakes_state == CONTROL)
        {
            const float acceleration_vertical = cur_imu_data.acceleration_z_meters_per_second_squared;

            ekf_update_acceleration(acceleration_vertical, airbrake_active_area);
        }

        new_imu = 0;
    }

    if (new_pressure)
    {
        ekf_update_barometer(cur_altitude_meters);
        new_pressure = 0;
    }
}

static void run_mprls()
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
        cur_air_density_kg_m_cubed = air_density_kg_per_meter_cubed(cur_altitude_meters);
        cur_speed_of_sound_meters_per_second = speed_of_sound_meters_per_second(cur_pressure_hpa, cur_air_density_kg_m_cubed);
        
        new_pressure = 1;
        mprls_status = MPRLS_IDLE;
    }
}

static void run_imu()
{
    if (imu_status == IMU_MAKE_MEASUREMENT)
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
        imu_status = IMU_IDLE;
    }
}