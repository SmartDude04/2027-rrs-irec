#include "main.h"

#include "stm32h7xx_hal.h"

#include "usb_cdc.h"
#include "servo.h"
#include "I2C_DMA.h"
#include "ICM_20948_IMU.h"
#include "MPRLS_PRESSURE.h"

static void run_mprls();
static void run_imu();

#include "usb_device.h"
#include "usb_serial.h"
#include "fatfs.h"
#include "sdmmc.h"
#include "spi.h"
// #include "lora_sx1276.h"
// #include "radio.h"

static ab_mpc_t mpc;
// static lora_sx1276 lora;

/*
Basic State Machine Logic

Idle: Initial state on the pad
    Everything starts here.

Boost Phase: AIR BRAKES ARE NOT ACTIVE
    Significant acceleration is detected for more than 2 seconds and velocity
    has gone above 10 m/s

Coast Phase: AIR BRAKES ARE NOW ACTIVE
    Has been in the boost phase for 4 seconds, velo has gone below 130 m/s (426 ft/s)

Idle: apogee detected
    Acceleration has gone to 0 or have been in the coast phase for 13 seconds

Failure: Something goes wrong and everything stops
*/

enum RocketState
{
    WaitingForLaunch,
    Armed,
    Boost,
    Coast,
    Idle,
    Failure,
};

enum RocketState state;

void SystemClock_Config(void);
void Error_Handler(void);
void MPU_Config(void);
void SDIO_SDCard_Test();

// void run_mprls(void);
unsigned long run_mprls(void);
void airbrake_start(void);
void airbrake_tick(float alt_m, float vz_mps);
void servo_set_opening(float a);
static volatile uint8_t new_pressure = 0;
static volatile uint8_t new_imu = 0;
static volatile double pressure_hpa;
static volatile double altitude_m;
static volatile IMU_Data imu_data;

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    SystemCoreClockUpdate();

    USB_CDC_INIT();
    Servo_Init_TIM3_PA6();
    I2C_DMA_Init();
    mprls_init();

    const HAL_StatusTypeDef imu_init_status = imu_init();
    if (imu_init_status != HAL_OK)
    {
        HAL_Delay(3000);
        USB_CDC_PRINTF("Failed to initialize IMU: Failed with code %d\n", imu_init_status);
        while (1)
            ;
    }

    MPU_Config();
    MX_USB_DEVICE_Init();
    MX_SDMMC1_SD_Init();
    MX_FATFS_Init();

    HAL_Delay(5000);

    SDIO_SDCard_Test();

    airbrake_start();
    state = WaitingForLaunch;

    /* Original test values; replace with measured altitude and velocity. */
    float h = 231.0f;
    float v = 140.0f;

    while (1)
    {
        // Servo_Move_Degrees(0);
        // HAL_Delay(1000);
        // Servo_Move_Degrees(70);
        // HAL_Delay(1000);
        // continue;

        run_mprls();
        run_imu();

        if (new_pressure)
        {
            HAL_Delay(10);
            USB_CDC_PRINTF("Pressure (hPa): %lf\tAltitude (m): %lf\n", pressure_hpa, altitude_m);
            new_pressure = 0;
        }
        if (new_imu)
        {
            HAL_Delay(10);
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

        // Call every AB_DT_MPC seconds once timing is implemented
        // airbrake_tick(h, v);
    }
}

// void run_mprls(void)
unsigned long run_mprls(void) void run_mprls()
{
    if (mprls_status == MPRLS_IDLE)
    {
        const HAL_StatusTypeDef ret = request_mprls_read();
        if (ret != HAL_OK)
        {
            HAL_Delay(10);
            // USB_CDC_PRINTF("request_mprls_read() failed with code %d", ret);
            USB_CDC_PRINTF("request_mprls_read() failed with code %d\n", ret);
            while (1)
                ;
        }

        return 0;
    }
    if (mprls_status == MPRLS_MEASURED)
    {
        const HAL_StatusTypeDef ret = start_mprls_dma_read();
        if (ret != HAL_OK)
        {
            HAL_Delay(10);
            USB_CDC_PRINTF("start_mplrs_dma_read() failed with code %d\n", ret);
            while (1)
                ;
        }

        return 0;
    }
    if (mprls_status == MPRLS_DATA_READY)
    {
        const uint32_t pressure = ((uint32_t)mprls_pressure_rx_buffer[1] << 16) | ((uint32_t)mprls_pressure_rx_buffer[2] << 8) | (uint32_t)mprls_pressure_rx_buffer[3];

        USB_CDC_PRINTF("Pressure: %lu\n", (unsigned long)pressure);

        return pressure;

        /*
         * TODO: Convert raw pressure to altitude and estimate
         * vertical velocity, then pass those to airbrake_tick().
         */

        mprls_status = IDLE;
    }
}

void airbrake_start(void)
{
    ab_rocket_t model = ab_default_rocket();
    airbrake_mpc_init(&mpc, &model, AB_TARGET_APOGEE_M);
}

void airbrake_tick(float alt_m, float vz_mps)
{
    ab_info_t info;
    float alpha = airbrake_mpc_step(&mpc, alt_m, vz_mps, &info);
    servo_set_opening(alpha);
}

void servo_set_opening(float a)
{
    float angle = a * 70; // Taken from dynamics, have 0-70 degrees to work with for deployment
    Servo_Move_Degrees((int)angle);
    void run_imu()
    {
        if (imu_status == IMU_IDLE)
        {
            const HAL_StatusTypeDef ret = read_acceleration_gyro();
            if (ret != HAL_OK)
            {
                USB_CDC_PRINTF("Error reading acceleration/gyro: %d\n", ret);
                while (1)
                    ;
            }
        }
        if (imu_status == IMU_DATA_READY)
        {
            imu_data = imu_data_from_rx_buffer();
            new_imu = 1;
        }
    }

    /**
     * Called when a write is finished from HAL_I2C_Master_Transmit_DMA()
     */
    void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef * hi2c)
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
    void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef * hi2c)
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
    void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef * hi2c)
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

    /**
     * Called when a timer interrupt is set to fire
     */
    void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
    {
        if (htim->Instance == TIM4)
        {
            if (imu_status == IMU_DATA_READY)
            {
                imu_status = IMU_IDLE;
            }
        }
    }