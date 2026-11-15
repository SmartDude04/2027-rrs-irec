#include "main.h"

#include "constants.h"
#include "stm32h7xx_hal.h"

#include "usb_cdc.h"
#include "servo.h"
#include "I2C_DMA.h"
#include "constants.h"
#include "ICM_20948_IMU.h"
#include "MPRLS_PRESSURE.h"
#include "finite_state_machine.h"
#include "kalman.h"
#include "gradient.h"
#include "usb_device.h"
#include "sdmmc.h"
#include "fatfs.h"
#include "spi.h"
#include "lookup.h"
#include "airbrake_mpc.h"

void DWT_Init(void);

static void run_kalman();
static void run_mprls();
static void run_imu();

volatile float cur_pressure_hpa;
volatile float cur_altitude_meters;
volatile float cur_air_density_kg_m_cubed;
volatile float cur_speed_of_sound_meters_per_second;
volatile IMU_Data cur_imu_data;
volatile float cur_vertical_acceleration;

static volatile uint8_t new_pressure = 0;
static volatile uint8_t new_imu = 0;
static uint8_t kalman_initialized = 0;

static float airbrake_state[QP_STATE_SIZE] = {0};

static uint32_t cycle_count_at_last_predict;

Airbrakes_Finite_State_Machine airbrakes_state = DISARMED;

static ab_mpc_t mpc;

int main(void)
{
    // TODO: Add better error handling all around, and especially in init functions
    HAL_Init();
    USB_CDC_INIT();
    Servo_Init_TIM3_PB4();
    I2C_DMA_Init();
    mprls_init();
    DWT_Init();
    MX_USB_DEVICE_Init();
    MX_SDMMC1_SD_Init();
    MX_FATFS_Init();
    SPI_Init();
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
        if (new_imu)
        {
            USB_CDC_PRINTF("Acceleration: %f\n", cur_vertical_acceleration);
            new_imu = 0;
            HAL_Delay(10);
        }
        if (new_pressure)
        {
            USB_CDC_PRINTF("Altitude: %f\n", cur_altitude_meters);
            new_pressure = 0;
            HAL_Delay(10);
        }
        // else
        // {
        //     USB_CDC_PRINTF("State: %d\n", mprls_status);
        // }
        continue;

        switch (airbrakes_state)
        {
        case DISARMED: {
            // TODO: Wait for radio
            break;
        }
        case PRE_LAUNCH: {
            run_mprls();
            run_imu();
            run_kalman();

            // TODO: Listen for radio
            // TODO: Look at accelerometer data to determine if in boost phase
            break;
        }
        case BOOST: {
            run_mprls();
            run_imu();
            run_kalman();

            // TODO: Listen for radio
            // TODO: Look at accelerometer/barometer data to determine if in coast phase
            break;
        }
        case COAST: {
            run_mprls();
            run_imu();

            run_kalman();

            // TODO: Listen for radio
            // TODO: Detect when to move into control phase
            break;
        }
        case CONTROL: {
            run_mprls();
            run_imu();
            run_kalman();

            // TODO: Listen for radio
            // TODO: Detect apogee to determine when to retract airbrakes
            break;
        }
        case EXIT: {

            // TODO: Listen for radio
            break;
        }
        case ABORT: {
            // TODO: Listen for radio
            // TODO: Retract airbrakes, if possible
            break;
        }
        }

        // USB_CDC_PRINTF("altitude: %f\t velocity: %f\t theta: %f\n", ekf_state.altitude, ekf_state.velocity, ekf_state.theta);
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

    ekf_predict(airbrake_state[0], time_since_last_predict);

    if (new_imu)
    {
        // Do not update during the boost phase; the model is not built for that
        if (airbrakes_state == COAST || airbrakes_state == CONTROL)
        {
            ekf_update_acceleration(cur_vertical_acceleration, airbrake_state[0]);
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
        // TODO: Confirm this or use trig to get a tilted version
        cur_vertical_acceleration = cur_imu_data.acceleration_z_meters_per_second_squared;
        new_imu = 1;
        // imu_sample_consumed = 1;
    }
}

void airbrake_start(void)
{
    ab_rocket_t model = ab_default_rocket();
    airbrake_mpc_init(&mpc, &model, DESIRED_APOGEE_METERS);
}

// void airbrake_tick(float alt_m, float vz_mps)
float airbrake_tick(float alt_m, float vz_mps)
{
    ab_info_t info;
    float alpha = airbrake_mpc_step(&mpc, alt_m, vz_mps, &info);
    return alpha;
    // servo_set_opening(alpha);
}

void servo_set_opening(float a)
{
    /* Also reject NaN before converting to an integer. */
    if (!(a >= 0.0f))
        a = 0.0f;
    if (a > 1.0f)
        a = 1.0f;
    const float angle = a * 70.0f;
    Servo_Move_Degrees((uint8_t)angle);
}


void test_flight(void)
{
    // Not working right now have to finish dealing w/ merge conflicts...
    for (int i = 0; i < NOISY_DATA_COUNT; i++)
    {
        const FlightSample frame = noisy_flight[i];

        float time = frame.time;
        float altitude = frame.altitude;
        float velo = frame.velo;
        float accel = frame.accel;
        float pressure = frame.pressure;

        ab_info_t info;
        float alpha = airbrake_mpc_step(&mpc, altitude, velo, &info);
        USB_CDC_PRINTF("alpha: %4.3f, time: %3.2f; height: %4.1f; velo: %3.1f; predicted_apogee: %4.1f; status: %ld; iterations: %ld, fallback: %ld\n",
                       alpha, time, altitude, velo, info.apogee_pred_m,
                       (long)info.solver_status, (long)info.solver_iters,
                       (long)info.used_fallback);
    }
}

void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    /* Disables the MPU */
    HAL_MPU_Disable();

    /** Initializes and configures the Region and the memory to be protected
     */
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress = 0x0;
    MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
    MPU_InitStruct.SubRegionDisable = 0x87;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    /* Enables the MPU */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Supply configuration update enable
     */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    /** Configure the main internal regulator output voltage
     */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
    {
    }

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 9;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
    RCC_OscInitStruct.PLL.PLLFRACN = 3072;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }
}
/**
 * Initializes a clock counter for microsecond-level accuracy for the EKF filter to tell time elapsed
 * since last call
 */
void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CYCCNT = 0;

    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
