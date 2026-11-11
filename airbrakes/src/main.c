#include "main.h"
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"

#include "airbrake_config.h"
#include "airbrake_mpc.h"

#include "usb_cdc.h"
#include "servo.h"
#include "I2C_DMA.h"
#include "../lib/MPRLS_PRESSURE/MPRLS_PRESSURE.h"

// #define PRESSURE_RAW_MIN       1677722.0f  /* 0x19999A */
// #define PRESSURE_RAW_MAX       15099494.0f /* 0xE66666 */
// #define PRESSURE_MIN_PA         0.0f
// #define PRESSURE_MAX_PA         (25.0f * 6894.757f)

// #define PAD_CALIBRATION_SAMPLES 64U
// #define SENSOR_INTERVAL_MS      20U
// #define SENSOR_TIMEOUT_MS       500U

// /* Tunable low-pass filter time constants. */
// #define ALTITUDE_FILTER_TAU_S   0.10f
// #define VELOCITY_FILTER_TAU_S   0.25f

static ab_mpc_t mpc;

enum RocketState {
    WaitingForLaunch,
    Armed,
    Boost,
    Coast,
    Idle,
    Failure,
};

enum RocketState state = WaitingForLaunch;

static float pad_pressure_pa;
static float pad_pressure_sum;
static uint32_t calibration_samples;

static float altitude_m;
static float vertical_speed_mps;
static bool estimate_ready;

static uint32_t last_sample_ms;
static uint32_t last_valid_sample_ms;
static uint32_t last_request_ms;
static uint32_t last_mpc_ms;

static volatile float last_alpha;

static void run_mprls(void);
static void process_pressure(uint32_t raw, uint32_t now);
static void enter_failure(void);

void airbrake_start(void)
{
    ab_rocket_t model = ab_default_rocket();
    airbrake_mpc_init(&mpc, &model, AB_TARGET_APOGEE_M);
}

/*
 * Replace this body with the appropriate PWM function from servo.h.
 * a is a normalized opening: 0 = closed, 1 = fully open.
 */
void servo_set_opening(float a)
{
    if (!isfinite(a)) {
        a = 0.0f;
    }

    if (a < 0.0f) {
        a = 0.0f;
    } else if (a > 1.0f) {
        a = 1.0f;
    }

    last_alpha = a;

    /* TODO: send a to your actual servo driver. */
}

void airbrake_tick(float alt_m, float vz_mps)
{
    ab_info_t info;

    float alpha = airbrake_mpc_step(
        &mpc,
        alt_m,
        vz_mps,
        &info
    );

    if (!isfinite(alpha)) {
        enter_failure();
        return;
    }

    servo_set_opening(alpha);
}

/*
 * Call once from your flight-state logic when burnout/coast is detected.
 * Returns false if the altitude estimator is not ready.
 */
bool airbrake_enter_coast(void)
{
    if (state == Failure || !estimate_ready) {
        return false;
    }

    if ((uint32_t)(HAL_GetTick() - last_valid_sample_ms)
        > SENSOR_TIMEOUT_MS) {
        enter_failure();
        return false;
    }

    airbrake_start();
    last_mpc_ms = HAL_GetTick();
    state = Coast;
    return true;
}

static void enter_failure(void)
{
    state = Failure;
    servo_set_opening(0.0f);
}

static void process_pressure(uint32_t raw, uint32_t now)
{
    const float pressure_pa = PRESSURE_MIN_PA
            + ((float)raw - PRESSURE_RAW_MIN)
            * (PRESSURE_MAX_PA - PRESSURE_MIN_PA)
            / (PRESSURE_RAW_MAX - PRESSURE_RAW_MIN);

    if (!isfinite(pressure_pa)
        || pressure_pa <= 0.0f
        || (float)raw < PRESSURE_RAW_MIN
        || (float)raw > PRESSURE_RAW_MAX) {
        enter_failure();
        return;
    }

    last_valid_sample_ms = now;

    /*
     * Establish launch-pad pressure while the rocket is stationary.
     * Do not move or launch during this calibration.
     */
    if (calibration_samples < PAD_CALIBRATION_SAMPLES) {
        pad_pressure_sum += pressure_pa;
        calibration_samples++;

        if (calibration_samples == PAD_CALIBRATION_SAMPLES) {
            pad_pressure_pa = pad_pressure_sum / (float)PAD_CALIBRATION_SAMPLES;

            altitude_m = 0.0f;
            vertical_speed_mps = 0.0f;
            last_sample_ms = now;
            estimate_ready = true;

            USB_CDC_PRINTF("Pad pressure calibration complete\n");
        }

        return;
    }

    /*
     * Standard-atmosphere approximation, referenced to pad pressure.
     * Positive altitude means above the launch pad.
     */
    const float measured_altitude_m = 44330.0f * (1.0f - powf(pressure_pa / pad_pressure_pa, 0.19029495f));

    if (!isfinite(measured_altitude_m)) {
        enter_failure();
        return;
    }

    const uint32_t elapsed_ms = (uint32_t)(now - last_sample_ms);

    if (elapsed_ms == 0U) {
        return;
    }

    const float dt = (float)elapsed_ms * 0.001f;
    last_sample_ms = now;

    const float altitude_gain =
        dt / (ALTITUDE_FILTER_TAU_S + dt);

    const float previous_altitude_m = altitude_m;

    altitude_m += altitude_gain * (measured_altitude_m - altitude_m);

    const float measured_velocity_mps =
        (altitude_m - previous_altitude_m) / dt;

    const float velocity_gain =
        dt / (VELOCITY_FILTER_TAU_S + dt);

    vertical_speed_mps += velocity_gain * (measured_velocity_mps - vertical_speed_mps);
}

static void run_mprls(void)
{
    const uint32_t now = HAL_GetTick();

    if (mprls_status == DATA_READY) {
        const uint8_t sensor_status = mprls_pressure_rx_buffer[0];

        /*
         * MPR status: busy=0x20, memory error=0x04,
         * mathematical saturation=0x01.
         */
        if ((sensor_status & 0x25U) != 0U) {
            enter_failure();
            return;
        }

        const uint32_t raw =
            ((uint32_t)mprls_pressure_rx_buffer[1] << 16)
            | ((uint32_t)mprls_pressure_rx_buffer[2] << 8)
            | (uint32_t)mprls_pressure_rx_buffer[3];

        mprls_status = IDLE;
        process_pressure(raw, now);
        return;
    }

    if (mprls_status == MEASURED) {
        const HAL_StatusTypeDef ret = start_mprls_dma_read();

        if (ret != HAL_OK) {
            enter_failure();
        }

        return;
    }

    if (mprls_status == IDLE && (uint32_t)(now - last_request_ms) >= SENSOR_INTERVAL_MS) {
        last_request_ms = now;

        const HAL_StatusTypeDef ret = request_mprls_read();

        if (ret != HAL_OK) {
            enter_failure();
        }
    }
}

int main(void)
{
    HAL_Init();

    /*
     * Keep your project's clock configuration here, if required.
     * SystemCoreClockUpdate() reports the configured clock;
     * it does not configure the PLL or peripheral clocks.
     */
    SystemCoreClockUpdate();

    USB_CDC_INIT();
    Servo_Init_TIM3_PA6();
    I2C1_DMA_Init();
    mprls_init();

    servo_set_opening(0.0f);

    HAL_Delay(3000);

    state = WaitingForLaunch;

    last_request_ms = HAL_GetTick();
    last_valid_sample_ms = last_request_ms;

    const uint32_t mpc_period_ms =
        (uint32_t)(AB_DT_MPC * 1000.0f + 0.5f);

    if (mpc_period_ms == 0U) {
        enter_failure();
    }

    while (1) {
        if (state == Failure) {
            continue;
        }

        run_mprls();

        if (state == Failure) {
            continue;
        }

        const uint32_t now = HAL_GetTick();

        if ((uint32_t)(now - last_valid_sample_ms)
            > SENSOR_TIMEOUT_MS) {
            enter_failure();
            continue;
        }

        /*
         * Add your launch/burnout state transitions here.
         *
         * At confirmed burnout:
         *     airbrake_enter_coast();
         *
         * At confirmed apogee:
         *     state = Idle;
         *
         * This deliberately does not invent acceleration-based
         * detection from a barometer alone.
         */

        if (state != Coast || !estimate_ready) {
            servo_set_opening(0.0f);
            continue;
        }

        if ((uint32_t)(now - last_mpc_ms) >= mpc_period_ms) {
            last_mpc_ms = now;

            airbrake_tick(altitude_m, vertical_speed_mps);
        }
    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1
        && hi2c->Devaddress == (MPRLS_PRESSURE_I2C_ADDRESS << 1)) {
        if (mprls_status == TX_PENDING) {
            mprls_status = MEASURING;
        }
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1
        && hi2c->Devaddress == (MPRLS_PRESSURE_I2C_ADDRESS << 1)) {
        if (mprls_status == RX_PENDING) {
            mprls_status = DATA_READY;
        }
    }
}

void HAL_GPIO_EXTI_Callback(const uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_8) {
        if (mprls_status == MEASURING) {
            mprls_status = MEASURED;
        }
    }
}

// <<<<<<< HEAD
// #include "stm32h7xx.h"

// #include "airbrake_config.h"
// #include "airbrake_mpc.h"

// static ab_mpc_t mpc;

// enum RocketState {
//     WaitingForLaunch,
//     Armed,
//     Boost,
//     Coast,
//     Idle,
//     Failure,
// };

// enum RocketState state;

// /*
// Basic State Machine Logic

// Idle: Initial state on the pad
//     Everything starts here.

// Boost Phase: AIR BRAKES ARE NOT ACTIVE
//     Significant acceleration is detected for more than 2 seconds and velocity
//     has gone above 10 m/s

// Coast Phase: AIR BRAKES ARE NOW ACTIVE
//     Has been in the boost phase for 4 seconds, velo has gone below 130 m/s (426 ft/s)

// Idle: apogee detected
//     Acceleration has gone to 0 or have been in the coast phase for 13 seconds

// Failure mode: Something goes wrong and everything stops

// */

// void airbrake_start(void) {                 /* once, e.g. at burnout detect */
//     ab_rocket_t model = ab_default_rocket();
//     airbrake_mpc_init(&mpc, &model, AB_TARGET_APOGEE_M);
// }

// static volatile float last_alpha;
// void servo_set_opening(float a) {
//     last_alpha = a;
// }   /* stub: replace with your PWM code */

// void airbrake_tick(float alt_m, float vz_mps) {   /* every AB_DT_MPC seconds */
//     ab_info_t info;
//     float alpha = airbrake_mpc_step(&mpc, alt_m, vz_mps, &info);
//     servo_set_opening(alpha);               /* 0 = closed, 1 = full */
// }

// int main(void) {
//     SystemCoreClockUpdate();
//     airbrake_start();

//     state = WaitingForLaunch;

//     float h = 231.0f, v = 140.0f;          /* fake state, only to exercise the solver */
//     while (1) {
//         airbrake_tick(h, v);               /* replace with your filtered altitude and velocity */
//         h += v; v -= 12.0f;
//         if (v < 1.0f) { h = 231.0f; v = 140.0f; airbrake_start(); }
//         for (volatile uint32_t i = 0; i < 1000000; i++) { }
//     }
// }
// =======
// #include "main.h"

// #include "stm32h7xx_hal.h"

// #include "usb_cdc.h"
// #include "servo.h"
// #include "I2C_DMA.h"
// #include "../lib/MPRLS_PRESSURE/MPRLS_PRESSURE.h"

// void run_mprls();

// int main(void)
// {
//     HAL_Init();
//     USB_CDC_INIT();
//     Servo_Init_TIM3_PA6();
//     I2C1_DMA_Init();
//     mprls_init();

//     HAL_Delay(3000);

//     while (1)
//     {
//         run_mprls();
//     }
// }

// void run_mprls()
// {
//     if (mprls_status == IDLE)
//     {
//         const HAL_StatusTypeDef ret = request_mprls_read();
//         if (ret != HAL_OK)
//         {
//             HAL_Delay(10);
//             USB_CDC_PRINTF("request_mprls_read() failed with code %d", ret);
//             while (1);
//         }
//     }
//     if (mprls_status == MEASURED)
//     {
//         const HAL_StatusTypeDef ret = start_mprls_dma_read();
//         if (ret != HAL_OK)
//         {
//             HAL_Delay(10);
//             USB_CDC_PRINTF("start_mplrs_dma_read() failed with code %d", ret);
//             while (1);
//         }
//     }
//     if (mprls_status == DATA_READY)
//     {
//         const uint32_t pressure = (mprls_pressure_rx_buffer[1] << 16) + (mprls_pressure_rx_buffer[2] << 8) + mprls_pressure_rx_buffer[3];
//         USB_CDC_PRINTF("Pressure: %d\n", pressure);
//         mprls_status = IDLE;
//     }
// }

// void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
// {
//     if (hi2c->Devaddress == MPRLS_PRESSURE_I2C_ADDRESS << 1)
//     {
//         if (mprls_status == TX_PENDING)
//         {
//             mprls_status = MEASURING;
//         }
//     }
// }

// /**
//  * Called when DMA memory is ready from HAL_I2C_Master_Receive_DMA() read
//  */
// void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
// {
//     if (hi2c->Devaddress == MPRLS_PRESSURE_I2C_ADDRESS << 1)
//     {
//         if (mprls_status == RX_PENDING)
//         {
//             mprls_status = DATA_READY;
//         }
//     }
// }

// /**
//  * Called during GPIO interrupt
//  */
// void HAL_GPIO_EXTI_Callback(const uint16_t GPIO_Pin)
// {
//     if (GPIO_Pin == GPIO_PIN_8)
//     {
//         if (mprls_status == MEASURING)
//         {
//             mprls_status = MEASURED;
//         }
//     }
// }
// >>>>>>> airbrakes-sensors
