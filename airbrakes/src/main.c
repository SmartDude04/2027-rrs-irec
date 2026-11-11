#include "main.h"
#include "stm32h7xx_hal.h"

#include "usb_cdc.h"
#include "servo.h"
#include "I2C_DMA.h"
#include "../lib/MPRLS_PRESSURE/MPRLS_PRESSURE.h"

#include "airbrake_config.h"
#include "airbrake_mpc.h"

static ab_mpc_t mpc;

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

Failure mode: Something goes wrong and everything stops
*/

enum RocketState {
    WaitingForLaunch,
    Armed,
    Boost,
    Coast,
    Idle,
    Failure,
};

enum RocketState state;

static volatile float last_alpha;

void run_mprls(void);

void airbrake_start(void)
{
    ab_rocket_t model = ab_default_rocket();
    airbrake_mpc_init(&mpc, &model, AB_TARGET_APOGEE_M);
}

void servo_set_opening(float a)
{
    last_alpha = a;
    /* Stub: replace with your PWM code. */
}

void airbrake_tick(float alt_m, float vz_mps)
{
    ab_info_t info;
    float alpha = airbrake_mpc_step(&mpc, alt_m, vz_mps, &info);
    servo_set_opening(alpha);
}

int main(void)
{
    HAL_Init();
    SystemCoreClockUpdate();

    USB_CDC_INIT();
    Servo_Init_TIM3_PA6();
    I2C1_DMA_Init();
    mprls_init();

    HAL_Delay(3000);

    airbrake_start();
    state = WaitingForLaunch;

    /* Original test values; replace with measured altitude and velocity. */
    float h = 231.0f;
    float v = 140.0f;

    while (1)
    {
        run_mprls();

        /* Call every AB_DT_MPC seconds once timing is implemented. */
        airbrake_tick(h, v);
    }
}

void run_mprls(void)
{
    if (mprls_status == IDLE)
    {
        const HAL_StatusTypeDef ret = request_mprls_read();

        if (ret != HAL_OK)
        {
            HAL_Delay(10);
            USB_CDC_PRINTF(
                "request_mprls_read() failed with code %d", ret
            );
            while (1);
        }
    }

    if (mprls_status == MEASURED)
    {
        const HAL_StatusTypeDef ret = start_mprls_dma_read();

        if (ret != HAL_OK)
        {
            HAL_Delay(10);
            USB_CDC_PRINTF(
                "start_mprls_dma_read() failed with code %d", ret
            );
            while (1);
        }
    }

    if (mprls_status == DATA_READY)
    {
        const uint32_t pressure =
            ((uint32_t)mprls_pressure_rx_buffer[1] << 16)
            | ((uint32_t)mprls_pressure_rx_buffer[2] << 8)
            | (uint32_t)mprls_pressure_rx_buffer[3];

        USB_CDC_PRINTF("Pressure: %lu\n", (unsigned long)pressure);

        /*
         * TODO: Convert raw pressure to altitude and estimate
         * vertical velocity, then pass those to airbrake_tick().
         */

        mprls_status = IDLE;
    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Devaddress == (MPRLS_PRESSURE_I2C_ADDRESS << 1))
    {
        if (mprls_status == TX_PENDING)
        {
            mprls_status = MEASURING;
        }
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Devaddress == (MPRLS_PRESSURE_I2C_ADDRESS << 1))
    {
        if (mprls_status == RX_PENDING)
        {
            mprls_status = DATA_READY;
        }
    }
}

void HAL_GPIO_EXTI_Callback(const uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_8)
    {
        if (mprls_status == MEASURING)
        {
            mprls_status = MEASURED;
        }
    }
}
