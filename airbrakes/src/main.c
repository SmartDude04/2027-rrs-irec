#include "stm32h7xx_hal.h"
#include "mpc.h"
#include "example_model.h"

/* Inspect these in the debugger. No sensor, PWM, or actuator is driven. */
volatile MPCOutput mpc_example_output;
volatile uint32_t mpc_example_cycles;
volatile uint32_t mpc_example_elapsed_ms;
volatile uint32_t mpc_example_overruns;

void SysTick_Handler(void) {
    HAL_IncTick();
}

int main(void) {
    /* Cube SystemInit selects internal HSI (64 MHz) and enables the FPU.
     * Keep that clock for a crystal-independent example; no 480 MHz PLL assumed. */
    SystemCoreClockUpdate();
    if (HAL_Init() != HAL_OK) {
        while (1) { __NOP(); }
    }
    SCB_EnableICache();
    SCB_EnableDCache();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    const MPCConfig config = mpc_default_config();
    const MPCModel model = mpc_example_model();
    if (mpc_init(&config, &model) != MPC_OK) {
        mpc_example_output.status = MPC_INVALID_CONFIG;
        while (1) { __WFI(); }
    }

    /* Synthetic coast state for bench inspection. Replace this with the state
     * estimator and flight-state burnout flag when integrating flight software. */
    MPCInput input = {
        .altitude_asl_m = 1000.0, .velocity_up_m_s = 200.0,
        .time_since_launch_s = 8.0, .deployment = 0.25,
        .control_dt_s = 0.1, .motor_burned_out = 1
    };
    uint32_t previous_tick = HAL_GetTick();
    while (1) {
        const uint32_t now = HAL_GetTick();
        const uint32_t interval_ms = now - previous_tick; /* handles tick wrap */
        if (interval_ms < 100U) { __WFI(); continue; }
        previous_tick = now;
        input.control_dt_s = (double)interval_ms / 1000.0;
        MPCOutput output;
        const uint32_t start_cycles = DWT->CYCCNT;
        (void)mpc_step(&input, &output);
        mpc_example_cycles = DWT->CYCCNT - start_cycles;
        mpc_example_elapsed_ms = HAL_GetTick() - now;
        if (mpc_example_elapsed_ms >= 100U) ++mpc_example_overruns;
        mpc_example_output = output;
        if (output.command_valid) input.deployment = output.command;

        if (input.velocity_up_m_s > 0.0) {
            const double rho = model.density(input.altitude_asl_m, model.density_context);
            const double cda = config.rocket_cd * config.rocket_area_m2
                + config.airbrake_cd * config.airbrake_area_m2 * input.deployment;
            const double a = -config.gravity_m_s2 - rho * cda
                * input.velocity_up_m_s * input.velocity_up_m_s / (2.0 * model.coast_mass_kg);
            input.altitude_asl_m += input.velocity_up_m_s * input.control_dt_s;
            input.velocity_up_m_s += a * input.control_dt_s;
        }
        input.time_since_launch_s += input.control_dt_s;
    }
}
