#include "stm32h7xx.h"

#include "airbrake_config.h"
#include "airbrake_mpc.h"

static ab_mpc_t mpc;

void airbrake_start(void) {                 /* once, e.g. at burnout detect */
    ab_rocket_t model = ab_default_rocket();
    airbrake_mpc_init(&mpc, &model, AB_TARGET_APOGEE_M);
}

static volatile float last_alpha;
void servo_set_opening(float a) {
    last_alpha = a;
}   /* stub: replace with your PWM code */

void airbrake_tick(float alt_m, float vz_mps) {   /* every AB_DT_MPC seconds */
    ab_info_t info;
    float alpha = airbrake_mpc_step(&mpc, alt_m, vz_mps, &info);
    servo_set_opening(alpha);               /* 0 = closed, 1 = full */
}

int main(void) {
    SystemCoreClockUpdate();
    airbrake_start();

    float h = 231.0f, v = 140.0f;          /* fake state, only to exercise the solver */
    while (1) {
        airbrake_tick(h, v);               /* replace with your filtered altitude and velocity */
        h += v; v -= 12.0f;
        if (v < 1.0f) { h = 231.0f; v = 140.0f; airbrake_start(); }
        for (volatile uint32_t i = 0; i < 1000000; i++) { }
    }
}
