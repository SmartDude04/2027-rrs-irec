/*
 * Airbrake MPC for STM32 (OSQP generated solver, float32).
 *
 * Port of airbrake_mpc.py.  Everything is statically allocated: no malloc.
 * Call airbrake_mpc_init() once, then airbrake_mpc_step() once per control
 * step (every AB_DT_MPC seconds) after burnout until apogee.
 *
 * Requires airbrake_config.h, which is written by airbrake_mpc_codegen.py.
 */
#ifndef AIRBRAKE_MPC_H
#define AIRBRAKE_MPC_H

#include <stdint.h>
#include "airbrake_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Rocket coast model parameters (drag-only, vertical flight) */
typedef struct {
  float mass;
  float body_area;
  float body_cd;
  float ab_area;
  float ab_cd;
  float cd_scale;     /* 1.0 for the controller's model; != 1 only in tests */
} ab_rocket_t;

/* Diagnostics returned by each step */
typedef struct {
  float   apogee_pred_m;    /* predicted apogee with the planned schedule     */
  int32_t solver_status;    /* OSQP status of the last solve (1 = solved)     */
  int32_t solver_iters;     /* ADMM iterations of the last solve              */
  int32_t used_fallback;    /* 1 if the QP failed and the shifted plan was used */
  uint32_t model_evals;     /* accel() evaluations this step (cost estimate)  */
} ab_info_t;

typedef struct {
  ab_rocket_t model;
  float target_apogee_m;
  float alpha_plan[AB_N];   /* linearization point alpha0 (shifted last plan) */
  float alpha_prev;         /* last commanded opening                         */
  uint32_t evals;           /* running accel() counter                        */
} ab_mpc_t;

/* Default model from airbrake_config.h */
ab_rocket_t ab_default_rocket(void);

/* Integrate one segment of constant alpha (RK2, n_sub sub-steps). */
void ab_rocket_step(const ab_rocket_t *r, float *h, float *v, float alpha,
                    float dt, int n_sub, uint32_t *evals);

/* Reset controller state. */
void airbrake_mpc_init(ab_mpc_t *m, const ab_rocket_t *model, float target_apogee_m);

/* One control step. h = altitude [m], v = vertical velocity [m/s] (up positive).
 * Returns the airbrake opening command in [0, AB_ALPHA_MAX].              */
float airbrake_mpc_step(ab_mpc_t *m, float h, float v, ab_info_t *info);

/* Predicted apogee for a piecewise-constant plan (last value held). */
float ab_predict_apogee(const ab_rocket_t *r, float h, float v,
                        const float *alpha_plan, uint32_t *evals);

#ifdef __cplusplus
}
#endif

#endif /* AIRBRAKE_MPC_H */
