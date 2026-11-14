#ifndef AIRBRAKES_CONSTANTS_H
#define AIRBRAKES_CONSTANTS_H

#define ACCELERATION_GRAVITY_MS2 9.80665f
#define FULLY_EXTENDED_AIRBRAKE_AREA 0.006690f
#define ROCKET_AREA 0.01904f
#define ROCKET_MASS 22.05593f
#define LAUNCH_PRESSURE_HPA 1013.25f
#define DESIRED_APOGEE_METERS 960.0f

#define QP_TIMESTEP_SECONDS 1.0f
#define QP_STATE_SIZE 15

#define AB_N              15
// #define AB_DT_MPC         1.0f      /* control step / plan segment [s] */
#define AB_DT_INT         0.01f     /* flight-model step while holding last alpha [s] */
#define AB_SUB_STEPS      20          /* RK2 sub-steps per plan segment */
#define AB_ALPHA_MAX      1.0f
#define AB_RATE_MAX       0.5f      /* max |d alpha/dt| [1/s] */
// #define AB_EPS            0.01f     /* finite-difference step */
#define AB_TRUST          0.3f      /* trust region about alpha0 */
#define AB_N_ITER         2          /* re-linearizations per step */
#define AB_W_APOGEE       1.0f

// S for stickout and Q for smoothness...why did they both have to start w/ the same letter...
static const float AB_S_DIAG[AB_N] = { 20.0f, 20.0f, 20.0f, 20.0f, 20.0f, 20.0f, 20.0f, 20.0f, 20.0f, 20.0f, 20.0f, 20.0f, 20.0f, 20.0f, 20.0f };
static const float AB_Q_DIAG[AB_N] = { 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f };

#define AB_RHO0           1.225f
#define AB_H_SCALE        8400.0f

#endif
