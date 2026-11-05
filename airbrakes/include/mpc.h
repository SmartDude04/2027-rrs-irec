#ifndef AIRBRAKE_MPC_H
#define AIRBRAKE_MPC_H

#include <stddef.h>
#include "osqp_mpc/mpc_dimensions.h"

#ifdef __cplusplus
extern "C" {
#endif

/* One statically allocated controller/solver; not reentrant or ISR-safe. */
typedef enum {
    MPC_OK = 0,
    MPC_INACTIVE = 1,
    MPC_INVALID_INPUT = -1,
    MPC_INVALID_CONFIG = -2,
    MPC_MODEL_ERROR = -3,
    MPC_SOLVER_ERROR = -4,
    MPC_NOT_INITIALIZED = -5
} MPCStatus;

typedef struct {
    double target_apogee_agl_m;
    double prediction_dt_s;
    double max_deployment_rate;
    double rocket_cd;
    double airbrake_cd;
    double rocket_area_m2;
    double airbrake_area_m2;
    double gravity_m_s2;
    double altitude_scale_m;
    double w_apogee;
    double w_smooth;
    double w_regularization;
    unsigned int max_coast_steps; /* 1..2000; bounds prediction work */
    double min_control_velocity_m_s;
} MPCConfig;

/* Density must return finite, nonnegative kg/m^3 at ASL altitude (m).
 * Returning NaN signals unavailable model data. Context must outlive use. */
typedef double (*MPCDensityFunction)(double altitude_asl_m, void *context);
typedef struct {
    double coast_mass_kg;
    double elevation_asl_m;
    MPCDensityFunction density;
    void *density_context;
} MPCModel;

typedef struct {
    const double *altitude_asl_m;
    const double *density_kg_m3;
    size_t count;
} MPCDensityTable;

typedef struct {
    double altitude_asl_m;
    double velocity_up_m_s;
    double time_since_launch_s;
    double deployment;       /* measured position if available, otherwise command */
    double control_dt_s;     /* elapsed interval, 0.1 s at nominal 10 Hz */
    int motor_burned_out;    /* flight-state logic supplies 0 or 1 */
} MPCInput;

typedef struct {
    double command;
    int command_valid;      /* false if deployment is not finite or input is NULL */
    MPCStatus status;
    int solver_status;     /* OSQP status_val, 0 when solver was not called */
    int solver_error;      /* OSQP API error code, 0 when no API error */
    int iterations;
    double nominal_apogee_asl_m;
    double primal_residual;
    double dual_residual;
} MPCOutput;

/* Read-only snapshot of the most recently prepared QP, for verification/logging.
 * P stores its full upper triangle in CSC order: (0,0),(0,1),(1,1),... . */
typedef struct {
    double nominal[MPC_HORIZON];
    double gradient[MPC_HORIZON];
    double p_upper[MPC_P_NNZ];
    double q[MPC_HORIZON];
    double lower[MPC_CONSTRAINTS];
    double upper[MPC_CONSTRAINTS];
    double nominal_apogee_asl_m;
} MPCProblem;

MPCConfig mpc_default_config(void);
MPCStatus mpc_init(const MPCConfig *config, const MPCModel *model);
/* Clear the previous plan and solver warm start between flights. */
void mpc_reset(void);
MPCStatus mpc_step(const MPCInput *input, MPCOutput *output);
/* NULL until a QP is prepared; invalidated by the next step/reset/init. */
const MPCProblem *mpc_last_problem(void);

/* Piecewise-linear interpolation; rejects altitudes outside the table.
 * Arrays must be finite, immutable, strictly increasing in altitude, >=2 rows.
 * mpc_init validates the table when this callback is used. */
int mpc_density_table_valid(const MPCDensityTable *table);
double mpc_density_from_table(double altitude_asl_m, void *context);

#ifdef __cplusplus
}
#endif
#endif
