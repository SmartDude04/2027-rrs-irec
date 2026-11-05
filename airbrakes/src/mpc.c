#include "mpc.h"

#include <math.h>
#include <string.h>
#include "osqp_mpc/osqp.h"
#include "osqp_mpc/airbrake_workspace.h"

#if OSQP_EMBEDDED_MODE != 2 || defined(OSQP_USE_FLOAT) || defined(OSQP_USE_LONG)
#error "Regenerate OSQP for matrix updates, double precision and int indices"
#endif
_Static_assert(sizeof(OSQPFloat) == sizeof(double), "OSQP precision mismatch");

static MPCConfig cfg;
static MPCModel model;
static MPCProblem problem;
static double last_solution[MPC_HORIZON];
static int initialized;
static int have_solution;
static int problem_valid;

static double clamp(double value, double lower, double upper) {
    return fmin(upper, fmax(lower, value));
}

static int all_finite(const double *values, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        if (!isfinite(values[i])) return 0;
    }
    return 1;
}

MPCConfig mpc_default_config(void) {
    const MPCConfig defaults = {
        .target_apogee_agl_m = 3000.0,
        .prediction_dt_s = 0.25,
        .max_deployment_rate = 0.2,
        .rocket_cd = 0.3,
        .airbrake_cd = 0.3,
        .rocket_area_m2 = 3.14159265358979323846 * 0.0762 * 0.0762,
        .airbrake_area_m2 = 0.060 * 0.040,
        .gravity_m_s2 = 9.81,
        .altitude_scale_m = 100.0,
        .w_apogee = 5.0,
        .w_smooth = 5.0,
        .w_regularization = 0.1,
        .max_coast_steps = 2000,
        .min_control_velocity_m_s = 5.0
    };
    return defaults;
}

static int config_valid(const MPCConfig *c) {
    if (!c) return 0;
    const double values[] = {
        c->target_apogee_agl_m, c->prediction_dt_s, c->max_deployment_rate,
        c->rocket_cd, c->airbrake_cd, c->rocket_area_m2, c->airbrake_area_m2,
        c->gravity_m_s2, c->altitude_scale_m, c->w_apogee, c->w_smooth,
        c->w_regularization, c->min_control_velocity_m_s
    };
    return all_finite(values, sizeof(values) / sizeof(values[0]))
        && c->target_apogee_agl_m >= 0.0 && c->prediction_dt_s > 0.0
        && c->max_deployment_rate >= 0.0 && c->rocket_cd >= 0.0
        && c->airbrake_cd >= 0.0 && c->rocket_area_m2 >= 0.0
        && c->airbrake_area_m2 >= 0.0 && c->gravity_m_s2 > 0.0
        && c->altitude_scale_m > 0.0 && c->w_apogee >= 0.0
        && c->w_smooth >= 0.0 && c->w_regularization > 0.0
        && c->max_coast_steps > 0 && c->max_coast_steps <= 2000
        && c->min_control_velocity_m_s >= 0.0;
}

int mpc_density_table_valid(const MPCDensityTable *table) {
    if (!table || !table->altitude_asl_m || !table->density_kg_m3 || table->count < 2)
        return 0;
    for (size_t i = 0; i < table->count; ++i) {
        if (!isfinite(table->altitude_asl_m[i])
            || !isfinite(table->density_kg_m3[i]) || table->density_kg_m3[i] < 0.0)
            return 0;
        if (i && table->altitude_asl_m[i] <= table->altitude_asl_m[i - 1]) return 0;
    }
    return 1;
}

double mpc_density_from_table(double h, void *context) {
    const MPCDensityTable *table = (const MPCDensityTable *)context;
    if (!table || !table->altitude_asl_m || !table->density_kg_m3
        || table->count < 2 || !isfinite(h)) return NAN;
    if (h < table->altitude_asl_m[0] || h > table->altitude_asl_m[table->count - 1])
        return NAN;
    size_t low = 0, high = table->count - 1;
    while (high - low > 1) {
        const size_t mid = low + (high - low) / 2;
        if (h < table->altitude_asl_m[mid]) high = mid;
        else low = mid;
    }
    const double span = table->altitude_asl_m[high] - table->altitude_asl_m[low];
    if (!(span > 0.0) || !isfinite(span)) return NAN;
    const double fraction = (h - table->altitude_asl_m[low]) / span;
    return table->density_kg_m3[low]
        + fraction * (table->density_kg_m3[high] - table->density_kg_m3[low]);
}

void mpc_reset(void) {
    have_solution = 0;
    problem_valid = 0;
    memset(last_solution, 0, sizeof(last_solution));
    osqp_cold_start(&airbrake_solver);
    /* Restore the generated starting penalty after adaptive-rho solves. */
    (void)osqp_update_rho(&airbrake_solver, 0.1);
}

MPCStatus mpc_init(const MPCConfig *config, const MPCModel *new_model) {
    initialized = 0;
    mpc_reset();
    if (!config_valid(config) || !new_model || !new_model->density
        || !isfinite(new_model->coast_mass_kg) || new_model->coast_mass_kg <= 0.0
        || !isfinite(new_model->elevation_asl_m)
        || !isfinite(config->target_apogee_agl_m + new_model->elevation_asl_m))
        return MPC_INVALID_CONFIG;
    if (new_model->density == mpc_density_from_table
        && !mpc_density_table_valid((const MPCDensityTable *)new_model->density_context))
        return MPC_INVALID_CONFIG;
    cfg = *config;
    model = *new_model;
    initialized = 1;
    return MPC_OK;
}

static int density(double h, double *rho) {
    *rho = model.density(h, model.density_context);
    return isfinite(*rho) && *rho >= 0.0;
}

static int acceleration(double h, double v, double deployment, double *accel) {
    double rho;
    if (!density(h, &rho)) return 0;
    const double cda = cfg.rocket_cd * cfg.rocket_area_m2
        + cfg.airbrake_cd * cfg.airbrake_area_m2 * deployment;
    *accel = -cfg.gravity_m_s2 - rho * cda * v * fabs(v) / (2.0 * model.coast_mass_kg);
    return isfinite(*accel);
}

static int coast_apogee(double h, double v, double deployment, double *apogee) {
    if (v <= 0.0) { *apogee = h; return isfinite(h); }
    const double dt = cfg.prediction_dt_s;
    for (unsigned int k = 0; k < cfg.max_coast_steps; ++k) {
        double a;
        if (!acceleration(h, v, deployment, &a)) return 0;
        const double next_v = v + dt * a;
        if (!isfinite(next_v)) return 0;
        if (next_v <= 0.0) {
            /* Match the reference's constant-acceleration crossing interpolation. */
            const double step_a = (next_v - v) / dt;
            const double tau = -v / step_a;
            *apogee = h + v * tau + 0.5 * step_a * tau * tau;
            return isfinite(*apogee);
        }
        h += dt * v;  /* Forward Euler, as in linearmodel.py. */
        v = next_v;
        if (!isfinite(h)) return 0;
    }
    return 0;
}

static int nominal_and_gradient(double h, double v) {
    double sh[MPC_HORIZON] = {0};
    double sv[MPC_HORIZON] = {0};
    const double dt = cfg.prediction_dt_s;
    for (int k = 0; k < MPC_HORIZON && v > 0.0; ++k) {
        double rho, plus, minus, a;
        if (!density(h, &rho) || !density(h + 1.0, &plus) || !density(h - 1.0, &minus)
            || !acceleration(h, v, problem.nominal[k], &a)) return 0;
        const double rho_h = (plus - minus) / 2.0;
        const double cda = cfg.rocket_cd * cfg.rocket_area_m2
            + cfg.airbrake_cd * cfg.airbrake_area_m2 * problem.nominal[k];
        const double a10 = -dt * rho_h * cda * v * fabs(v) / (2.0 * model.coast_mass_kg);
        const double a11 = 1.0 - dt * rho * cda * fabs(v) / model.coast_mass_kg;
        const double b1 = -dt * rho * cfg.airbrake_cd * cfg.airbrake_area_m2
            * v * fabs(v) / (2.0 * model.coast_mass_kg);
        for (int j = 0; j < MPC_HORIZON; ++j) {
            const double previous_h = sh[j], previous_v = sv[j];
            sh[j] = previous_h + dt * previous_v;
            sv[j] = a10 * previous_h + a11 * previous_v;
        }
        sv[k] += b1;
        if (!all_finite(sh, MPC_HORIZON) || !all_finite(sv, MPC_HORIZON)) return 0;
        const double next_v = v + dt * a;
        if (!isfinite(next_v)) return 0;
        if (next_v <= 0.0) {
            const double step_a = (next_v - v) / dt;
            const double tau = -v / step_a;
            problem.nominal_apogee_asl_m = h + v * tau + 0.5 * step_a * tau * tau;
            /* Preserve the Python prototype's approximate event sensitivity. */
            memcpy(problem.gradient, sh, sizeof(sh));
            return isfinite(problem.nominal_apogee_asl_m);
        }
        h += dt * v;
        v = next_v;
        if (!isfinite(h)) return 0;
    }
    const double end_u = problem.nominal[MPC_HORIZON - 1];
    double hp, hm, vp, vm, up, um;
    if (!coast_apogee(h, v, end_u, &problem.nominal_apogee_asl_m)
        || !coast_apogee(h + 0.1, v, end_u, &hp)
        || !coast_apogee(h - 0.1, v, end_u, &hm)
        || !coast_apogee(h, v + 0.1, end_u, &vp)
        || !coast_apogee(h, v - 0.1, end_u, &vm)
        || !coast_apogee(h, v, end_u + 1e-3, &up)
        || !coast_apogee(h, v, end_u - 1e-3, &um)) return 0;
    const double gh = (hp - hm) / 0.2, gv = (vp - vm) / 0.2;
    for (int j = 0; j < MPC_HORIZON; ++j)
        problem.gradient[j] = gh * sh[j] + gv * sv[j];
    problem.gradient[MPC_HORIZON - 1] += (up - um) / 2e-3;
    return all_finite(problem.gradient, MPC_HORIZON);
}

static int prepare_problem(const MPCInput *input, double current) {
    for (int j = 0; j < MPC_HORIZON; ++j) {
        const int shifted = j + 1 < MPC_HORIZON ? j + 1 : j;
        problem.nominal[j] = have_solution ? last_solution[shifted] : current;
    }
    if (!nominal_and_gradient(input->altitude_asl_m, input->velocity_up_m_s)) return 0;
    double g[MPC_HORIZON];
    double error = problem.nominal_apogee_asl_m
        - (cfg.target_apogee_agl_m + model.elevation_asl_m);
    for (int j = 0; j < MPC_HORIZON; ++j) {
        error -= problem.gradient[j] * problem.nominal[j];
        g[j] = problem.gradient[j] / cfg.altitude_scale_m;
    }
    error /= cfg.altitude_scale_m;
    int index = 0;
    for (int col = 0; col < MPC_HORIZON; ++col) {
        for (int row = 0; row <= col; ++row) {
            double dtd = 0.0;
            if (row == col) dtd = col == MPC_HORIZON - 1 ? 1.0 : 2.0;
            else if (row + 1 == col) dtd = -1.0;
            problem.p_upper[index++] = 2.0 * (cfg.w_apogee * g[row] * g[col]
                + cfg.w_smooth * dtd + (row == col ? cfg.w_regularization : 0.0));
        }
        problem.q[col] = 2.0 * (cfg.w_apogee * error * g[col]
            - (col == 0 ? cfg.w_smooth * current : 0.0)
            - cfg.w_regularization * problem.nominal[col]);
        const double rate = cfg.max_deployment_rate
            * (col == 0 ? input->control_dt_s : cfg.prediction_dt_s);
        const double previous = col == 0 ? current : 0.0;
        problem.lower[col] = 0.0;
        problem.upper[col] = 1.0;
        problem.lower[MPC_HORIZON + col] = previous - rate;
        problem.upper[MPC_HORIZON + col] = previous + rate;
    }
    return all_finite(problem.p_upper, MPC_P_NNZ) && all_finite(problem.q, MPC_HORIZON)
        && all_finite(problem.lower, MPC_CONSTRAINTS)
        && all_finite(problem.upper, MPC_CONSTRAINTS);
}

const MPCProblem *mpc_last_problem(void) { return problem_valid ? &problem : NULL; }

static MPCStatus finish(MPCOutput *output, MPCStatus status) {
    output->status = status;
    if (status == MPC_SOLVER_ERROR || status == MPC_MODEL_ERROR)
        osqp_cold_start(&airbrake_solver);
    return status;
}

MPCStatus mpc_step(const MPCInput *input, MPCOutput *output) {
    problem_valid = 0;
    if (!output) return MPC_INVALID_INPUT;
    memset(output, 0, sizeof(*output));
    output->nominal_apogee_asl_m = NAN;
    output->primal_residual = NAN;
    output->dual_residual = NAN;
    if (!input) return finish(output, MPC_INVALID_INPUT);
    if (isfinite(input->deployment)) {
        output->command = clamp(input->deployment, 0.0, 1.0);
        output->command_valid = 1;
    }
    if (!initialized) return finish(output, MPC_NOT_INITIALIZED);
    const double values[] = { input->altitude_asl_m, input->velocity_up_m_s,
        input->time_since_launch_s, input->deployment, input->control_dt_s };
    if (!all_finite(values, sizeof(values) / sizeof(values[0]))
        || input->control_dt_s <= 0.0 || input->time_since_launch_s < 0.0
        || (input->motor_burned_out != 0 && input->motor_burned_out != 1))
        return finish(output, MPC_INVALID_INPUT);
    if (!input->motor_burned_out || input->velocity_up_m_s <= cfg.min_control_velocity_m_s)
        return finish(output, MPC_INACTIVE);
    const double current = output->command;
    if (!prepare_problem(input, current)) return finish(output, MPC_MODEL_ERROR);
    problem_valid = 1;
    output->nominal_apogee_asl_m = problem.nominal_apogee_asl_m;
    /* Match Python OSQP.update(): vectors first, then matrices. Matrix updates
     * recompute scaling using q, so reversing these calls changes conditioning. */
    OSQPInt error = osqp_update_data_vec(&airbrake_solver, problem.q,
                                        problem.lower, problem.upper);
    if (!error) error = osqp_update_data_mat(&airbrake_solver, problem.p_upper, NULL,
                                            MPC_P_NNZ, NULL, NULL, 0);
    if (!error && have_solution)
        error = osqp_warm_start(&airbrake_solver, problem.nominal, NULL);
    if (error) {
        output->solver_error = error;
        return finish(output, MPC_SOLVER_ERROR);
    }
    error = osqp_solve(&airbrake_solver);
    output->solver_error = error;
    output->solver_status = airbrake_solver.info->status_val;
    output->iterations = airbrake_solver.info->iter;
    output->primal_residual = airbrake_solver.info->prim_res;
    output->dual_residual = airbrake_solver.info->dual_res;
    if (error || airbrake_solver.info->status_val != OSQP_SOLVED
        || !airbrake_solver.solution || !airbrake_solver.solution->x)
        return finish(output, MPC_SOLVER_ERROR);
    const OSQPFloat *solution = airbrake_solver.solution->x;
    double previous = current;
    for (int j = 0; j < MPC_HORIZON; ++j) {
        const double delta = solution[j] - previous;
        /* Independently check the full plan before accepting it. */
        const double rate = cfg.max_deployment_rate
            * (j == 0 ? input->control_dt_s : cfg.prediction_dt_s);
        if (!isfinite(solution[j]) || solution[j] < -1e-4 || solution[j] > 1.0 + 1e-4
            || fabs(delta) > rate + 1e-4) return finish(output, MPC_SOLVER_ERROR);
        previous = solution[j];
    }
    for (int j = 0; j < MPC_HORIZON; ++j)
        last_solution[j] = clamp(solution[j], 0.0, 1.0);
    have_solution = 1;
    const double rate = cfg.max_deployment_rate * input->control_dt_s;
    output->command = clamp(last_solution[0], fmax(0.0, current - rate),
                                             fmin(1.0, current + rate));
    return finish(output, MPC_OK);
}
