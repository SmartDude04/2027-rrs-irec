/*
 * Airbrake MPC for STM32: C port of airbrake_mpc.py using the OSQP generated
 * solver (embedded mode 2: matrices may be updated, sparsity is fixed).
 *
 * Per control step (see the PDF / Python docstring):
 *   1. alpha0 <- previous plan shifted by one step
 *   2. h0 = apogee(alpha0);  grad_i = (apogee(alpha0 + eps e_i) - h0) / eps
 *   3. beta = h0 - grad.alpha0 - h_target
 *   4. P = 2 (wA grad grad^T + S + D^T Q D),  q = 2 wA beta grad - 2 alpha_prev Q_0 e_0
 *   5. bounds: trust region / [0, max] on alpha, rate limit on D alpha
 *   6. osqp_update_data_mat / osqp_update_data_vec / osqp_solve
 */
#include <math.h>
#include <string.h>

#include "osqp.h"        /* generated OSQP embedded API */
#include "workspace.h"   /* extern OSQPSolver solver;   */
#include "airbrake_mpc.h"

_Static_assert(sizeof(OSQPFloat) == sizeof(float),
               "Generate the solver with use_float=True (OSQPFloat must be float)");

#define NNZ_P ((AB_N * (AB_N + 1)) / 2)   /* dense upper triangle, CSC order */
#define MAX_HOLD_STEPS 4000               /* guard against a runaway hold loop */

static float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

/* ------------------------------------------------------------------ model */

ab_rocket_t ab_default_rocket(void) {
  ab_rocket_t r = { AB_MASS_KG, AB_BODY_AREA_M2, AB_BODY_CD,
                    AB_BRAKE_AREA_M2, AB_BRAKE_CD, 1.0f };
  return r;
}

static float accel(const ab_rocket_t *r, float h, float v, float alpha,
                   uint32_t *evals) {
  float rho = AB_RHO0 * expf(-h / AB_H_SCALE);
  float cda = r->cd_scale * (r->body_cd * r->body_area + r->ab_cd * r->ab_area * alpha);
  float drag = 0.5f * rho * cda * v * fabsf(v) / r->mass;
  if (evals) (*evals)++;
  return -AB_G0 - drag;
}

void ab_rocket_step(const ab_rocket_t *r, float *h, float *v, float alpha,
                    float dt, int n_sub, uint32_t *evals) {
  float d = dt / (float)n_sub;
  float hh = *h, vv = *v;
  for (int k = 0; k < n_sub; k++) {
    float a1 = accel(r, hh, vv, alpha, evals);
    float hm = hh + 0.5f * d * vv;
    float vm = vv + 0.5f * d * a1;
    float a2 = accel(r, hm, vm, alpha, evals);
    hh = hh + d * vm;
    vv = vv + d * a2;
  }
  *h = hh;
  *v = vv;
}

float ab_predict_apogee(const ab_rocket_t *r, float h, float v,
                        const float *plan, uint32_t *evals) {
  for (int i = 0; i < AB_N; i++) {
    if (v <= 0.0f) return h;
    float hn = h, vn = v;
    ab_rocket_step(r, &hn, &vn, plan[i], AB_DT_MPC, AB_SUB_STEPS, evals);
    if (vn <= 0.0f) {                      /* apogee inside this step */
      float frac = v / (v - vn);
      return h + frac * (hn - h);
    }
    h = hn;
    v = vn;
  }
  /* hold the last opening until apogee */
  const float a = plan[AB_N - 1];
  for (int k = 0; k < MAX_HOLD_STEPS && v > 0.0f; k++) {
    float hn = h, vn = v;
    ab_rocket_step(r, &hn, &vn, a, AB_DT_INT, 1, evals);
    if (vn <= 0.0f) {
      float frac = v / (v - vn);
      return h + frac * (hn - h);
    }
    h = hn;
    v = vn;
  }
  return h;
}

/* -------------------------------------------------------------------- MPC */

void airbrake_mpc_init(ab_mpc_t *m, const ab_rocket_t *model, float target_apogee_m) {
  m->model = *model;
  m->target_apogee_m = target_apogee_m;
  memset(m->alpha_plan, 0, sizeof(m->alpha_plan));
  m->alpha_prev = 0.0f;
  m->evals = 0;
  osqp_cold_start(&solver);
}

float airbrake_mpc_step(ab_mpc_t *m, float h, float v, ab_info_t *info) {
  static float grad[AB_N];
  static float alpha_try[AB_N];
  static float Px[NNZ_P];
  static float q[AB_N];
  static float l[2 * AB_N], u[2 * AB_N];

  float alpha0[AB_N];
  memcpy(alpha0, m->alpha_plan, sizeof(alpha0));

  const uint32_t evals_start = m->evals;
  const float dmax = AB_RATE_MAX * AB_DT_MPC;
  int fallback = 0;
  int32_t status = 0, iters = 0;

  for (int it = 0; it < AB_N_ITER; it++) {
    /* ---- 2. finite-difference gradient about alpha0 ---- */
    const float h0 = ab_predict_apogee(&m->model, h, v, alpha0, &m->evals);
    for (int i = 0; i < AB_N; i++) {
      memcpy(alpha_try, alpha0, sizeof(alpha_try));
      alpha_try[i] += AB_EPS;
      grad[i] = (ab_predict_apogee(&m->model, h, v, alpha_try, &m->evals) - h0) / AB_EPS;
    }
    float g_dot_a0 = 0.0f;
    for (int i = 0; i < AB_N; i++) g_dot_a0 += grad[i] * alpha0[i];
    const float beta = h0 - g_dot_a0 - m->target_apogee_m;

    /* ---- 4. P (upper triangle, CSC order) and q ---- */
    int k = 0;
    for (int j = 0; j < AB_N; j++) {
      for (int i = 0; i <= j; i++) {
        float val = AB_W_APOGEE * grad[i] * grad[j];
        if (i == j) {
          val += AB_S_DIAG[j] + AB_Q_DIAG[j] + (j + 1 < AB_N ? AB_Q_DIAG[j + 1] : 0.0f);
        } else if (i == j - 1) {
          val -= AB_Q_DIAG[j];             /* (D^T Q D)_{j-1,j} = -Q_j */
        }
        Px[k++] = 2.0f * val;
      }
    }
    for (int i = 0; i < AB_N; i++) q[i] = 2.0f * AB_W_APOGEE * beta * grad[i];
    q[0] -= 2.0f * m->alpha_prev * AB_Q_DIAG[0];

    /* ---- 5. bounds ---- */
    for (int i = 0; i < AB_N; i++) {
      l[i] = clampf(alpha0[i] - AB_TRUST, 0.0f, AB_ALPHA_MAX);
      u[i] = clampf(alpha0[i] + AB_TRUST, 0.0f, AB_ALPHA_MAX);
      l[AB_N + i] = -dmax;
      u[AB_N + i] = dmax;
    }
    /* row 0 of D*alpha is alpha_0 itself, so its rate window is centred on alpha_prev */
    l[AB_N] += m->alpha_prev;
    u[AB_N] += m->alpha_prev;
    /* keep the step-0 box consistent with the rate window (never empty) */
    l[0] = clampf(fmaxf(l[0], m->alpha_prev - dmax), 0.0f, AB_ALPHA_MAX);
    u[0] = clampf(fminf(u[0], m->alpha_prev + dmax), 0.0f, AB_ALPHA_MAX);
    if (l[0] > u[0]) {
      float a = clampf(alpha0[0], m->alpha_prev - dmax, m->alpha_prev + dmax);
      l[0] = u[0] = clampf(a, 0.0f, AB_ALPHA_MAX);
    }

    /* ---- 6. update + solve (warm started by the workspace) ---- */
    if (osqp_update_data_mat(&solver, Px, OSQP_NULL, NNZ_P, OSQP_NULL, OSQP_NULL, 0) != 0 ||
        osqp_update_data_vec(&solver, q, l, u) != 0) {
      fallback = 1;
      break;
    }
    osqp_solve(&solver);
    status = (int32_t)solver.info->status_val;
    iters  = (int32_t)solver.info->iter;

    const int ok = (status == OSQP_SOLVED || status == OSQP_SOLVED_INACCURATE ||
                    status == OSQP_MAX_ITER_REACHED);
    int finite = 1;
    for (int i = 0; i < AB_N; i++) {
      if (!(solver.solution->x[i] == solver.solution->x[i])) finite = 0;   /* NaN check */
    }
    if (!ok || !finite) {
      fallback = 1;
      break;
    }
    for (int i = 0; i < AB_N; i++) {
      alpha0[i] = clampf(solver.solution->x[i], 0.0f, AB_ALPHA_MAX);
    }
  }

  /* Command: first element, clamped to the actuator limits as a last safeguard */
  float cmd = clampf(alpha0[0], m->alpha_prev - dmax, m->alpha_prev + dmax);
  cmd = clampf(cmd, 0.0f, AB_ALPHA_MAX);
  alpha0[0] = cmd;

  /* Next linearization point: plan shifted by one step, last value held */
  for (int i = 0; i + 1 < AB_N; i++) m->alpha_plan[i] = alpha0[i + 1];
  m->alpha_plan[AB_N - 1] = alpha0[AB_N - 1];
  m->alpha_prev = cmd;

  if (info) {
    info->apogee_pred_m = ab_predict_apogee(&m->model, h, v, alpha0, &m->evals);
    info->solver_status = status;
    info->solver_iters = iters;
    info->used_fallback = fallback;
    info->model_evals = m->evals - evals_start;
  }
  return cmd;
}
