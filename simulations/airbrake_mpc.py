"""
Airbrake Model Predictive Control (OSQP)
========================================

Implements the algorithm in "updated-airbrakes-mpc-overview.pdf" using the same
OSQP closed-loop structure as the quadcopter MPC example.

Decision variable
    alpha = (alpha_0, ..., alpha_{N-1})   airbrake opening (0 = closed, 1 = full)
    held constant over each MPC step of length DT_MPC; after the horizon the last
    value is held until apogee.

Algorithm (per control step)
    1. Take alpha0 = previous plan shifted by one step (the "known" linearization
       point).  Predict apogee h(alpha0) with the flight model.
    2. Finite-difference gradient  grad_i = (h(alpha0 + eps*e_i) - h(alpha0)) / eps
    3. Linearize:  h(alpha) ~= h(alpha0) + grad.(alpha - alpha0)
                   beta = h(alpha0) - grad.alpha0 - h_target
    4. Cost  J = wA (grad.alpha + beta)^2 + alpha^T S alpha
                 + (D alpha - alpha_prev e0)^T Q (D alpha - alpha_prev e0)
       =>  P = 2 (wA grad grad^T + S + D^T Q D)
           q = 2 wA beta grad - 2 alpha_prev D^T Q e0
    5. Constraints  l <= A alpha <= u,   A = [ I ; D ]
         I rows: 0 <= alpha <= alpha_max   (optionally inside a trust region
                 around alpha0, because the Taylor model is only local)
         D rows: |alpha_i - alpha_{i-1}| <= rate_max * DT_MPC
                 (row 0 is relative to the last commanded alpha_prev)
    6. Apply alpha_0, advance the plant, repeat.

Note on D: the PDF's rendered matrix reads like an upper-bidiagonal form, but its
"previous airbrake value" term only makes sense for  (D alpha)_i = alpha_i - alpha_{i-1},
i.e. +1 on the diagonal and -1 on the sub-diagonal, so that is what is used here.
"""

import numpy as np
import scipy.sparse as sparse
import osqp

# ----------------------------------------------------------------------------
# Rocket / atmosphere model  (EDIT THESE FOR YOUR VEHICLE)
# ----------------------------------------------------------------------------
G0 = 9.80665
RHO0 = 1.225            # sea-level density [kg/m^3]
H_SCALE = 8400.0        # exponential atmosphere scale height [m]


class RocketModel:
    """1-D vertical coast model with quadratic drag.

    State: altitude h [m], vertical velocity v [m/s].  Airbrake opening alpha in
    [0, 1] adds drag area  alpha * ab_area  with coefficient ab_cd.
    """

    def __init__(
        self,
        mass=22.0,
        body_area=0.0182,
        body_cd=0.45,
        ab_area=0.012,
        ab_cd=1.2,
        cd_scale=1.0,
    ):

        self.m = mass
        self.A = body_area
        self.cd = body_cd
        self.ab_area = ab_area
        self.ab_cd = ab_cd
        self.cd_scale = cd_scale   # use !=1 to make a "true" plant that differs from the model

    def accel(self, h, v, alpha):
        rho = RHO0 * np.exp(-h / H_SCALE)
        cda = self.cd_scale * (self.cd * self.A + self.ab_cd * self.ab_area * alpha)
        drag = 0.5 * rho * cda * v * abs(v) / self.m
        return -G0 - drag

    def step(self, h, v, alpha, dt, n_sub=10):
        """Advance (h, v) by dt with constant alpha (RK2 sub-stepping)."""
        d = dt / n_sub
        for _ in range(n_sub):
            a1 = self.accel(h, v, alpha)
            hm, vm = h + 0.5 * d * v, v + 0.5 * d * a1
            a2 = self.accel(hm, vm, alpha)
            h, v = h + d * vm, v + d * a2
        return h, v

    def predict_apogee(self, h, v, alpha_plan, dt_mpc, dt_int=0.05):
        """Apogee altitude if alpha_plan (piecewise constant, dt_mpc each) is flown,
        then its last value is held until v <= 0."""
        # Planned portion of the flight
        for a in alpha_plan:
            if v <= 0.0:
                return h
            n_sub = max(1, int(round(dt_mpc / dt_int)))
            h_new, v_new = self.step(h, v, a, dt_mpc, n_sub)
            if v_new <= 0.0:   # apogee inside this step: linear interpolate
                frac = v / (v - v_new)
                return h + frac * (h_new - h)
            h, v = h_new, v_new
        # Hold last value to apogee
        a = alpha_plan[-1]
        while v > 0.0:
            h_new, v_new = self.step(h, v, a, dt_int, 1)
            if v_new <= 0.0:
                frac = v / (v - v_new)
                return h + frac * (h_new - h)
            h, v = h_new, v_new
        return h


# ----------------------------------------------------------------------------
# MPC
# ----------------------------------------------------------------------------
class AirbrakeMPC:
    def __init__(
        self,
        model,
        target_apogee,
        N=15,
        dt_mpc=1.0,
        alpha_max=1.0,
        rate_max=0.5,
        w_apogee=1.0,
        stickout_w=2.0e2,
        smooth_w=1.0e3,
        eps=0.01,
        trust_region=0.3,
        n_iter=2,
    ):

        self.model = model
        self.h_target = target_apogee
        self.N = N
        self.dt = dt_mpc
        self.alpha_max = alpha_max
        self.rate_max = rate_max
        self.wA = w_apogee
        self.eps = eps
        self.trust = trust_region
        self.n_iter = n_iter          # re-linearizations per control step

        # Weight matrices S (stickout) and Q (smoothness), diagonal per time slot
        self.S = sparse.diags(np.full(N, stickout_w)).toarray()
        self.Q = sparse.diags(np.full(N, smooth_w)).toarray()

        # Difference matrix:  (D a)_i = a_i - a_{i-1}
        self.D = (sparse.eye(N) - sparse.eye(N, k=-1)).toarray()
        self.e0 = np.zeros(N)
        self.e0[0] = 1.0
        self.DtQD = self.D.T @ self.Q @ self.D
        self.DtQe0 = self.D.T @ self.Q @ self.e0

        # Constraint matrix  A = [I ; D]
        self.A = sparse.vstack([sparse.eye(N), sparse.csc_matrix(self.D)], format='csc')

        # OSQP is created once with a dense upper-triangular P pattern so that
        # P and q can be updated in place every iteration (warm started).
        pattern = sparse.triu(sparse.csc_matrix(np.ones((N, N))), format='csc')
        self._P_pattern = pattern
        self.prob = osqp.OSQP()
        l0, u0 = np.zeros(2 * N), np.ones(2 * N)
        self.prob.setup(
            pattern,
            np.zeros(N),
            self.A,
            l0,
            u0,
            warm_starting=True,
            verbose=False,
            eps_abs=1e-6,
            eps_rel=1e-6,
            max_iter=10000,
            polish=False,
        )

        self.alpha_plan = np.zeros(N)   # alpha0 for the first linearization
        self.alpha_prev = 0.0           # last commanded value

    # -- gradient by finite differences (PDF "Solution" section) ---------------
    def _linearize(self, h, v, alpha0):
        h0 = self.model.predict_apogee(h, v, alpha0, self.dt)
        grad = np.zeros(self.N)
        for i in range(self.N):
            a = alpha0.copy()
            a[i] += self.eps
            grad[i] = (self.model.predict_apogee(h, v, a, self.dt) - h0) / self.eps
        return h0, grad

    def solve(self, h, v):
        """Return the airbrake command for the current state (h, v)."""
        N = self.N
        alpha0 = self.alpha_plan.copy()
        info = {}

        for _ in range(self.n_iter):
            h0, grad = self._linearize(h, v, alpha0)
            beta = h0 - grad @ alpha0 - self.h_target

            # P = 2 (wA grad grad^T + S + D^T Q D),   q = 2 wA beta grad - 2 a_prev D^T Q e0
            P = 2.0 * (self.wA * np.outer(grad, grad) + self.S + self.DtQD)
            q = 2.0 * self.wA * beta * grad - 2.0 * self.alpha_prev * self.DtQe0

            # Upper-triangle values in CSC order (matches the setup pattern)
            Px = np.concatenate([P[:j + 1, j] for j in range(N)])

            # Position bounds, restricted to a trust region around alpha0
            lo = np.clip(alpha0 - self.trust, 0.0, self.alpha_max)
            hi = np.clip(alpha0 + self.trust, 0.0, self.alpha_max)
            # Rate bounds; row 0 is relative to the last commanded value
            dmax = self.rate_max * self.dt
            rate_lo = np.full(N, -dmax)
            rate_hi = np.full(N, dmax)
            rate_lo[0] += self.alpha_prev
            rate_hi[0] += self.alpha_prev
            # Keep the step-0 box consistent with the rate window (never an empty interval)
            lo[0] = np.clip(max(lo[0], self.alpha_prev - dmax), 0.0, self.alpha_max)
            hi[0] = np.clip(min(hi[0], self.alpha_prev + dmax), 0.0, self.alpha_max)
            if lo[0] > hi[0]:
                lo[0] = hi[0] = np.clip(alpha0[0], self.alpha_prev - dmax, self.alpha_prev + dmax).clip(0.0, self.alpha_max)

            l = np.hstack([lo, rate_lo])
            u = np.hstack([hi, rate_hi])

            self.prob.update(Px=Px, q=q, l=l, u=u)
            res = self.prob.solve()
            if res.info.status not in ('solved', 'solved inaccurate'):
                raise RuntimeError(f'OSQP failed: {res.info.status}')
            alpha0 = np.clip(res.x, 0.0, self.alpha_max)
            info = dict(h_pred_lin=h0, status=res.info.status)

        self.alpha_plan_full = alpha0
        # Shift plan by one step (hold last) as the next linearization point
        self.alpha_plan = np.append(alpha0[1:], alpha0[-1])
        cmd = float(alpha0[0])
        self.alpha_prev = cmd
        info['apogee_pred'] = self.model.predict_apogee(h, v, alpha0, self.dt)
        return cmd, info


# ----------------------------------------------------------------------------
# Closed-loop simulation
# ----------------------------------------------------------------------------
def simulate(h_burnout=900.0, v_burnout=280.0, target=3048.0, plot=True,
             cd_error=1.10, noise_h=1.0, noise_v=0.5, seed=0):
    rng = np.random.default_rng(seed)

    model = RocketModel()                              # what the MPC believes
    plant = RocketModel(cd_scale=cd_error)             # "truth" (drag mismatch)
    mpc = AirbrakeMPC(model, target)

    # No-brake / full-brake apogee bounds for reference
    n = mpc.N
    ap_min = model.predict_apogee(h_burnout, v_burnout, np.zeros(n), mpc.dt)
    ap_max = model.predict_apogee(h_burnout, v_burnout, np.ones(n), mpc.dt)
    print(f'Model apogee, brakes closed: {ap_min:8.1f} m')
    print(f'Model apogee, brakes full:   {ap_max:8.1f} m')
    print(f'Target apogee:               {target:8.1f} m\n')

    h, v, t = h_burnout, v_burnout, 0.0
    log = dict(t=[], h=[], v=[], alpha=[], pred=[])
    print(' t[s]   h[m]    v[m/s]  alpha  predicted apogee[m]')
    while v > 0.0:
        h_meas = h + rng.normal(0, noise_h)
        v_meas = v + rng.normal(0, noise_v)
        cmd, info = mpc.solve(h_meas, v_meas)
        log['t'].append(t); log['h'].append(h); log['v'].append(v)
        log['alpha'].append(cmd); log['pred'].append(info['apogee_pred'])
        print(f'{t:5.1f} {h:8.1f} {v:8.1f}  {cmd:5.3f}  {info["apogee_pred"]:8.1f}')

        # Advance the true plant one MPC step (stop at apogee)
        h_new, v_new = plant.step(h, v, cmd, mpc.dt, 20)
        if v_new <= 0.0:
            frac = v / (v - v_new)
            h = h + frac * (h_new - h)
            t += frac * mpc.dt
            v = 0.0
        else:
            h, v, t = h_new, v_new, t + mpc.dt

    print(f'\nActual apogee: {h:.1f} m   (target {target:.1f} m, error {h - target:+.1f} m)')

    if plot:
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(3, 1, figsize=(7, 8), sharex=True)
            ax[0].plot(log['t'], log['h']); ax[0].set_ylabel('altitude [m]')
            ax[1].plot(log['t'], log['pred']); ax[1].axhline(target, ls='--', c='k')
            ax[1].set_ylabel('predicted apogee [m]')
            ax[2].step(log['t'], log['alpha'], where='post'); ax[2].set_ylabel('airbrake alpha')
            ax[2].set_xlabel('time since burnout [s]')
            fig.tight_layout(); fig.savefig('airbrake_mpc_sim.png', dpi=120)
            print('Saved airbrake_mpc_sim.png')
        except ImportError:
            pass
    return h


if __name__ == '__main__':
    simulate()
