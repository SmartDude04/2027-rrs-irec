"""Successive-linearization apogee MPC for RocketPy, solved with OSQP.

Simulation prototype only; not validated for flight or real-time deployment.
Requires numpy, scipy (sparse matrices), osqp, and a RocketPy Rocket/Environment.
"""
from dataclasses import dataclass
import numpy as np
from scipy import sparse
import osqp


@dataclass
class MPCConfig:
    target_apogee_agl: float = 3000.0
    horizon: int = 12
    prediction_dt: float = 0.25
    max_deployment_rate: float = 0.2
    rocket_cd: float = 0.3
    airbrake_cd: float = 0.3
    rocket_area: float = np.pi * 0.0762**2
    airbrake_area: float = 0.060 * 0.040
    gravity: float = 9.81
    altitude_scale: float = 100.0
    w_apogee: float = 5.0
    w_smooth: float = 5.0
    w_regularization: float = 0.1
    max_coast_steps: int = 2000
    min_control_velocity: float = 5.0


def airbrake_drag_curve(deployment: float, mach: float, cd: float = 0.3) -> float:
    """Additional airbrake Cd using airbrake_area as RocketPy reference area."""
    return cd * float(np.clip(deployment, 0.0, 1.0))


class ApogeeMPC:
    """Condensed linearized-dynamics MPC; reuse an OSQP workspace."""

    def __init__(self, rocket, environment, config=None):
        self.rocket = rocket
        self.environment = environment
        self.cfg = config or MPCConfig()
        c = self.cfg
        if c.horizon < 1 or c.prediction_dt <= 0 or c.altitude_scale <= 0:
            raise ValueError('Invalid MPC horizon, prediction timestep, or scale')
        self.N = c.horizon
        self.D = np.eye(self.N)
        self.D[1:, :-1] -= np.eye(self.N - 1)
        self.constraints = sparse.vstack([
            sparse.eye(self.N, format='csc'), sparse.csc_matrix(self.D)
        ], format='csc')
        # Dense upper-triangle storage keeps the OSQP P sparsity pattern fixed.
        rows, cols = np.triu_indices(self.N)
        self._p_rows, self._p_cols = rows, cols
        self._P_pattern = sparse.csc_matrix(
            (np.ones(len(rows)), (rows, cols)), shape=(self.N, self.N)
        )
        self.solver = None
        self.last_solution = None
        self.last_info = {}

    def _mass(self, t):
        m = float(self.rocket.total_mass(t))
        if not np.isfinite(m) or m <= 0:
            raise ValueError('Rocket mass must be positive and finite')
        return m

    def _rho(self, h):
        return float(self.environment.density(h))

    def _rho_prime(self, h):
        # Derivative of RocketPy atmospheric density with respect to ASL altitude.
        dh = 1.0
        return (self._rho(h + dh) - self._rho(h - dh)) / (2 * dh)

    def _f(self, h, v, t, alpha):
        c = self.cfg
        rho = self._rho(h)
        cda = c.rocket_cd * c.rocket_area + c.airbrake_cd * c.airbrake_area * alpha
        return np.array([v, -c.gravity - rho * cda * v * abs(v) / (2 * self._mass(t))])

    def next_height(self, h, v, dt):
        return h + v * dt

    def next_velocity(self, h, v, t, dt, deployment):
        return v + dt * self._f(h, v, t, deployment)[1]

    def _step(self, x, t, u, dt):
        return x + dt * self._f(x[0], x[1], t, u)

    def _jacobians(self, x, t, u, dt):
        c = self.cfg
        h, v = x
        rho, rho_h, m = self._rho(h), self._rho_prime(h), self._mass(t)
        cda = c.rocket_cd * c.rocket_area + c.airbrake_cd * c.airbrake_area * u
        Ac = np.array([
            [0., 1.],
            [-rho_h * cda * v * abs(v) / (2 * m),
             -rho * cda * abs(v) / m],
        ])
        Bc = np.array([0., -rho * c.airbrake_cd * c.airbrake_area * v * abs(v) / (2 * m)])
        return np.eye(2) + dt * Ac, dt * Bc

    def coast_apogee(self, h, v, t, alpha):
        """Nonlinear terminal-to-apogee rollout, holding alpha constant."""
        dt = self.cfg.prediction_dt
        if v <= 0:
            return float(h)
        x = np.array([h, v], dtype=float)
        for _ in range(self.cfg.max_coast_steps):
            nxt = self._step(x, t, alpha, dt)
            if nxt[1] <= 0:
                # Interpolate time to v=0 with constant step acceleration.
                a = (nxt[1] - x[1]) / dt
                tau = -x[1] / a
                return float(x[0] + x[1] * tau + 0.5 * a * tau * tau)
            x = nxt
            t += dt
        raise RuntimeError('Coast rollout failed to reach apogee')

    def _nominal_and_gradient(self, h, v, t, alpha):
        """Propagate nonlinear nominal trajectory and linearized state sensitivities."""
        dt = self.cfg.prediction_dt
        x = np.array([h, v], dtype=float)
        S = np.zeros((2, self.N))
        for k in range(self.N):
            if x[1] <= 0:
                # Stop at predicted apogee. At this nonsmooth boundary,
                # the local gradient is approximate; disable control near apogee.
                break
            A, B = self._jacobians(x, t, alpha[k], dt)
            nxt = self._step(x, t, alpha[k], dt)
            S = A @ S
            S[:, k] += B
            if nxt[1] <= 0:
                # Interpolate apogee altitude within last prediction interval.
                a = (nxt[1] - x[1]) / dt
                tau = -x[1] / a
                apogee = x[0] + x[1] * tau + 0.5 * a * tau**2
                # Near this event the sensitivity is approximated using
                # the end-of-step altitude row; not an event-time derivative.
                return float(apogee), S[0].copy()
            x = nxt
            t += dt

        # Terminal coast model. Differentiate w.r.t. terminal (h,v) and
        # held deployment, then chain through state sensitivity matrix S.
        u_end = float(alpha[-1])
        phi = self.coast_apogee(x[0], x[1], t, u_end)
        dh, dv, du = 0.1, 0.1, 1e-3
        gh = (self.coast_apogee(x[0] + dh, x[1], t, u_end)
              - self.coast_apogee(x[0] - dh, x[1], t, u_end)) / (2 * dh)
        gv = (self.coast_apogee(x[0], x[1] + dv, t, u_end)
              - self.coast_apogee(x[0], x[1] - dv, t, u_end)) / (2 * dv)
        gu = (self.coast_apogee(x[0], x[1], t, u_end + du)
              - self.coast_apogee(x[0], x[1], t, u_end - du)) / (2 * du)
        grad = np.array([gh, gv]) @ S
        grad[-1] += gu
        return phi, grad

    def solve(self, h_asl, v_up, time, current_deployment, sampling_rate=10.0):
        """Return (first_deployment, diagnostics). Uses one QP per controller tick."""
        c, n = self.cfg, self.N
        current_deployment = float(np.clip(current_deployment, 0, 1))
        if v_up <= c.min_control_velocity:
            return current_deployment, {'status': 'inactive: low ascent velocity'}
        if sampling_rate is None or sampling_rate <= 0:
            raise ValueError('A positive discrete controller sampling_rate is required')

        # Shift the previous solution for a warm-started nominal trajectory.
        if self.last_solution is None:
            nominal = np.full(n, current_deployment)
        else:
            nominal = np.r_[self.last_solution[1:], self.last_solution[-1]]
            nominal = np.clip(nominal, 0, 1)

        predicted_apogee, grad = self._nominal_and_gradient(
            h_asl, v_up, time, nominal
        )
        target_asl = c.target_apogee_agl + float(self.environment.elevation)
        G = grad / c.altitude_scale
        # Approximate error = e0 + G @ U.
        e0 = (predicted_apogee - target_asl - grad @ nominal) / c.altitude_scale
        previous = np.zeros(n)
        previous[0] = current_deployment
        P = 2 * (c.w_apogee * np.outer(G, G)
                 + c.w_smooth * (self.D.T @ self.D)
                 + c.w_regularization * np.eye(n))
        q = 2 * (c.w_apogee * e0 * G
                 - c.w_smooth * self.D.T @ previous
                 - c.w_regularization * nominal)

        rates = np.full(n, c.max_deployment_rate * c.prediction_dt)
        rates[0] = c.max_deployment_rate / sampling_rate
        lower = np.r_[np.zeros(n), previous - rates]
        upper = np.r_[np.ones(n), previous + rates]
        # CSC upper triangle in column-major order, matching OSQP Px.
        if self.solver is None:
            self.solver = osqp.OSQP()
            self.solver.setup(P=self._P_pattern, q=q, A=self.constraints,
                              l=lower, u=upper, verbose=False,
                              warm_starting=True, max_iter=1500,
                              eps_abs=1e-5, eps_rel=1e-5)
        # The dense upper triangular pattern stays constant across updates.
        self.solver.update(Px=P[self._p_rows, self._p_cols][np.lexsort((self._p_rows, self._p_cols))],
                           q=q, l=lower, u=upper)
        if self.last_solution is not None:
            self.solver.warm_start(x=nominal)
        result = self.solver.solve()
        status = result.info.status.lower()
        success = status == 'solved' and result.x is not None and np.all(np.isfinite(result.x))
        if success:
            self.last_solution = np.clip(result.x.copy(), 0, 1)
            command = float(self.last_solution[0])
        else:
            command = current_deployment  # Simulation fallback; flight safety requires review.
        command = float(np.clip(command,
                                max(0., current_deployment - rates[0]),
                                min(1., current_deployment + rates[0])))
        self.last_info = {'status': result.info.status,
                          'nominal_apogee_asl': predicted_apogee,
                          'target_apogee_asl': target_asl,
                          'iterations': result.info.iter,
                          'command': command}
        return command, self.last_info


def make_rocketpy_controller(mpc, burnout_time):
    """Create a RocketPy-compatible callback with persistent MPC workspace."""
    def controller_function(time, sampling_rate, state, state_history,
                            observed_variables, air_brakes, sensors, environment):
        h = float(state[2])  # ASL, not AGL
        vx, vy, vz = map(float, state[3:6])
        current = float(air_brakes.deployment_level)
        if time >= burnout_time and vz > mpc.cfg.min_control_velocity:
            try:
                command, _ = mpc.solve(h, vz, time, current, sampling_rate)
            except (ValueError, RuntimeError, FloatingPointError):
                command = current
            air_brakes.deployment_level = command
        wind_x = environment.wind_velocity_x(h)
        wind_y = environment.wind_velocity_y(h)
        relative_speed = np.sqrt((vx - wind_x)**2 + (vy - wind_y)**2 + vz**2)
        mach = relative_speed / environment.speed_of_sound(h)
        return (time, air_brakes.deployment_level,
                air_brakes.drag_coefficient(air_brakes.deployment_level, mach))
    return controller_function


# Example integration (place in your existing RocketPy script):
# from rocketpy import Flight
# from rocketpy_osqp_mpc import MPCConfig, ApogeeMPC, make_rocketpy_controller, airbrake_drag_curve
# cfg = MPCConfig(target_apogee_agl=3000.0)
# mpc = ApogeeMPC(subscale, envURRG, cfg)
# brakes = subscale.add_air_brakes(
#     drag_coefficient_curve=airbrake_drag_curve,
#     controller_function=make_rocketpy_controller(mpc, L1420R.burn_out_time),
#     sampling_rate=10, reference_area=cfg.airbrake_area,
#     clamp=True, override_rocket_drag=False, name='OSQP MPC Airbrakes')
# flight = Flight(rocket=subscale, environment=envURRG, rail_length=5.2,
#                 inclination=90, heading=0, time_overshoot=False,
#                 terminate_on_apogee=True)
