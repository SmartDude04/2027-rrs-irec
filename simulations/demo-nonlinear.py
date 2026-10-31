import numpy as np
from scipy.optimize import minimize, Bounds, LinearConstraint

TARGET_AGL = 914.4
FULL_AIRBRAKE_AREA = 0.060 * 0.040  # total exposed area at full deployment, m^2
ROCKET_AREA = np.pi * 0.0762**2  # assumes 0.0762 m is the RADIUS
MAX_DEPLOYMENT_RATE = 0.2  # deployment fraction / second
SAMPLING_RATE = 10.0  # Hz; benchmark optimizer runtime before increasing
HORIZON = 8
PLAN_DT = 0.5  # seconds per future control interval
INTEGRATION_DT = 0.1
MAX_PREDICTION_TIME = 120.0
W_APOGEE = 1.0  # objective uses squared altitude error in meters
W_DEPLOYMENT = 5.0  # penalizes deployment changes, not deployment itself


def get_cd_rocket(v):
    return 0.3  # replace with a calibrated coast Cd curve


def get_cd_airbrakes(deployment, mach):
    return 0.3 * deployment  # callable also passed to RocketPy


def next_velocity(h, v, t, dt, deployment, environment, rocket):
    rho = float(environment.density(h))  # kg/m^3, NOT pressure in Pa
    mass = float(rocket.total_mass(t))
    mach = abs(v) / float(environment.speed_of_sound(h))
    cda = (get_cd_rocket(v) * ROCKET_AREA + get_cd_airbrakes(deployment, mach) * FULL_AIRBRAKE_AREA)
    acceleration = (-float(environment.gravity(h)) - 0.5 * rho * cda * v * abs(v) / mass)
    return v + acceleration * dt


def predict_apogee(hi, vi, t, alpha, durations, environment, rocket):
    """Predict ASL apogee; hold final control until apogee after horizon.

    Each planned deployment is held over its interval. Actual actuator slew
    is approximated by constraints between successive interval commands.
    """
    h, v, elapsed = float(hi), float(vi), 0.0
    if v <= 0:
        return h
    alpha = np.asarray(alpha, dtype=float)
    boundaries = np.cumsum(durations)
    i = 0
    while elapsed < MAX_PREDICTION_TIME:
        while i < len(alpha) - 1 and elapsed >= boundaries[i] - 1e-10:
            i += 1
        dt = min(INTEGRATION_DT, MAX_PREDICTION_TIME - elapsed)
        if elapsed < boundaries[i] - 1e-10:
            dt = min(dt, boundaries[i] - elapsed)
        vn = next_velocity(h, v, t + elapsed, dt, alpha[i], environment, rocket)
        a = (vn - v) / dt
        if vn <= 0:
            tau = -v / a
            return h + v * tau + 0.5 * a * tau**2
        h += 0.5 * (v + vn) * dt
        v = vn
        elapsed += dt
    raise RuntimeError("Prediction did not reach apogee within time limit")


def cost_function(alpha, hi, vi, t, previous_deployment, durations,
                  environment, rocket):
    apogee = predict_apogee(hi, vi, t, alpha, durations, environment, rocket)
    error = apogee - (TARGET_AGL + float(environment.elevation))
    changes = np.diff(np.r_[previous_deployment, alpha])
    return float(W_APOGEE * error**2 + W_DEPLOYMENT * changes @ changes)


def optimize_deployments(hi, vi, t, previous, sampling_rate, environment, rocket):
    # First move is applied now, one callback interval after previous command.
    # Remaining moves are separated by PLAN_DT.
    intervals = np.r_[1.0 / sampling_rate, np.full(HORIZON - 1, PLAN_DT)]
    limits = MAX_DEPLOYMENT_RATE * intervals
    difference = np.eye(HORIZON) - np.eye(HORIZON, k=-1)
    offset = np.r_[previous, np.zeros(HORIZON - 1)]
    constraint = LinearConstraint(difference, offset - limits, offset + limits)
    initial = np.full(HORIZON, previous)  # always feasible
    result = minimize(
        cost_function, initial,
        args=(hi, vi, t, previous, intervals, environment, rocket),
        method="SLSQP", bounds=Bounds(0.0, 1.0), constraints=[constraint],
        options={"maxiter":50, "ftol":1e-4, "eps":1e-4},
    )
    feasible = (np.all(np.isfinite(result.x))
                and np.all(result.x >= -1e-7)
                and np.all(result.x <= 1 + 1e-7)
                and np.all(np.abs(difference @ result.x - offset) <= limits + 1e-7))
    plan = np.clip(result.x, 0, 1) if result.success and feasible else initial
    return plan, bool(result.success and feasible)


def make_controller(rocket, burnout_time):
    def controller_function(
        time,
        sampling_rate,
        state,
        state_history,
        observed_variables,
        air_brakes,
        sensors,
        environment,
    ):
        if sampling_rate is None or sampling_rate <= 0:
            raise ValueError("This controller requires a positive discrete sampling rate")

        h, vz = float(state[2]), float(state[5])
        previous = float(air_brakes.deployment_level)
        max_change = MAX_DEPLOYMENT_RATE / sampling_rate
        predicted, success = float("nan"), False

        if time <= burnout_time or vz <= 0:
            command = max(0.0, previous - max_change)
        else:
            plan, success = optimize_deployments(h, vz, time, previous, sampling_rate, environment, rocket)
            command = float(np.clip(plan[0], max(0.0, previous - max_change), min(1.0, previous + max_change)))
            intervals = np.r_[1 / sampling_rate, np.full(HORIZON - 1, PLAN_DT)]
            predicted = predict_apogee(h, vz, time, plan, intervals, environment, rocket)

        air_brakes.deployment_level = command
        wx = float(environment.wind_velocity_x(h))
        wy = float(environment.wind_velocity_y(h))
        speed = np.sqrt((wx - state[3])**2 + (wy - state[4])**2 + vz**2)
        mach = speed / float(environment.speed_of_sound(h))

        return (
            time,
            command,
            float(air_brakes.drag_coefficient(command, mach)),
            predicted,
            success,
        )

    return controller_function


if __name__ == "__main__":
    from rocketpy import Flight
    from envs import envURRG
    from motors import L1420R
    from rockets import subscale

    print("===========================================================================")
    print("===== Nominal Flight                                                  =====")
    print("===========================================================================")

    flight_base = Flight(
        rocket=subscale,
        environment=envURRG,
        rail_length=5.2,
        inclination=90,
        heading=0,
        time_overshoot=False
    )
    flight_base.prints.initial_conditions()
    flight_base.prints.burn_out_conditions()
    flight_base.prints.apogee_conditions()
    flight_base.prints.maximum_values()

    print("===========================================================================")
    print()
    print("===========================================================================")
    print("===== Under Air Brakes                                                =====")
    print("===========================================================================")

    subscale.add_air_brakes(
        drag_coefficient_curve=get_cd_airbrakes,
        controller_function=make_controller(subscale, L1420R.burn_out_time),
        sampling_rate=SAMPLING_RATE,
        reference_area=FULL_AIRBRAKE_AREA,
        clamp=True,
        override_rocket_drag=False,
    )
    flight_brakes = Flight(
        rocket=subscale,
        environment=envURRG,
        rail_length=5.2,
        inclination=90,
        heading=0,
        time_overshoot=False
    )
    flight_brakes.prints.initial_conditions()
    flight_brakes.prints.burn_out_conditions()
    flight_brakes.prints.apogee_conditions()
    flight_brakes.prints.maximum_values()

    print()

    print(f"Target apogee: {TARGET_AGL:.2f} m AGL")
    print(f"Actual apogee: {flight_brakes.apogee - envURRG.elevation:.2f} m AGL")

    print("===========================================================================")
