# Embedded apogee MPC

This project ports `simulations/linearmodel.py` and `docs/linear-mpc.md` to C for
the STM32H743VIT6. OSQP 1.1.3 generates the allocation-free QP solver; `src/mpc.c`
implements the nonlinear coast prediction, linearization, terminal derivatives,
condensed QP construction, warm starts, and command selection.

The first milestone is a controller module and a synthetic STM32 bench example.
`src/main.c` does not read sensors or drive a servo. It exposes results and timing
through debugger-visible variables. No hardware execution or timing validation
has been performed as part of the software implementation.

## Build and verify

From the repository root, using PowerShell:

```powershell
# Existing environment, or create it with: cd simulations; uv sync --locked
.\simulations\.venv\Scripts\python.exe -B simulations/codegen.py --check
.\simulations\.venv\Scripts\python.exe -B simulations/test_embedded_mpc.py
pio run -d airbrakes
```

If `pio` is not on PATH, the usual Windows install is:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -d airbrakes
```

Native verification requires GCC on PATH (`CC` can name another GCC-compatible
compiler). It builds the actual C controller and generated solver as a temporary
shared library and compares them with the Python implementation. It verifies
apogee, sensitivities, all QP arrays, commands, full-plan constraints, a 60-tick
replay, reset behavior, invalid data, model failures, and solver failures. It
does not import the forecast environment or download weather data.

The STM32 build selects PlatformIO ststm32 20.0.0, STM32Cube, the DevEBox H743
board, and the internal reset-state HSI clock at **64 MHz**. It enables instruction
and data caches and compiles/links with the Cortex-M7 double-precision FPU ABI.
`scripts/fpu.py` supplies the matching linker flags, which Cube's generic builder
does not inherit from `build_flags`. Changing to a faster clock requires a real
board clock configuration as well as updating `board_build.f_cpu`.

The original two-variable OSQP experiment remains in `src/` and `include/`, with
the old main saved in `main-osqp-example.c.txt`. `build_src_filter` compiles
only `main.c` and `mpc.c` from the project sources. The active solver is the local
`lib/osqp_mpc` library, included as `osqp_mpc/...` to avoid the older headers.

## Controller interface

```c
#include "mpc.h"

/* Supply persistent, launch-specific density data. */
static const double altitude_m[] = {-1000, 0, 1000, 2000, 4000, 8000, 20000};
static const double density_kg_m3[] = {1.35, 1.225, 1.112, 1.007, 0.819, 0.525, 0.089};
static MPCDensityTable atmosphere = {altitude_m, density_kg_m3, 7};

void controller_example(void) {
    MPCConfig config = mpc_default_config();
    MPCModel model = {
        .coast_mass_kg = 24.3209,  /* illustrative: replace with calibrated mass */
        .elevation_asl_m = 270.0,
        .density = mpc_density_from_table,
        .density_context = &atmosphere
    };
    if (mpc_init(&config, &model) != MPC_OK) return;

    MPCInput state = {
        .altitude_asl_m = 1000.0,
        .velocity_up_m_s = 200.0,
        .time_since_launch_s = 8.0,
        .deployment = 0.25,
        .control_dt_s = 0.1,
        .motor_burned_out = 1
    };
    MPCOutput output;
    MPCStatus status = mpc_step(&state, &output);
    if (output.command_valid) {
        /* output.command is the normalized [0,1] command for actuator integration.
         * Log status/output diagnostics; inactive/failure paths hold deployment. */
        (void)status;
    }
}
```

Initialize once, then call `mpc_step` at each controller sample. Configuration and
model structs are copied; the density context and its arrays must remain valid.
The controller and solver use one static workspace: calls must be serialized and
must not run in interrupt context. Call `mpc_reset()` between flights to discard
the previous plan and cold-start the solver; configuration remains intact.

Units are meters, seconds, kilograms, and kg/m³. Altitude input and density-table
altitudes are **ASL**; the target is **AGL** and is converted using model elevation.
Velocity is positive upward. Supply measured deployment if available; otherwise
supply the command currently applied. `control_dt_s` is the actual elapsed sample
interval, which determines the first slew bound. The remaining bounds use the
prediction step. Flight-state logic supplies `motor_burned_out`; powered ascent
and velocity at/below the configured 5 m/s threshold skip optimization.

The table callback uses piecewise-linear density and rejects out-of-range
altitudes with NaN. Include all predicted altitudes, with margin for ±1 m density
derivatives and ±0.1 m terminal differences. Tables must have at least two finite
rows, strictly increasing altitudes, and nonnegative densities. They are checked
at initialization and must remain immutable. Alternatively provide a deterministic
`MPCDensityFunction` and context; return NaN when the model cannot be evaluated.
Mass is constant during coast in this port; there is no powered-flight mass model.

The example table/mass are illustrative bench inputs. A configured RocketPy
environment can supply launch-specific density data before flight:

```python
# environment is your already-configured RocketPy Environment.
import numpy as np
altitudes = np.arange(-1000.0, 20000.1, 100.0)
densities = [float(environment.density(h)) for h in altitudes]
print("static const double altitude_m[] = {" + ",".join(map(str, altitudes)) + "};")
print("static const double density_kg_m3[] = {" + ",".join(map(str, densities)) + "};")
```

## Solver generation and settings

To regenerate after changing the compile-time horizon:

```powershell
.\simulations\.venv\Scripts\python.exe -B simulations/codegen.py
.\simulations\.venv\Scripts\python.exe -B simulations/codegen.py --check
.\simulations\.venv\Scripts\python.exe -B simulations/test_embedded_mpc.py
pio run -d airbrakes
```

The generator reads `MPCConfig.horizon` from `simulations/linearmodel.py`. It
requires OSQP 1.1.3 and emits self-contained C under `lib/osqp_mpc`, including
`airbrake_workspace.c`, a dimensions header, license, and a manifest with source
hashes. Timestamps are normalized so `--check` can verify regeneration. No Python,
NumPy, SciPy, or RocketPy is needed on the MCU or to build the checked-in C sources.
Do not edit generated files directly. The script leaves `simulations/out` and
the older example solver untouched.

Defaults match the linear prototype: N=12, prediction step 0.25 s, target 3000 m
AGL, slew rate 0.2/s, and weights 5/5/0.1. These differ from the separate nonlinear
SLSQP demo, whose target is 914.4 m. Runtime configuration can change the target,
weights, prediction step, drag parameters, and bounds without regenerating.
Changing the horizon requires regeneration and rebuilding. The existing radius
assumption `rocket_area = pi * 0.0762**2` is preserved; calibrate it against the
rocket geometry (`rockets.py` currently specifies a different radius).

The solver stores all 78 upper-triangular P entries and 35 constraint-matrix
entries for the 12-variable, 24-constraint QP. It uses embedded mode 2 (`matrices`),
double precision, 1500 iterations maximum, and absolute/relative tolerances of
1e-5. Rho adaptation uses a fixed 50-iteration interval rather than host timing.
The parity harness gives Python these same settings. C updates vectors before
matrices, matching Python OSQP 1.1.3: matrix updates rescale using the current q.

Only exact `OSQP_SOLVED`, a finite solution, and a separately checked feasible
plan are accepted. The full-plan feasibility allowance is 1e-4 deployment units;
the returned first command is always clamped to the exact range and slew bound.
`solved inaccurate`, iteration limits, and other solver statuses hold deployment.
There are valid but poorly conditioned cases where the current 1500-iteration
budget is exhausted in both Python and C; this port preserves that behavior.

Status values are `MPC_OK` (0), `MPC_INACTIVE` (1), invalid input (-1), invalid
configuration (-2), model/prediction failure (-3), solver failure (-4), and not
initialized (-5). `solver_status`, `solver_error`, iterations, residuals, and nominal
apogee are returned for diagnostics. If the supplied deployment is finite, error
paths return its clipped value and set `command_valid=1`; a nonfinite deployment
or NULL input sets `command_valid=0`, so the caller must not apply that command.
An invalid initialization disables the controller. `mpc_last_problem()` exposes
the most recently prepared QP until the next call/reset, for inspection and replay.

## Board validation still needed

`main.c` provides `mpc_example_output`, `mpc_example_cycles`,
`mpc_example_elapsed_ms`, and `mpc_example_overruns` for debugger inspection. DWT
cycles measure the controller call, and the millisecond counter flags calls at
or above the nominal 100 ms control period. Confirm the cycle counter advances on
your board and record worst-case execution time over relevant ascent conditions.
Iteration limits bound the solver, and `max_coast_steps` is restricted to 1..2000
per coast rollout; the nominal terminal computation can require seven rollouts.
These work limits do not by themselves prove a 100 ms deadline.

The predictor preserves the prototype's Euler integration, hold-last-deployment
terminal policy, and approximate gradient when apogee occurs inside the horizon.
It does not model actuator dynamics or provide a flight-state safety policy.
Use calibrated atmosphere/mass/drag inputs, replay representative flight states,
and perform hardware-in-the-loop timing and actuator integration before flight.
