"""Compile and verify the actual C controller/generated OSQP on the host.

Run: .venv/Scripts/python.exe -B test_embedded_mpc.py (or python on Unix).
Requires a native GCC-compatible compiler on PATH, or the CC environment variable.
No board, forecast download, RocketPy environment, or Python extension build needed.
"""
import ctypes as ct
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

from codegen import make_solver
from linearmodel import ApogeeMPC, MPCConfig

ROOT = Path(__file__).resolve().parents[1]
AIRBRAKES = ROOT / "airbrakes"
GENERATED = AIRBRAKES / "lib/osqp_mpc/src/osqp_mpc"
META = json.loads((AIRBRAKES / "lib/osqp_mpc/generation.json").read_text())
N = META["horizon"]
D = ct.c_double
I = ct.c_int


class Config(ct.Structure):
    _fields_ = [(name, D) for name in (
        "target_apogee_agl_m", "prediction_dt_s", "max_deployment_rate",
        "rocket_cd", "airbrake_cd", "rocket_area_m2", "airbrake_area_m2",
        "gravity_m_s2", "altitude_scale_m", "w_apogee", "w_smooth", "w_regularization"
    )] + [("max_coast_steps", ct.c_uint), ("min_control_velocity_m_s", D)]


class Model(ct.Structure):
    _fields_ = [("coast_mass_kg", D), ("elevation_asl_m", D),
                ("density", ct.c_void_p), ("density_context", ct.c_void_p)]


class Table(ct.Structure):
    _fields_ = [("altitude_asl_m", ct.POINTER(D)),
                ("density_kg_m3", ct.POINTER(D)), ("count", ct.c_size_t)]


class Input(ct.Structure):
    _fields_ = [(name, D) for name in (
        "altitude_asl_m", "velocity_up_m_s", "time_since_launch_s", "deployment", "control_dt_s"
    )] + [("motor_burned_out", I)]


class Output(ct.Structure):
    _fields_ = [("command", D), ("command_valid", I), ("status", I),
                ("solver_status", I), ("solver_error", I), ("iterations", I),
                ("nominal_apogee_asl_m", D), ("primal_residual", D), ("dual_residual", D)]


class Problem(ct.Structure):
    _fields_ = [("nominal", D * N), ("gradient", D * N),
                ("p_upper", D * META["p_nnz"]), ("q", D * N),
                ("lower", D * (2 * N)), ("upper", D * (2 * N)),
                ("nominal_apogee_asl_m", D)]


ALTITUDES = np.array([-1000, 0, 1000, 2000, 4000, 8000, 20000], dtype=float)
DENSITIES = np.array([1.35, 1.225, 1.112, 1.007, 0.819, 0.525, 0.089], dtype=float)
CONFIG_NAMES = {
    "target_apogee_agl": "target_apogee_agl_m", "prediction_dt": "prediction_dt_s",
    "rocket_area": "rocket_area_m2", "airbrake_area": "airbrake_area_m2",
    "gravity": "gravity_m_s2", "altitude_scale": "altitude_scale_m",
    "min_control_velocity": "min_control_velocity_m_s"
}


class Environment:
    elevation = 270.0

    @staticmethod
    def density(h):
        return float(np.interp(h, ALTITUDES, DENSITIES, left=np.nan, right=np.nan))


class Rocket:
    def __init__(self, mass=24.3209):
        self.mass = mass

    def total_mass(self, _time):
        return self.mass


class RecordingSolver:
    def __init__(self, config):
        self.solver, _, _ = make_solver(config)
        self.data = {}

    def update(self, **data):
        self.data = {name: value.copy() for name, value in data.items()}
        self.solver.update(**data)

    def warm_start(self, **kwargs):
        self.solver.warm_start(**kwargs)

    def solve(self):
        return self.solver.solve(raise_error=False)


class EmbeddedMPCTest(unittest.TestCase):
    max_command_error = 0.0
    max_apogee_error = 0.0

    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="mpc-native-")
        cls.addClassCleanup(cls.temp.cleanup)
        library = Path(cls.temp.name) / ("mpc.dll" if sys.platform == "win32" else "mpc.so")
        compiler = os.environ.get("CC", "gcc")
        includes = ["-I" + str(AIRBRAKES / "include"), "-I" + str(GENERATED.parent)]
        # Apply strict warnings to handwritten code; upstream OSQP is unmodified.
        subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wpedantic",
                        "-fsyntax-only", *includes, str(AIRBRAKES / "src/mpc.c"),
                        str(AIRBRAKES / "test/native_hooks.c")], check=True)
        command = [compiler, "-std=c11", "-O2", "-shared", "-fPIC", "-Wall", *includes,
                   str(AIRBRAKES / "src/mpc.c"), str(AIRBRAKES / "test/native_hooks.c"),
                   *map(str, sorted(GENERATED.glob("*.c"))), "-o", str(library), "-lm"]
        subprocess.run(command, check=True)
        cls.lib = ct.CDLL(str(library))
        if sys.platform == "win32":
            import _ctypes
            cls.addClassCleanup(lambda: _ctypes.FreeLibrary(cls.lib._handle))
        cls.lib.mpc_default_config.restype = Config
        cls.lib.mpc_init.argtypes = [ct.POINTER(Config), ct.POINTER(Model)]
        cls.lib.mpc_init.restype = I
        cls.lib.mpc_reset.argtypes = []
        cls.lib.mpc_reset.restype = None
        cls.lib.mpc_step.argtypes = [ct.POINTER(Input), ct.POINTER(Output)]
        cls.lib.mpc_step.restype = I
        cls.lib.mpc_last_problem.argtypes = []
        cls.lib.mpc_last_problem.restype = ct.POINTER(Problem)
        cls.lib.mpc_density_from_table.argtypes = [D, ct.c_void_p]
        cls.lib.mpc_density_from_table.restype = D
        cls.lib.mpc_density_table_valid.argtypes = [ct.POINTER(Table)]
        cls.lib.mpc_density_table_valid.restype = I
        cls.lib.test_set_max_iter.argtypes = [I]
        cls.lib.test_set_max_iter.restype = I
        cls.lib.test_solution.argtypes = [I]
        cls.lib.test_solution.restype = D
        cls.lib.test_structure_size.argtypes = [I]
        cls.lib.test_structure_size.restype = ct.c_size_t
        for index, structure in enumerate((Config, Model, Table, Input, Output, Problem)):
            if cls.lib.test_structure_size(index) != ct.sizeof(structure):
                raise RuntimeError(f"Host ABI mismatch for {structure.__name__}")

    @classmethod
    def tearDownClass(cls):
        print(f"Maximum command error: {cls.max_command_error:.3g}; "
              f"maximum apogee error: {cls.max_apogee_error:.3g} m")

    def setUp(self):
        self.config = self.lib.mpc_default_config()
        self.heights = (D * len(ALTITUDES))(*ALTITUDES)
        self.densities = (D * len(DENSITIES))(*DENSITIES)
        self.table = Table(self.heights, self.densities, len(ALTITUDES))
        self.model = Model(24.3209, 270.0,
                           ct.cast(self.lib.mpc_density_from_table, ct.c_void_p),
                           ct.cast(ct.pointer(self.table), ct.c_void_p))
        self.assertEqual(self.lib.test_set_max_iter(META["max_iter"]), 0)
        self.initialize()

    def initialize(self):
        self.assertEqual(self.lib.mpc_init(ct.byref(self.config), ct.byref(self.model)), 0)

    def step(self, h=1000.0, v=200.0, deployment=0.25, dt=0.1, burned_out=1, time=8.0):
        sample = Input(h, v, time, deployment, dt, burned_out)
        output = Output()
        result = self.lib.mpc_step(ct.byref(sample), ct.byref(output))
        self.assertEqual(result, output.status)
        return result, output

    def reference(self):
        config = MPCConfig(**{name: getattr(self.config, CONFIG_NAMES.get(name, name))
                              for name in MPCConfig.__dataclass_fields__ if name != "horizon"})
        reference = ApogeeMPC(Rocket(self.model.coast_mass_kg), Environment(), config)
        reference.solver = RecordingSolver(config)
        return reference

    def compare_tick(self, reference, h, v, current, dt=0.1, time=8.0):
        expected, info = reference.solve(h, v, time, current, 1.0 / dt)
        status, output = self.step(h, v, current, dt, time=time)
        self.assertEqual(info["status"], "solved")
        self.assertEqual(status, 0, (status, output.solver_status, output.iterations))
        self.assertEqual(output.command_valid, 1)
        command_error = abs(output.command - expected)
        type(self).max_command_error = max(self.max_command_error, command_error)
        self.assertLess(command_error, 2e-4)
        rate = self.config.max_deployment_rate * dt
        self.assertGreaterEqual(output.command, max(0.0, current - rate) - 1e-12)
        self.assertLessEqual(output.command, min(1.0, current + rate) + 1e-12)
        problem = self.lib.mpc_last_problem().contents
        # Compare the C nominal trajectory to Python with the SAME nominal plan:
        # solver roundoff in earlier plans must not mask model/QP assembly errors.
        nominal = np.array(problem.nominal)
        apogee, gradient = reference._nominal_and_gradient(h, v, time, nominal)
        apogee_error = abs(apogee - problem.nominal_apogee_asl_m)
        type(self).max_apogee_error = max(self.max_apogee_error, apogee_error)
        self.assertLess(apogee_error, 1e-8)
        np.testing.assert_allclose(problem.gradient, gradient, rtol=2e-7, atol=2e-7)
        # Compare every tick to arrays assembled by the unmodified Python solve().
        for c_name, python_name in (("p_upper", "Px"), ("q", "q"),
                                    ("lower", "l"), ("upper", "u")):
            np.testing.assert_allclose(getattr(problem, c_name),
                                       reference.solver.data[python_name],
                                       rtol=2e-7, atol=2e-7)
        plan = np.array([self.lib.test_solution(i) for i in range(N)])
        a = reference.constraints @ plan
        self.assertTrue(np.all(a >= np.array(problem.lower) - 1e-4))
        self.assertTrue(np.all(a <= np.array(problem.upper) + 1e-4))
        return expected

    def test_defaults_match_python(self):
        reference = MPCConfig()
        self.assertEqual(reference.horizon, N)
        for name in MPCConfig.__dataclass_fields__:
            if name != "horizon":
                self.assertAlmostEqual(getattr(reference, name),
                                       getattr(self.config, CONFIG_NAMES.get(name, name)), places=14)

    def test_python_c_parity_across_ascent_states(self):
        cases = [(1000, 200, 3000, .25, .1, 24.3209),
                 (1000, 280, 914.4, .25, .1, 24.3209),
                 (2500, 80, 3000, .5, .033, 24.3209),
                 (1500, 25, 1500, .8, .2, 24.3209),
                 (1500, 6, 1500, .5, .1, 24.3209),
                 (4000, 250, 3000, 0, .1, 18),
                 (1000, 180, 914.4, 1, .25, 30)]
        for h, v, target, current, dt, mass in cases:
            with self.subTest(h=h, v=v, target=target, current=current):
                self.config.target_apogee_agl_m = target
                self.model.coast_mass_kg = mass
                self.initialize()
                self.compare_tick(self.reference(), h, v, current, dt)

    def test_receding_horizon_replay(self):
        self.config.target_apogee_agl_m = 914.4
        self.initialize()
        reference = self.reference()
        current = 0.0
        for tick in range(60):
            current = self.compare_tick(reference, 1000 + 20 * tick,
                                        280 - 2.5 * tick, current, time=8 + .1 * tick)

    def test_python_c_solver_failure_parity(self):
        # The reference also exhausts 1500 iterations at this large apogee error.
        self.model.coast_mass_kg = 18
        self.initialize()
        expected, info = self.reference().solve(6000, 350, 8, 0, 10)
        self.assertEqual(info["status"], "maximum iterations reached")
        status, output = self.step(h=6000, v=350, deployment=0)
        self.assertEqual(status, -4)
        self.assertEqual(output.command, expected)
        self.assertEqual(output.solver_status, 7)

    def test_inactive_before_burnout_and_near_apogee(self):
        for v, burnout in ((200, 0), (5, 1), (0, 1), (-10, 1)):
            status, output = self.step(v=v, burned_out=burnout)
            self.assertEqual(status, 1)
            self.assertEqual(output.command, .25)
            self.assertEqual(output.command_valid, 1)
            self.assertEqual(output.iterations, 0)
            self.assertFalse(self.lib.mpc_last_problem())

    def test_invalid_sensor_data_and_command_validity(self):
        for kwargs in ({"h": math.nan}, {"v": math.inf}, {"dt": 0},
                       {"dt": -1}, {"time": -1}, {"burned_out": 2}):
            status, output = self.step(**kwargs)
            self.assertEqual(status, -1)
            self.assertEqual(output.command, .25)
            self.assertEqual(output.command_valid, 1)
        status, output = self.step(deployment=math.nan)
        self.assertEqual(status, -1)
        self.assertEqual(output.command_valid, 0)
        self.assertEqual(self.lib.mpc_step(None, ct.byref(output)), -1)
        self.assertEqual(output.command_valid, 0)
        sample = Input(1000, 200, 8, .25, .1, 1)
        self.assertEqual(self.lib.mpc_step(ct.byref(sample), None), -1)

    def test_invalid_configuration_disables_controller(self):
        for name, value in (("prediction_dt_s", 0), ("w_regularization", 0),
                            ("w_apogee", -1), ("gravity_m_s2", math.nan),
                            ("altitude_scale_m", 0), ("max_coast_steps", 2001)):
            bad = self.lib.mpc_default_config()
            setattr(bad, name, value)
            self.assertEqual(self.lib.mpc_init(ct.byref(bad), ct.byref(self.model)), -2)
            status, output = self.step()
            self.assertEqual(status, -5)
            self.assertEqual(output.command, .25)
        self.model.coast_mass_kg = 0
        self.assertEqual(self.lib.mpc_init(ct.byref(self.config), ct.byref(self.model)), -2)
        self.assertEqual(self.lib.mpc_init(None, None), -2)

    def test_density_table_interpolation_and_rejection(self):
        for h in (-1000, 0, 270, 1000, 5500, 20000):
            self.assertAlmostEqual(self.lib.mpc_density_from_table(h, ct.byref(self.table)),
                                   Environment.density(h), places=14)
        self.assertTrue(math.isnan(self.lib.mpc_density_from_table(20001, ct.byref(self.table))))
        self.assertEqual(self.step(h=20001)[0], -3)
        self.heights[2] = 0  # duplicate altitude
        self.assertEqual(self.lib.mpc_density_table_valid(ct.byref(self.table)), 0)
        self.assertEqual(self.lib.mpc_init(ct.byref(self.config), ct.byref(self.model)), -2)

    def test_bad_density_callback_and_finite_overflow(self):
        callback_type = ct.CFUNCTYPE(D, D, ct.c_void_p)
        for value in (math.nan, -1.0, math.inf):
            callback = callback_type(lambda _h, _ctx, value=value: value)
            self.model.density = ct.cast(callback, ct.c_void_p)
            self.initialize()
            status, output = self.step()
            self.assertEqual(status, -3)
            self.assertEqual(output.command, .25)
        self.model.density = ct.cast(self.lib.mpc_density_from_table, ct.c_void_p)
        self.initialize()
        self.assertEqual(self.step(v=1e308)[0], -3)

    def test_bounded_rollout_failure(self):
        self.config.max_coast_steps = 1
        self.initialize()
        status, output = self.step()
        self.assertEqual(status, -3)
        self.assertEqual(output.command, .25)
        self.assertFalse(self.lib.mpc_last_problem())

    def test_iteration_limit_holds_command_and_recovers(self):
        self.assertEqual(self.lib.test_set_max_iter(1), 0)
        status, output = self.step()
        self.assertEqual(status, -4)
        self.assertEqual(output.command, .25)
        self.assertEqual(output.command_valid, 1)
        self.assertNotEqual(output.solver_status, 1)
        self.assertEqual(self.lib.test_set_max_iter(META["max_iter"]), 0)
        self.assertEqual(self.step()[0], 0)

    def test_reset_clears_plan(self):
        self.assertEqual(self.step()[0], 0)
        self.assertTrue(self.lib.mpc_last_problem())
        self.lib.mpc_reset()
        self.assertFalse(self.lib.mpc_last_problem())
        self.assertEqual(self.step(deployment=.75)[0], 0)
        np.testing.assert_array_equal(self.lib.mpc_last_problem().contents.nominal, .75)

    def test_zero_rate_and_zero_apogee_weight(self):
        self.config.max_deployment_rate = 0
        self.initialize()
        status, output = self.step()
        self.assertEqual(status, 0)
        self.assertEqual(output.command, .25)
        self.config = self.lib.mpc_default_config()
        self.config.w_apogee = 0
        self.initialize()
        self.compare_tick(self.reference(), 1000, 200, .25)


if __name__ == "__main__":
    unittest.main(verbosity=2)
