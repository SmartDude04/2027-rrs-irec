"""
Airbrake MPC -> STM32 via OSQP code generation
==============================================

This is the embedded-targeted version of airbrake_mpc.py.  Instead of solving
the QP with the OSQP Python interface at run time, this script

  1. builds the *same* QP (P, q, A, l, u) as airbrake_mpc.AirbrakeMPC,
  2. calls OSQP's code generator (embedded "matrices" mode, float32) to emit a
     malloc-free C solver, and
  3. writes airbrake_config.h so the hand-written C controller in
     stm32_airbrake_mpc/app/ (flight model, finite-difference gradient, P/q/l/u
     update, solve) uses exactly the same parameters as the Python model.

Why "matrices" mode: the Taylor-linearized cost changes P (through grad grad^T)
and q every control step, so both must be updatable on the target.  OSQP's
embedded mode requires a fixed sparsity pattern, so P is generated with a dense
upper triangle and A = [I; D] is constant.

Usage
    python3 airbrake_mpc_codegen.py              # generate + build + verify on host
    python3 airbrake_mpc_codegen.py --no-test    # generate only
"""

import argparse
import os
import shutil
import subprocess
import sys

import numpy as np
import scipy.sparse as sparse
import osqp

from airbrake_mpc import RocketModel, AirbrakeMPC, G0, RHO0, H_SCALE

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'stm32_airbrake_mpc')
OSQP_DIR = os.path.join(OUT, 'osqp')
APP_DIR = os.path.join(OUT, 'app')

# ----------------------------------------------------------------------------
# Flight / tuning configuration (the C controller is generated from these)
# ----------------------------------------------------------------------------
# TARGET_APOGEE = 3048.0      # [m]
# H_BURNOUT = 900.0           # nominal state used to condition the generated solver
# V_BURNOUT = 280.0
TARGET_APOGEE = 944.0
H_BURNOUT = 231.0
V_BURNOUT = 140.0
DT_INT = 0.05               # flight-model integration step [s]
FLOAT_EPS = 1e-4            # OSQP tolerances (float32 cannot reach 1e-6)
OSQP_MAX_ITER = 200         # hard iteration cap = bounded run time on the MCU


def make_controller():
    """Python reference controller (double precision, same tuning as the C port)."""
    return AirbrakeMPC(RocketModel(), TARGET_APOGEE)


# ----------------------------------------------------------------------------
# Representative QP: fixes sparsity, scaling and rho of the generated solver
# ----------------------------------------------------------------------------
def representative_qp(mpc, h, v):
    """Build P (dense upper triangle), q, A, l, u at a typical mid-flight state.

    OSQP fixes the sparsity pattern (and KKT factorization layout) at setup time;
    only values may change afterwards.  The pattern is therefore built explicitly
    as a full upper triangle and must NOT depend on the flight state: slots past
    apogee have exactly zero gradient, and csc_matrix(dense) would silently drop
    those zeros.  Values are overwritten at run time by osqp_update_data_*.
    """
    N = mpc.N
    alpha0 = np.full(N, 0.2)
    h0, grad = mpc._linearize(h, v, alpha0)
    beta = h0 - grad @ alpha0 - mpc.h_target
    Pfull = 2.0 * (mpc.wA * np.outer(grad, grad) + mpc.S + mpc.DtQD)
    # Explicit dense upper-triangle pattern; numerical zeros stay as stored entries.
    P = sparse.triu(sparse.csc_matrix(np.ones((N, N))), format='csc')
    P.sort_indices()
    cols = np.repeat(np.arange(N), np.diff(P.indptr))
    P.data = Pfull[P.indices, cols].astype(float)
    assert P.nnz == N * (N + 1) // 2, 'P must be a dense upper triangle (fixed pattern)'
    assert np.array_equal(P.indptr, [j * (j + 1) // 2 for j in range(N + 1)])
    assert np.array_equal(P.indices, [r for j in range(N) for r in range(j + 1)])
    q = 2.0 * mpc.wA * beta * grad
    dmax = mpc.rate_max * mpc.dt
    lo = np.clip(alpha0 - mpc.trust, 0, mpc.alpha_max)
    hi = np.clip(alpha0 + mpc.trust, 0, mpc.alpha_max)
    l = np.hstack([lo, np.full(N, -dmax)])
    u = np.hstack([hi, np.full(N, dmax)])
    return P, q, mpc.A, l, u


def generate_osqp_solver(mpc):
    P, q, A, l, u = representative_qp(mpc, H_BURNOUT, V_BURNOUT)
    prob = osqp.OSQP()
    prob.setup(P, q, A, l, u,
               verbose=False, polish=False, warm_starting=True,
               eps_abs=FLOAT_EPS, eps_rel=FLOAT_EPS, max_iter=OSQP_MAX_ITER,
               adaptive_rho=False, scaling=10)
    res = prob.solve()
    print(f'Representative QP solved on host: {res.info.status}, {res.info.iter} iterations')

    if os.path.isdir(OSQP_DIR):
        shutil.rmtree(OSQP_DIR)
    prob.codegen(OSQP_DIR, parameters='matrices', extension_name=None,
                 force_rewrite=True, use_float=True, include_codegen_src=True,
                 printing_enable=False, profiling_enable=False, interrupt_enable=False)
    # Drop the generator's sample main and desktop Makefile; we ship our own.
    for f in ('emosqp.c', 'Makefile'):
        p = os.path.join(OSQP_DIR, f)
        if os.path.exists(p):
            os.remove(p)
    print(f'OSQP embedded solver written to {OSQP_DIR}')


# ----------------------------------------------------------------------------
# airbrake_config.h
# ----------------------------------------------------------------------------
def cf(x):
    s = f'{float(x):.9g}'
    if not any(c in s for c in '.en'):
        s += '.0'
    return s + 'f'


def write_config(mpc, model):
    sub_steps = max(1, int(round(mpc.dt / DT_INT)))
    S = np.diag(mpc.S)
    Q = np.diag(mpc.Q)
    lines = [
        '/* Generated by airbrake_mpc_codegen.py -- do not edit by hand. */',
        '#ifndef AIRBRAKE_CONFIG_H',
        '#define AIRBRAKE_CONFIG_H',
        '',
        '/* MPC structure */',
        f'#define AB_N              {mpc.N}',
        f'#define AB_DT_MPC         {cf(mpc.dt)}      /* control step / plan segment [s] */',
        f'#define AB_DT_INT         {cf(DT_INT)}     /* flight-model step while holding last alpha [s] */',
        f'#define AB_SUB_STEPS      {sub_steps}          /* RK2 sub-steps per plan segment */',
        f'#define AB_ALPHA_MAX      {cf(mpc.alpha_max)}',
        f'#define AB_RATE_MAX       {cf(mpc.rate_max)}      /* max |d alpha/dt| [1/s] */',
        f'#define AB_EPS            {cf(mpc.eps)}     /* finite-difference step */',
        f'#define AB_TRUST          {cf(mpc.trust)}      /* trust region about alpha0 */',
        f'#define AB_N_ITER         {mpc.n_iter}          /* re-linearizations per step */',
        f'#define AB_W_APOGEE       {cf(mpc.wA)}',
        f'#define AB_TARGET_APOGEE_M {cf(mpc.h_target)}',
        '',
        '/* Per-slot weights: stickout S and smoothness Q (diagonals) */',
        f'static const float AB_S_DIAG[AB_N] = {{ {", ".join(cf(x) for x in S)} }};',
        f'static const float AB_Q_DIAG[AB_N] = {{ {", ".join(cf(x) for x in Q)} }};',
        '',
        '/* Rocket / atmosphere model */',
        f'#define AB_G0             {cf(G0)}',
        f'#define AB_RHO0           {cf(RHO0)}',
        f'#define AB_H_SCALE        {cf(H_SCALE)}',
        f'#define AB_MASS_KG        {cf(model.m)}',
        f'#define AB_BODY_AREA_M2   {cf(model.A)}',
        f'#define AB_BODY_CD        {cf(model.cd)}',
        f'#define AB_BRAKE_AREA_M2  {cf(model.ab_area)}',
        f'#define AB_BRAKE_CD       {cf(model.ab_cd)}',
        '',
        '#endif /* AIRBRAKE_CONFIG_H */',
        '',
    ]
    os.makedirs(APP_DIR, exist_ok=True)
    with open(os.path.join(APP_DIR, 'airbrake_config.h'), 'w') as f:
        f.write('\n'.join(lines))
    print(f'Wrote {os.path.join(APP_DIR, "airbrake_config.h")}')


MAKEFILE = """\
# Host build of the embedded controller, for verification only.
# For the STM32 build, add osqp/src/*.c, osqp/workspace.c and app/airbrake_mpc.c
# to your project (see README.md).
CC      ?= gcc
CFLAGS  ?= -O2 -Wall -std=c11
INC     := -Iosqp/inc/public -Iosqp/inc/private -Iosqp -Iapp
SRCS    := $(wildcard osqp/src/*.c) osqp/workspace.c app/airbrake_mpc.c

test_host: $(SRCS) app/test_host.c app/airbrake_config.h
\t$(CC) $(CFLAGS) $(INC) -o $@ $(SRCS) app/test_host.c -lm

clean:
\trm -f test_host
"""


# ----------------------------------------------------------------------------
# Host verification: C (float32, generated OSQP) vs Python reference (float64)
# ----------------------------------------------------------------------------
def python_reference(cd_error):
    model = RocketModel()
    plant = RocketModel(cd_scale=cd_error)
    mpc = make_controller()
    h, v = H_BURNOUT, V_BURNOUT
    alphas = []
    while v > 0.0:
        cmd, _ = mpc.solve(h, v)
        alphas.append(cmd)
        h_new, v_new = plant.step(h, v, cmd, mpc.dt, 20)
        if v_new <= 0.0:
            h = h + v / (v - v_new) * (h_new - h)
            v = 0.0
        else:
            h, v = h_new, v_new
    return h, np.array(alphas)


def parse_c_output(text):
    rows, summary = [], {}
    for line in text.splitlines():
        if line.startswith('# apogee'):
            tok = line.split()
            summary['apogee'] = float(tok[2]); summary['error'] = float(tok[6])
        elif line.startswith('# max_model'):
            tok = line.split()
            summary['evals'] = int(tok[2]); summary['iters'] = int(tok[4]); summary['fallbacks'] = int(tok[6])
        elif not line.startswith('#') and line.strip():
            rows.append([float(x) for x in line.split()])
    return np.array(rows), summary


def build_and_verify():
    with open(os.path.join(OUT, 'Makefile'), 'w') as f:
        f.write(MAKEFILE)
    subprocess.check_call(['make', '-s', 'clean'], cwd=OUT)
    subprocess.check_call(['make', '-s'], cwd=OUT)
    print('\nHost build OK.  Closed-loop comparison (no sensor noise):')
    print(' cd_error | python apogee | C apogee | max |d alpha| | ADMM iters | fallbacks')
    ok = True
    for cd_error in (0.9, 1.0, 1.1, 1.2):
        h_py, a_py = python_reference(cd_error)
        out = subprocess.check_output(['./test_host', str(cd_error), str(H_BURNOUT), str(V_BURNOUT)],
                                      cwd=OUT, text=True)
        rows, s = parse_c_output(out)
        a_c = rows[:, 3]
        n = min(len(a_c), len(a_py))
        dalpha = np.max(np.abs(a_c[:n] - a_py[:n]))
        print(f'  {cd_error:5.2f}   | {h_py:10.1f} m  | {s["apogee"]:7.1f} m | {dalpha:12.4f}  | {s["iters"]:9d}  | {s["fallbacks"]:d}')
        ok &= abs(s['apogee'] - h_py) < 15.0 and s['fallbacks'] == 0
    print('\nVERIFIED: embedded C matches the Python reference.' if ok else
          '\nWARNING: C and Python results differ more than expected.')
    out = subprocess.check_output(['./test_host', '1.0', str(H_BURNOUT), str(V_BURNOUT)],
                                  cwd=OUT, text=True)
    _, s = parse_c_output(out)
    print(f'Worst-case flight-model evaluations per step: {s["evals"]}  '
          f'(~{s["evals"] * 120 / 1e6:.1f} M cycles at ~120 cycles per accel(); '
          f'divide by core clock for run time)')
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--no-test', action='store_true', help='generate only; skip host build/verify')
    args = ap.parse_args()

    mpc = make_controller()
    generate_osqp_solver(mpc)
    write_config(mpc, mpc.model)
    if not args.no_test:
        sys.exit(0 if build_and_verify() else 1)


if __name__ == '__main__':
    main()
