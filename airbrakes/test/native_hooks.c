/* Host verification only; excluded from the STM32 source filter. */
#include <math.h>
#include "mpc.h"
#include "osqp_mpc/osqp.h"
#include "osqp_mpc/airbrake_workspace.h"

int test_set_max_iter(int iterations) {
    OSQPSettings settings = *airbrake_solver.settings;
    settings.max_iter = iterations;
    return osqp_update_settings(&airbrake_solver, &settings);
}

double test_solution(int index) {
    if (index < 0 || index >= MPC_HORIZON) return NAN;
    return airbrake_solver.solution->x[index];
}

size_t test_structure_size(int index) {
    switch (index) {
        case 0: return sizeof(MPCConfig);
        case 1: return sizeof(MPCModel);
        case 2: return sizeof(MPCDensityTable);
        case 3: return sizeof(MPCInput);
        case 4: return sizeof(MPCOutput);
        case 5: return sizeof(MPCProblem);
        default: return 0;
    }
}
