#ifndef MPC_EXAMPLE_MODEL_H
#define MPC_EXAMPLE_MODEL_H

#include "mpc.h"

/* Illustrative bench model, not launch-specific atmosphere or calibrated mass.
 * Replace with a preflight RocketPy density export/calibrated coast mass.
 * Padding is needed for the +/-1 m density and +/-0.1 m terminal differences. */
static const double example_altitudes[] = {-1000, 0, 1000, 2000, 4000, 8000, 20000};
static const double example_densities[] = {1.35, 1.225, 1.112, 1.007, 0.819, 0.525, 0.089};
static MPCDensityTable example_density_table = {
    example_altitudes, example_densities,
    sizeof(example_altitudes) / sizeof(example_altitudes[0])
};

static inline MPCModel mpc_example_model(void) {
    const MPCModel result = {
        .coast_mass_kg = 24.3209,
        .elevation_asl_m = 270.0,
        .density = mpc_density_from_table,
        .density_context = &example_density_table
    };
    return result;
}
#endif
