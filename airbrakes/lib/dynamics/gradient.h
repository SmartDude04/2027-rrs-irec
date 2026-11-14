#ifndef AIRBRAKES_GRADIENT_H
#define AIRBRAKES_GRADIENT_H

#include "constants.h"

/**
 * Computes the gradient of the function which maps an airbrake opening amount to an apogee. Used to linearize flight
 * dynamics to plug into the QP solver
 *
 * @param airbrake_state List of airbrake active areas, from 0 to 1
 * @param gradient Gradient will be calculated in this array; all existing data will be overridden
 * @param altitude Current altitude, in meters, above the pad
 * @param velocity Current velocity, in meters per second
 * @param theta Drag scale factor
 */
void airbrake_state_gradient(float airbrake_state[QP_STATE_SIZE], float gradient[QP_STATE_SIZE], float altitude, float velocity, float theta);

#endif