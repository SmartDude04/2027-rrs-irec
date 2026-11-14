#ifndef AIRBRAKES_AIRBRAKE_APOGEE_H
#define AIRBRAKES_AIRBRAKE_APOGEE_H

#include <stdint.h>

#include "constants.h"

/**
 * Performs a double-integration rollout of computed acceleration values on the rocket to estimate an apogee value
 * given airbrake opening amounts for the rest of the flight
 * 
 * @param airbrake_state List of future airbrake opening amounts
 * @param apogee_index Will be set to the index right after apogee
 * @param cur_altitude Current altitude above the pad, in meters
 * @param cur_velocity Current velocity, in meters per second
 * @param cur_theta Drag coefficient scale factor
 * @return Predicted apogee, in meters
 */
double airbrake_apogee(float airbrake_state[QP_STATE_SIZE], uint16_t *apogee_index, float cur_altitude, float cur_velocity, float cur_theta);

#endif