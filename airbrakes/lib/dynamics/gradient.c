#include "gradient.h"

#include "airbrake_apogee.h"
#include "usb_cdc.h"

#define EPSILON 1e-3f

void airbrake_state_gradient(float airbrake_state[QP_STATE_SIZE], float gradient[QP_STATE_SIZE], const float altitude, const float velocity, const float theta)
{
    // Because we are performing a discrete derivative, when varying a single state, we need a reference
    // to compare against
    // Remember, derivative formula is simply f'(x) = ((f(x + dx) - f(x))/dx
    uint16_t apogee_index;
    const double standard_apogee = airbrake_apogee(airbrake_state, &apogee_index, altitude, velocity, theta);

    // We will only compute the gradient up to the standard apogee. This is because if we perturb values after it,
    // nothing will happen and we will simply waste compute
    for (int i = 0; i < apogee_index; ++i)
    {
        const float original_value = airbrake_state[i];

        // If we are beyond the boundary for the airbrake state range [0.0, 1.0], then use a backward difference
        if (airbrake_state[i] + EPSILON < 1.0f)
        {
            airbrake_state[i] += EPSILON;
        }
        else
        {
            airbrake_state[i] -= EPSILON;
        }
        const double step = airbrake_state[i] - ((double) original_value); // This is the "dx" value

        uint16_t apogee_index_filler;
        const double perturbed_apogee = airbrake_apogee(airbrake_state, &apogee_index_filler, altitude, velocity, theta);

        gradient[i] = (float) ((perturbed_apogee - standard_apogee) / step);
        airbrake_state[i] = original_value;
    }

    // Fill the rest of the gradient with zeros
    for (int i = apogee_index; i < QP_STATE_SIZE; ++i)
    {
        gradient[i] = 0.0f;
    }
}