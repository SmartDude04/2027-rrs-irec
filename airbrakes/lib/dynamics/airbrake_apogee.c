#include "airbrake_apogee.h"

#include "airbrake_helper.h"
#include "MPRLS_PRESSURE.h"

static float acceleration(const float altitude, const float velocity, const float theta, const float airbrake_active_area)
{
    const float coefficient_of_drag_airbrakes = airbrake_drag_coefficient(airbrake_active_area);
    const float coefficient_of_drag_rocket = rocket_drag_coefficient(airbrake_active_area);
    const float coefficients_of_drag_times_areas = coefficient_of_drag_airbrakes * FULLY_EXTENDED_AIRBRAKE_AREA * airbrake_active_area + coefficient_of_drag_rocket * ROCKET_AREA;

    // The fabs() on velocity ensures it returns the correct sign
    const float drag_acceleration = - theta * 0.5 * air_density_kg_per_meter_cubed(altitude) * velocity * fabs(velocity) * coefficients_of_drag_times_areas / ROCKET_MASS;
    return -ACCELERATION_GRAVITY_MS2 + drag_acceleration;
}

float apogee_from_airbrakes(float airbrake_state[QP_STATE_SIZE], uint16_t *apogee_state_index, const float cur_altitude, const float cur_velocity, const float cur_theta)
{
    float predicted_altitude = cur_altitude;
    float predicted_velocity = cur_velocity;
    
    for (int i = 0; i < QP_STATE_SIZE; ++i)
    {
        // If we are going downward, then we have reached apogee; return
        if (predicted_velocity < 0.0f)
        {
            *apogee_state_index = i;
            return predicted_altitude;
        }
        
        const float prev_altitude = predicted_altitude;
        
        // Update the state
        predicted_altitude += predicted_velocity * QP_TIMESTEP_SECONDS;
        predicted_velocity += acceleration(prev_altitude, predicted_velocity, cur_theta, airbrake_state[i]) * QP_TIMESTEP_SECONDS;
    }
    
    // If we get here, we were notable to reach apogee
    *apogee_state_index = QP_STATE_SIZE - 1;
    return predicted_altitude;
}