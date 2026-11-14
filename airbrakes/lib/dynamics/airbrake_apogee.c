#include "airbrake_apogee.h"

#include "airbrake_helper.h"
#include "MPRLS_PRESSURE.h"

static double acceleration(const double altitude, const double velocity, const float theta, const float airbrake_active_area)
{
    const float coefficient_of_drag_airbrakes = airbrake_drag_coefficient(airbrake_active_area);
    const float coefficient_of_drag_rocket = rocket_drag_coefficient(airbrake_active_area);
    const float coefficients_of_drag_times_areas = coefficient_of_drag_airbrakes * FULLY_EXTENDED_AIRBRAKE_AREA * airbrake_active_area + coefficient_of_drag_rocket * ROCKET_AREA;

    // The fabs() on velocity ensures it returns the correct sign
    const double drag_acceleration = - theta * 0.5 * air_density_kg_per_meter_cubed((float) altitude) * velocity * fabs(velocity) * coefficients_of_drag_times_areas / ROCKET_MASS;
    return -ACCELERATION_GRAVITY_MS2 + drag_acceleration;
}

double airbrake_apogee(float airbrake_state[QP_STATE_SIZE], uint16_t *apogee_index, const float cur_altitude, const float cur_velocity, const float cur_theta)
{
    double predicted_altitude = cur_altitude;
    double predicted_velocity = cur_velocity;
    
    if (cur_velocity <= 0.0f)
    {
        *apogee_index = 0;
        return cur_altitude;
    }
    
    for (int i = 0; i < QP_STATE_SIZE; ++i)
    {
        const double cur_acceleration = acceleration(predicted_altitude, predicted_velocity, cur_theta, airbrake_state[i]);
        const double next_velocity = predicted_velocity + cur_acceleration * QP_TIMESTEP_SECONDS;
        
        if (next_velocity <= 0.0)
        {
            // Find the time to apogee to return the correct value
            const double time_to_apogee = -predicted_velocity / cur_acceleration;
            
            *apogee_index = i + 1;
            return predicted_altitude + predicted_velocity * time_to_apogee + 0.5 * cur_acceleration * time_to_apogee * time_to_apogee;
        }
        
        predicted_altitude += predicted_velocity * QP_TIMESTEP_SECONDS + 0.5 * cur_acceleration * QP_TIMESTEP_SECONDS * QP_TIMESTEP_SECONDS;
        predicted_velocity = next_velocity;
    }
    
    // If we get here, we were unable to reach apogee
    *apogee_index = QP_STATE_SIZE;
    return predicted_altitude;
}