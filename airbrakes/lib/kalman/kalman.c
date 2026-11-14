#include "kalman.h"

#include <memory.h>

#include "airbrake_helper.h"
#include "MPRLS_PRESSURE.h"

#define ACCELERATION_GRAVITY_MS2 9.80665f
#define FULLY_EXTENDED_AIRBRAKE_AREA 1.0f
#define ROCKET_AREA 1.0f
#define ROCKET_MASS 1.0f

EKF_State ekf_state;

/**
 * Calculates the specific force on the rocket given given parameters
 * 
 * @param altitude Altitude of the rocket, in meters
 * @param velocity Velocity of the rocket, in meters per second
 * @param theta Scale factor for the drag coefficient
 * @param airbrake_active_area Amount the airbrake is open, from 0 to 1
 * @return The specific force, in Newtons per meter, on the rocket
 */
static float specific_force(const float altitude, const float velocity, const float theta, float airbrake_active_area)
{
    const float coefficient_of_drag_airbrakes = airbrake_drag_coefficient(airbrake_active_area);
    const float coefficient_of_drag_rocket = rocket_drag_coefficient(airbrake_active_area);
    const float coefficients_of_drag_times_areas = coefficient_of_drag_airbrakes * FULLY_EXTENDED_AIRBRAKE_AREA * airbrake_active_area + coefficient_of_drag_rocket * ROCKET_AREA;
    
    // The fabs() on velocity ensures it returns the correct sign
    return - theta * 0.5 * air_density_kg_per_meter_cubed(altitude) * velocity * fabs(velocity) * coefficients_of_drag_times_areas / ROCKET_MASS;
}

/**
 * Calculates the middle row divided by the (delta t) term of the Jacobian of the specific force on the Rocket given
 * the EKF state
 * 
 * The specific force, f, is a vector function denoted as f(h, v, theta) = (h.hat, v.hat, theta.hat)
 * Written out, these hat functions, which are functions of all inputs, have the form
 * - h.hat(h, v, theta, alpha) = h + v (delta t)
 * - v.hat = v + (a(h, v, theta, alpha) - g) (delta t)
 * - theta.hat = theta
 * 
 * Thus, the Jacobian, F, is the matrix of partial derivatives of each of these outputs with respect to each input
 * 
 * 
 * @param altitude Altitude of the rocket, in meters
 * @param velocity Velocity of the rocket, in meters per second
 * @param theta Scale factor for the drag coefficient
 * @param airbrake_active_area Amount the airbrake is open, from 0 to 1
 * @param jacobian Array where the Jacobian will be stored
 */
static void partial_specific_force_jacobian(const float altitude, const float velocity, const float theta, const float airbrake_active_area, float jacobian[3])
{
    // Derivatives are calculated by finite differences, essentially the formula learned in high school:
    // f'(x) = (f(x + dx) - f(x))/dx
    
    const float dh = 1.0f; // Small change in altitude
    const float dv = 0.1f; // Small change in velocity
    const float dt = 1e-3f; // Small change in theta
    
    // Only the middle row divided by the (delta t) is calculated the enhance code reusability
    const float initial_specific_force = specific_force(altitude, velocity, theta, airbrake_active_area);
    
    jacobian[0] = (specific_force(altitude + dh, velocity, theta, airbrake_active_area) - initial_specific_force) / dh;
    jacobian[1] = (specific_force(altitude, velocity + dv, theta, airbrake_active_area) - initial_specific_force) / dv;
    jacobian[2] = (specific_force(altitude, velocity, theta + dt, airbrake_active_area) - initial_specific_force) / dt;
}

void ekf_init(const float initial_altitude_meters, const float initial_velocity_meters_per_second)
{
    // TODO: Tune these initial values
    memset((void *) &ekf_state, 0, sizeof(ekf_state));
    ekf_state.altitude = initial_altitude_meters;
    ekf_state.velocity = initial_velocity_meters_per_second;
    ekf_state.theta = 1.0f;
    
    // Set covariance to realistic numbers
    ekf_state.covariance[0][0] = 4.0f;
    ekf_state.covariance[1][1] = 1.0f;
    ekf_state.covariance[2][2] = 0.01f;
    
    // Again, set to realistic numbers
    ekf_state.process_noise[0] = 0.0f;
    ekf_state.process_noise[1] = 0.5f;
    ekf_state.process_noise[2] = 2e-4;
    
    ekf_state.accelerometer_variance = 0.5f;
    ekf_state.barometer_variance = 4.0f;
}

void ekf_predict(const float airbrake_active_area, const float time_since_last_predict)
{
    const float altitude = ekf_state.altitude, velocity = ekf_state.velocity, theta = ekf_state.theta;
    
    float partial_jacobian[3];
    partial_specific_force_jacobian(altitude, velocity, theta, airbrake_active_area, partial_jacobian);
    const float jacobian[3][3] = {
        {1.0f, time_since_last_predict, 0.0f},
        {time_since_last_predict * partial_jacobian[0], 1.0f + time_since_last_predict * partial_jacobian[1], time_since_last_predict * partial_jacobian[2]},
        {0.0f, 0.0f, 1.0f}
    };
    
    // Update the state
    const float acceleration = specific_force(altitude, velocity, theta, airbrake_active_area) - ACCELERATION_GRAVITY_MS2;
    ekf_state.altitude = altitude + velocity * time_since_last_predict;
    ekf_state.velocity = velocity + acceleration * time_since_last_predict;
    
    // Update the uncertainty matrices
    // 
    // New covariance = jacobian * covariance * jacobian^T + noise * dt
    float intermediate_covariance[3][3];
    float new_covariance[3][3];
    // First do intermediate_covariance = jacobian * covariance
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            intermediate_covariance[i][j] = 0.0f;
            for (int k = 0; k < 3; ++k)
            {
                intermediate_covariance[i][j] += jacobian[i][k] * ekf_state.covariance[k][j];
            }
        }
    }
    // Now do new_covariance = intermediate_covariance * jacobian^T
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            new_covariance[i][j] = 0.0f;
            for (int k = 0; k < 3; ++k)
            {
                new_covariance[i][j] += intermediate_covariance[i][k] * jacobian[j][k];
            }
        }
    }
    
    // Now add on process noise * time, where process noise is treated as a diagonal matrix
    for (int i = 0; i < 3; ++i)
    {
        new_covariance[i][i] += ekf_state.process_noise[i] * time_since_last_predict;
    }
    
    memcpy((void *) ekf_state.covariance, new_covariance, sizeof(ekf_state.covariance));
}

/**
 * Updates the EKF with a scalar from an environmental measurement (i.e. barometer or IMU)
 * 
 * @param measurement New measurement made by the sensor
 * @param predicted_measurement Predicted measurement by the EKF
 * @param measurement_jacobian Jacobian of the state as a function of the previous states
 * @param measurement_noise Noise of this measurement; should come from the EKF state
 * @return The innovation variance squared; a measure of how well a new measurement conforms to the current EKF state
 */
static float scalar_update(const float measurement, const float predicted_measurement, const float measurement_jacobian[3], const float measurement_noise)
{
    float covariance_times_jacobian[3];
    for (int i = 0; i < 3; ++i)
    {
        covariance_times_jacobian[i] = ekf_state.covariance[i][0] * measurement_jacobian[0] + ekf_state.covariance[i][1] * measurement_jacobian[1] + ekf_state.covariance[i][2] * measurement_jacobian[2];
    }
    
    const float innovation = measurement - predicted_measurement;
    const float innovation_variance = measurement_jacobian[0] * covariance_times_jacobian[0] + measurement_jacobian[1] * covariance_times_jacobian[1] + measurement_jacobian[2] * covariance_times_jacobian[2] + measurement_noise;
    
    // If the sensor produces a measurement that is way outside what we predict it should be, simply throw it away
    const float normalized_innovation_squared = innovation * innovation / innovation_variance;
    if (normalized_innovation_squared > 9.0f)
    {
        return normalized_innovation_squared;
    }
    
    // Update the state of the system with an amount proportional to the measurement as well as how nicely the
    // measurement conforms to our previous values
    const float kalman_gain[3] = {covariance_times_jacobian[0] / innovation_variance,
                            covariance_times_jacobian[1] / innovation_variance,
                            covariance_times_jacobian[2] / innovation_variance};
    ekf_state.altitude += kalman_gain[0] * innovation;
    ekf_state.velocity += kalman_gain[1] * innovation;
    ekf_state.theta += kalman_gain[2] * innovation;
    
    // Update the covariance
    // Uses Joseph form: P = (I-KH) P (I-KH)^T + R K K^T
    float i_minus_kh[3][3];
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            i_minus_kh[i][j] = (i == j ? 1.0f : 0.0f) - kalman_gain[i] * measurement_jacobian[j];
        }
    }
    float i_minus_kh_p[3][3];
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            i_minus_kh_p[i][j] = 0.0f;
            for (int k = 0; k < 3; ++k)
            {
                i_minus_kh_p[i][j] += i_minus_kh[i][k] * ekf_state.covariance[k][j];
            }
        }
    }
    float new_covariance[3][3];
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            new_covariance[i][j] = measurement_noise * kalman_gain[i] * kalman_gain[j];
            for (int k = 0; k < 3; ++k)
            {
                new_covariance[i][j] += i_minus_kh_p[i][k] * i_minus_kh[j][k];
            }
        }
    }
    memcpy((void *) ekf_state.covariance, new_covariance, sizeof(ekf_state.covariance));
    
    // If theta is updated to be quite big/small, then clamp it
    if (ekf_state.theta < 0.7f) ekf_state.theta = 0.7f;
    if (ekf_state.theta > 1.3f) ekf_state.theta = 1.3f;
    
    return normalized_innovation_squared;
}

void ekf_update_barometer(const float measured_altitude_meters)
{
    // Measurement Jacobian is simply [1, 0, 0]; no update dependence on altitude with velocity or theta
    const float measurement_jacobian[3] = {1.0f, 0.0f, 0.0f};
    
    scalar_update(measured_altitude_meters, ekf_state.altitude, measurement_jacobian, ekf_state.barometer_variance);
}

void ekf_update_acceleration(const float measured_acceleration, const float airbrake_active_area)
{
    // Measurement Jacobian is the partial Jacobian of the specific force (see function above)
    float measurement_jacobian[3];
    partial_specific_force_jacobian(ekf_state.altitude, ekf_state.velocity, ekf_state.theta, airbrake_active_area, measurement_jacobian);
    const float predicted_acceleration = specific_force(ekf_state.altitude, ekf_state.velocity, ekf_state.theta, airbrake_active_area);
    
    scalar_update(measured_acceleration, predicted_acceleration, measurement_jacobian, ekf_state.accelerometer_variance);
}