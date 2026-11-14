#ifndef AIRBRAKES_KALMAN_H
#define AIRBRAKES_KALMAN_H


typedef struct
{
    float altitude;
    float velocity;
    float theta;
    float covariance[3][3];
    float process_noise[3];
    float barometer_variance;
    float accelerometer_variance;
} EKF_State;
extern EKF_State ekf_state;


/**
 * Initializes the kalman filter state
 * 
 * @param initial_altitude_meters Initial altitude above sea level in meters
 * @param initial_velocity_meters_per_second Initial velocity of the rocket in meters per second
 */
void ekf_init(float initial_altitude_meters, float initial_velocity_meters_per_second);

/**
 * Runs the predict step in the EKF filter. Should be run *before* updating the EKF with sensor data, and should be
 * run frequently
 * 
 * The predict step does two things. First, it updates the state to agree with the model and physics in general. Second,
 * it increases the uncertainty in the state values. The uncertainty will go down when new measurements come in.
 * 
 * @param airbrake_active_area Amount the airbrake is open, from 0 to 1
 * @param time_since_last_predict Time since the last time ekf_predict() was run
 */
void ekf_predict(float airbrake_active_area, float time_since_last_predict);

/**
 * Updates the EKF with an altitude measurement from the barometer
 * 
 * @param measured_altitude_meters Altitude, in meters, above sea level for the rocket
 */
void ekf_update_barometer(float measured_altitude_meters);

/**
 * Updates the EKF with an acceleration measurement from the IMU
 * 
 * @param measured_acceleration Measured acceleration, in meters per second squared
 * @param airbrake_active_area Amount the airbrake is open, from 0 to 1
 */
void ekf_update_acceleration(float measured_acceleration, float airbrake_active_area);


#endif
