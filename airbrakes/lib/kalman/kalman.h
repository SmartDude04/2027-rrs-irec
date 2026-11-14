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
 * Calculates the specific force on the rocket given given parameters
 * 
 * @param altitude Altitude of the rocket, in meters
 * @param velocity Velocity of the rocket, in meters per second
 * @param theta Scale factor for the drag coefficient
 * @param airbrake_active_area Amount the airbrake is open, from 0 to 1
 * @return The specific force, in Newtons per meter, on the rocket
 */
static float specific_force(float altitude, float velocity, float theta, float airbrake_active_area);

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
static void partial_specific_force_jacobian(float altitude, float velocity, float theta, float airbrake_active_area, float jacobian[3]);

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
 * Updates the EKF with a scalar from an environmental measurement (i.e. barometer or IMU)
 * 
 * @param measurement New measurement made by the sensor
 * @param predicted_measurement Predicted measurement by the EKF
 * @param measurement_jacobian Jacobian of the state as a function of the previous states
 * @param measurement_noise Noise of this measurement; should come from the EKF state
 * @return The innovation variance squared; a measure of how well a new measurement conforms to the current EKF state
 */
static float scalar_update(float measurement, float predicted_measurement, const float measurement_jacobian[3], float measurement_noise);

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
