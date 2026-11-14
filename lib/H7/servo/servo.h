#ifndef AIRBRAKES_SERVO_H
#define AIRBRAKES_SERVO_H

#include <stdint.h>

/**
 * Initialize PA6 to be a PWM output for the motor. Initializes TIM3 to be the timer for PWM
 */
void Servo_Init_TIM3_PA6();

/**
 * Sets the motor position in the available range to the given degree value
 * @param degrees Amount to set the motor position, from 0 to 180
 */
void Servo_Move_Degrees(uint8_t degrees);

#endif
