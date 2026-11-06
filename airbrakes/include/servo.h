#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

void FINNIAN_TIM16_Init(void);
void Servo_Start(void);
void Servo_SetPulseUs(uint16_t pulse_us);
void Error_Handler(void);

#endif