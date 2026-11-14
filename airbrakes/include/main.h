#ifndef __MAIN_FUN_H__
#define __MAIN_FUN_H__

#include "ICM_20948_IMU.h"

extern volatile float cur_pressure_hpa;
extern volatile float cur_altitude_meters;
extern volatile float cur_air_density_kg_m_cubed;
extern volatile float cur_speed_of_sound_meters_per_second;
extern volatile IMU_Data cur_imu_data;

#endif