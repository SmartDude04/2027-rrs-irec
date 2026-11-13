#ifndef __MAIN_H
#define __MAIN_H

#include "ICM_20948_IMU.h"

extern volatile float pressure_hpa;
extern volatile float altitude_meters;
extern volatile float air_density_kg_m_cubed;
extern volatile IMU_Data imu_data;

#endif