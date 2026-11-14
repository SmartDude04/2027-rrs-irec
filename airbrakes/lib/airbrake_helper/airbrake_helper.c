#include "airbrake_helper.h"

#include <math.h>

float airbrake_drag_coefficient(const float airbrake_active_area)
{
    // Polynomial derived from order 3 regression on airbrake data
    // Drag coefficient not found to substantially vary with altitude
    return 1.9854f * powf(airbrake_active_area, 3) - 3.3552f * powf(airbrake_active_area, 2) + 2.1117f * airbrake_active_area + 0.2166;
}

float rocket_drag_coefficient(const float airbrake_active_area)
{
    // Polynomial derived from order 3 regression on airbrake data
    // Drag coefficient not found to substantially vary with altitude
    return -0.0459f * powf(airbrake_active_area, 3) + 0.0256f * powf(airbrake_active_area, 2) - 0.0269f * airbrake_active_area + 0.4328f;
}