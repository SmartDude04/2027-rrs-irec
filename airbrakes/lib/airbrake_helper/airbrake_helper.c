#include "airbrake_helper.h"

#include <math.h>

float airbrake_drag_coefficient(const float airbrake_active_area)
{
    // Polynomial derived from order 3 regression on airbrake data
    // Drag coefficient not found to substantially vary with altitude
    return 7e06f * powf(airbrake_active_area, 3) - 73613.0f * powf(airbrake_active_area, 2) + 305.55f * airbrake_active_area + 0.2378f;
}

float rocket_drag_coefficient(const float airbrake_active_area)
{
    // Polynomial derived from order 3 regression on airbrake data
    // Drag coefficient not found to substantially vary with altitude
    return -973.15f * powf(airbrake_active_area, 2) - 0.1895f * airbrake_active_area + 0.4311;
}