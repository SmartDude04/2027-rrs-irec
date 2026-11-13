#ifndef AIRBRAKES_AIRBRAKE_HELPER_H
#define AIRBRAKES_AIRBRAKE_HELPER_H

/**
 * Performs a lookup/approximation of the coefficient of drag for the airbrake given the opening amount
 * 
 * @param airbrake_active_area Area of the airbrake in the wind stream being used to slow the rocket down
 * @return The CFD-derived coefficient of drag for the airbrake
 */
float airbrake_drag_coefficient(float airbrake_active_area);

#endif