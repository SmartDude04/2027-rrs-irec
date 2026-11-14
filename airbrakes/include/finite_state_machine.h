#ifndef AIRBRAKES_FINITE_STATE_MACHINE_H
#define AIRBRAKES_FINITE_STATE_MACHINE_H

typedef enum
{
    DISARMED,
    PRE_LAUNCH,
    BOOST,
    COAST,
    CONTROL,
    EXIT,
    INFEASIBLE
} Airbrakes_Finite_State_Machine;
extern Airbrakes_Finite_State_Machine airbrakes_state;

#endif