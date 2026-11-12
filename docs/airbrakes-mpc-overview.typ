#set page(
  numbering: "1",
  paper: "us-letter"
)
#set text(
  font: "New Computer Modern",
  size: 12pt
)
#set par(
  justify: true,
)

#set math.cases(gap: 1em)

#set math.mat(delim: "[")
#set math.vec(delim: "[")

// Equation Numbering
#set math.equation(numbering: "(1)", number-align: bottom)

// Allow equations to break across pages
#show math.equation: set block(breakable: true)

// Increase vector spacing
#set math.vec(gap: 0.7em)
#set math.mat(gap: 0.7em)

= Problem Description

This document details an algorithm for an altitudinal airbrake system for a rocket. After motor burnout, the airbrake system deploys from the side of the rocket, increasing drag and thus slowing the rocket down. This system allows the rocket to reach a desired altitude.

= Algorithm Choice

Model Predictive Control (MPC) is used to control the airbrakes. Given a cost over the desired timestep, MPC uses a quadratic programming algorithm to determine the best airbrake deployment throughout the time horizon. It then executes on the first of these time steps and recalculates for the next.

MPC was chosen over Proportional Integral Derivative (PID) and Linear Quadratic Regulator (LQR) due to its ability to better predict how current decisions impact future planning. With a large enough time window, too much or too little airbrake deployment at the start can be shown to have outstanding effects near the end.

= Rocket Kinematics

Consider a rocket launched with upward velocity $v_0$ at height $z_0$ just after motor burnout. We will only be considering vertical velocity and position from now onward. The force on the rocket is given by

$ F = -m g - F_"Drag" $

Where $m$ is the mass of the rocket _after burnout_ and $g = 9.81 thin "m/s"^2$. The force due to drag can be broken up into two separate components: drag from the rocket itself, and drag from the airbrakes. Given this, we can say

$ F_"Drag" &= 1/2 rho C_D_R A_R v^2 + 1/2 rho C_D_A A_A v^2\
&= 1/2 rho (C_D_R A_R + C_D_A A_A) v^2 $

where the following terms are used:

$ rho &eq.triple "Air density"\
C_D_R &eq.triple "Rocket coefficient of drag"\
C_D_A &eq.triple "Airbrakes coefficient of drag"\
A_R &eq.triple "Rocket cross sectional surface area"\
A_A &eq.triple "Airbrakes cross sectional surface area"\ $

In reality, many of these terms depend on environmental constants. A simplified model (which we will be using) for air density depends on height, and the cross sectional surface area of the airbrakes depend on the airbrake opening amount we will call $alpha$. Thus the drag force is the function

$ F_"Drag" [z, alpha] = 1/2 rho[z] (C_D_R A_R + C_D_A A_A [alpha]) v^2 $

Newton's second law gives

$ m dot.double(z) = F $

and thus 

$ m dot.double(z) &= - m g - 1/2 rho[z] (C_D_R A_R + C_D_A A_A [alpha]) v^2\
(d v)/(d t) &= -g - 1/(2 m) rho[z] (C_D_R A_R + C_D_A A_A [alpha]) v^2 $

Given environmental parameters $z_i, v_i$, and $alpha_i$, after some small timestep $Delta t$, the next set of environmental parameters is given by

$ v_(i + 1) = v_i + (-g - 1/(2 m) rho[z_i] (C_D_R A_R + C_D_A A_A [alpha_i]) v_i^2) (Delta t) $ <next_vel>

$ z_(i + 1) &= z_i +  v_i (Delta t) #h(212pt) $ <next_height>

The apogee $h$ can thus be calculated as

$ h = max_N (z_0 + sum_(i = 0)^N v_i (Delta t)) $

In fact, the calculation of $h$ takes several parameters. They are

$ z_0 &eq.triple "Height after motor burnout"\
Delta t &eq.triple "Time step"\
arrow(alpha) &eq.triple "List of airbrake deployment amounts for all steps from" [0, N] $

Thus $h$ is the function

$ h[z_0, Delta t, arrow(alpha)] = max_N (z_0 + sum_(i = 0)^N v_(i) [v_i, z_i, alpha_i] (Delta t)) $

However, because it is usually difficult to run MPC on a varying set (in our case, $arrow(alpha)$) in a single optimization, we will likely set the horizon larger than needed and discard the part beyond apogee.

= Defining the Cost Function

Model Predictive Control needs a cost function to determine how close a given airbrake deployment vector $arrow(alpha)$ is to the target apogee. Several options for a cost function exist, but the one being used has the form

$ J = w_A (hat(h) - h)^2 + w_D sum_(i = 1)^N (alpha_i - alpha_(i - 1))^2 $

The first term is the expected apogee $hat(h)$ minus the predicted apogee $h$, and the second term ensures the difference in airbrake deployment between subsequent timesteps is minimized. $w_A$ and $w_D$ are the apogee and deployment weights, which can be tuned to favor one term over the other and ensure stricter adherence. This cost function is quadratic as is expected for MPC.

= Running Model Predictive Control

== Setup

The following functions should be created to allow MPC to operate:

1. `next_height(prev_height, prev_velocity, timestep)` - Calculates the height of the rocket at the next timestep; given by @next_height

2. `next_velocity(prev_velocity, air_density, airbrake_deployment_amount, timestep)` - Calculates the velocity of the rocket at the next timestep; given by @next_vel

3. `cost_function(current_height, airbrake_deployment_amount_vector)` - For the current height and list of airbrake deployments, calculates how well the airbrake deployments conform to constraints

== Algorithm

The MPC algorithm requires the following steps:

1. Get the current height from an environmental sensor

2. Using a quadratic programming solver, compute the minimum $arrow(alpha)$ given the cost function

3. Execute only on $alpha_i$

4. Repeat until apogee is reached