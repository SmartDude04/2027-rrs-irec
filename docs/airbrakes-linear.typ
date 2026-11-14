#set math.mat(delim: "[")

#let eval(expr) = $lr(#expr|, size: #100%)$

We initially defined a cost function for our Model Predictive Control simulation in the form of

$ J = dots.c. $

We were then able to using this model in conjuction with Python's SciPy package, resulting in a model that could be solved using Python. However, this wasn't able to be used with the solver we wanted to use for our microcontroller, as the quadratic terms weren't something that could be solved efficiently by OSQP.

We therefore defined a new cost function using a first-order taylor approximation that could be solved on the microcontroller with OSQP. For the purposes of exhaustion we will demonstrate the same fundamental physics derivations as previous cost function. It will be explicitly mentioned with the linear model differs from the original cost function.

Newton's fundamental laws give us

$ sum F = m (dif v) / (dif t) = - m g - F_D, $

while we can relate the height of the rocket with the vertical velocity as

$ (dif h) / (dif t) = v. $

We are using a quadratic drag model, resulting in

$
  F_D &= 1/2 rho C_D A v^2 \
  &= 1/2 rho(h) (C_(D, R) A_R + C_(D, A) A_A alpha) v^2.
$

Therefore, the fundamental equation of motion for the rocket in the vertical direction is

$ (dif v) / (dif t) = - g - rho(h) / (2m) (C_(D, R) A_R + C_(D, A) A_A alpha) v^2. $

We are then going to define a state space form for our equations, which will represent a matrix that can be manipulated by OSQP.

$ bold(x) (t) = mat(h(t); v(t)). $

We will also difine $u(t) = alpha(t)$. We also get

$ bold(dot(x)) = mat(dot(h) (t); dot(v) (t)) = mat(v; - g - rho(h) / (2 m) (C_(D, R) A_R + C_(D, A) A_A u) v^2) $

where $bold(dot(x)) = f(x, u)$ is the nonlinear state space model. We then define the nominal states of the system to be $bold(overline(x)) = mat(overline(h); overline(v))$ and $overline(u)$. Then also have $delta x = x - overline(x)$ and $delta u = u - overline(u)$. A first-order taylor approximation can be defined as

$ f(x, u) approx f(overline(x), overline(u)) + eval((partial f) / (partial x))_(overline(x), overline(u)) delta x + eval((partial f) / (partial y))_(overline(x), overline(u)) delta u. $

Intuitively speaking, $eval((partial f) / (partial x))_(overline(x), overline(u))$ describe how a change in altitude adnd velocity affect the system's rate of chance while $eval((partial f) / (partial u))_(overline(x), overline(u))$ describe how changing the airbrakes will change the system. We will then define

$ A_c = eval((partial f) / (partial x))_(overline(x), overline(u)), quad B_c = eval((partial f) / (partial u))_(overline(x), overline(u)). $

In order to take the derivative, we take the Jacobian, which results in

$ A_c = mat((partial dot(h) (t)) / (partial h), (partial dot(h) (t)) / (partial v); (partial dot(v) (t)) / (partial h), (partial dot(v) (t)) / (partial v)) = mat(0, 1; - (rho' overline(h)) / (2 m) (C_(D, R) A_R + C_(D, A) A_A overline(u)) overline(v)^2, - (rho (overline(h))) / (2 m) (C_(D, R) A_R + C_(D, A) A_A overline(u)) overline(v)) $

and

$ B_c = mat((partial dot(h) (t)) / (partial u); (partial dot(v) (t)) / (partial u)) = mat(0; - (rho (overline(h))) / (2 m) (C_(D, A) A_A) overline(v)^2). $

Thus we now have the approximation

$ bold(dot(x)) approx f(overline(x), overline(u)) + A_c (x - overline(x)) + B_c (u - overline(u)). $
