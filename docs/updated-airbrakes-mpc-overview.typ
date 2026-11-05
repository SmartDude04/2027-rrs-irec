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

= Problem Description

Although the contents in `airbrakes-mpc-overview` had logical reasoning, they failed to account for an important aspect of quadratic programming: Functions in terms of the variables we wish to minimize must be _linear_. This means our previous cost function

$ J = w_A (hat(h) - h)^2 + w_D sum_(i = 1)^N (alpha_i - alpha_(i - 1))^2 $

is a valid cost function but not a valid QP problem.

Given a dynamical state $arrow(x)_(i + 1) = A arrow(x)_i + B arrow(u)_i$ where $arrow(u)_i$ is external input at timestep $i$ and $A$ and $B$ are matrices, a QP algorithm minimizes

$ 1/2 arrow(x)^T P arrow(x) + arrow(q)^T arrow(x)  $

subject to

$ l <= A arrow(x) <= u $

where $P$ and $A$ are matrices, $arrow(q)$ is a vector, and $l$ and $u$ are lower and upper bounds respectively.

= Solution

To solve this problem, we approximate the function $h(arrow(alpha))$ with a first-order Taylor series about some known $arrow(alpha)_0$. 

Let $arrow(alpha)_0$ be some airbrake opening amount guaranteed to reach apogee for the remaining flight time. At the beginning of the flight, it may simply be $arrow(0)$, but as the flight progresses and data becomes clearer, it is likely $arrow(alpha)_0$ is the previous airbrake opening amounts given by the QP algorithm. To find a Taylor approximation about $arrow(alpha)_0$, vary each $(arrow(alpha)_0)_i$ by a small amount $epsilon$ of order $0.001 - 0.01$. Then compute the change in $h$ when only one $(arrow(alpha)_0)_i$ is modified. Let $arrow(alpha)_0 '$ by the new gradient value (recomputed at each $i$). The value of the gradient at position $i$ is thus

$ (gradient h(arrow(alpha)_0))_i = (h(arrow(alpha)_0 ') - h(arrow(alpha)_0))/epsilon $

Repeat by changing each $(arrow(alpha)_0)_i$ by $epsilon$. Now, the height of the rocket at some $arrow(alpha)$ close to $arrow(alpha)_0$ is given by

$ h(arrow(alpha)) approx h(arrow(alpha)_0) + gradient h(arrow(alpha)_0) (arrow(alpha) - arrow(alpha)_0) $

In other words, our cost function is now linear: We wish to minimize

$ J = (gradient h(arrow(alpha)_0) (arrow(alpha) - arrow(alpha)_0) + h(arrow(alpha)_0) - hat(h))^2 + "stickout weight"^2 + "smoothness weight"^2 $

Let $beta = h(arrow(alpha)_0) - gradient h(arrow(alpha)_0) arrow(alpha)_0 - hat(h)$ which is constant. Now,

$ J = (gradient h(arrow(alpha)_0) arrow(alpha) + beta)^2 + "stickout weight"^2 + "smoothness weight"^2 $

"stickout weight" wishes to minimize the amount that the airbrakes are out far. This reduces the likelyhood of a plan to have the airbrake open lots for a while and then not a lot. It can be given as

$ "stickout weight"^2 = arrow(alpha)^T S arrow(alpha) $

where $S$ is some diagonal matrix of weights for each timeslot. "smoothness weight" minimizes the difference between subsequent timesteps. It uses the Difference matrix, defined as

$ D = mat(1, 0, 0, 0, 0; -1, 1, 0, 0, 0; 0, -1, 1, 0, 0; 0, 0, -1, 1, 0; 0, 0, 0, -1, 1; , , , , , dots.down) $

Thus, "smoothness weight" is defined a

$ "smoothness weight" = (D arrow(alpha) - vec(1, 0, 0, dots.v) (arrow(alpha)_(-1))_1)^T Q (D arrow(alpha)_i - vec(1, 0, 0, dots.v) (arrow(alpha)_(-1))_1) $

the extra vector term at the end adds the previous airbrake opening value and $Q$ is a diagonal matrix of weights for each timeslot.

== Converting to a Matrix Equation

Expanding all expressions in the cost function, we find

// $ "height term" = (h(arrow(alpha)) - hat(h))^2 + 2 (h(arrow(alpha)) - hat(h)) (gradient^T h(arrow(alpha)_0) (arrow(alpha) - arrow(alpha)_0)) + (arrow(alpha) - arrow(alpha)_0)^T (gradient h(arrow(alpha)_0)) (gradient^T h(arrow(alpha)_0)) (arrow(alpha) - arrow(alpha)_0) $

$ "height term" = arrow(alpha)^T gradient h(arrow(alpha)_0) gradient^T h(arrow(alpha)_0) arrow(alpha) + 2 beta gradient h(arrow(alpha)_0) arrow(alpha) $

$ "stickout weight"^2 = arrow(alpha)^T S arrow(alpha) $

$ "smoothness weight" = arrow(alpha)^T D^T Q D arrow(alpha)  $

Writing in QP algorithm form, we find

$ P &= 2(gradient h(arrow(alpha)_0) gradient^T h(arrow(alpha)_0) + S + D^T Q D)\
q &= 2(beta gradient h(arrow(alpha)_0) - arrow(alpha)_(-1) D^T Q vec(1, 0, 0, dots.v) $

Our constraint matrix $A$ is given by

$ A = mat(II_N, 0; 0, D_N) $

where $II_N$ and $D_n$ are identity and diagonal matrices of size $arrow(alpha)$. This allows for constraints on opening angles and angular rates.