# Rocket airbrake MPC using linearized dynamics and OSQP

## 1. Variables and units

| Symbol | Meaning | Units |
|---|---|---|
| $t$ | Time since launch | s |
| $h$ | Altitude above sea level (ASL) | m |
| $h_\mathrm{AGL}=h-h_\mathrm{elev}$ | Altitude above ground level | m |
| $v=\dot h$ | Vertical velocity, positive upward | m/s |
| $m(t)$ | Rocket mass (RocketPy `rocket.total_mass(t)`) | kg |
| $g$ | Gravitational acceleration | m/s² |
| $\rho(h)$ | Atmospheric mass density (RocketPy `environment.density(h)`) | kg/m³ |
| $\rho'(h)$ | Density derivative with respect to ASL altitude | kg/m⁴ |
| $C_{D,R}$ | Simplified body drag coefficient | dimensionless |
| $A_R$ | Rocket reference area | m² |
| $C_{D,A}$ | Airbrake coefficient at full effective area | dimensionless |
| $A_A$ | Full effective airbrake area | m² |
| $u=\alpha$ | Airbrake deployment fraction | 0 to 1 |
| $\Delta t$ | Prediction time step | s |
| $N$ | Number of optimized deployment intervals | integer |
| $r$ | Maximum deployment rate | 1/s |
| $\hat h$ | Target apogee ASL | m |
| $\bar x,\bar u$ | Nominal state and deployment for linearization | mixed |
| $S_k$ | Derivative of state at step $k$ with respect to deployment vector | mixed |
| $w_A,w_D,w_R$ | Apogee, smoothness, regularization weights | chosen after normalization |
| $H_s$ | Altitude-error normalization scale | m |

In code, `target_apogee_agl` is converted to ASL using `environment.elevation`. `rocket_area = pi * 0.0762**2` treats **0.0762 m as a radius**; if it is a diameter, replace it with `pi * (0.0762/2)**2`.

## 2. Nonlinear coast model

Assume thrust has ended, vertical ascent ($v>0$), and no wind in the simplified prediction model. Gravity and aerodynamic drag act downward:

$$m\dot v=-mg-\tfrac12\rho(h)[C_{D,R}A_R+C_{D,A}A_Au]v^2.$$

To handle negative velocity in a numerical routine, use $v|v|$ instead of $v^2$:

$$\boxed{\dot h=v,\quad \dot v=-g-\frac{\rho(h)}{2m(t)}[C_{D,R}A_R+C_{D,A}A_Au]v|v|.}$$

Define $x=[h,v]^T$ and $f(x,u,t)=[v,\dot v]^T$. The simple discrete predictor is forward Euler:

$$x_{k+1}=F_k(x_k,u_k)=x_k+\Delta t f(x_k,u_k,t_k).$$

**Model consistency:** RocketPy's full six-degree-of-freedom flight simulation may include Mach-dependent body drag, winds, changing orientation, and other effects that this vertical predictor omits. The simplified body $C_D$ and airbrake drag curve should be calibrated against the actual RocketPy configuration. `environment.pressure(h)` is *not* air density.

## 3. Linearize the rocket dynamics

At nominal state $\bar x_k=[\bar h_k,\bar v_k]^T$ and nominal control $\bar u_k$, use a first-order Taylor approximation:

$$f(x,u,t_k)\approx f(\bar x_k,\bar u_k,t_k)+A_{c,k}(x-\bar x_k)+B_{c,k}(u-\bar u_k).$$

Here $A_{c,k}=\partial f/\partial x$ and $B_{c,k}=\partial f/\partial u$ evaluated at the nominal values. Set $K=C_{D,R}A_R+C_{D,A}A_A\bar u_k$. During upward coast, $\bar v_k>0$, so:

$$\boxed{A_{c,k}=\begin{bmatrix}0&1\\-\frac{\rho'(\bar h_k)K\bar v_k^2}{2m(t_k)}&-\frac{\rho(\bar h_k)K\bar v_k}{m(t_k)}\end{bmatrix}}$$

$$\boxed{B_{c,k}=\begin{bmatrix}0\\-\frac{\rho(\bar h_k)C_{D,A}A_A\bar v_k^2}{2m(t_k)}\end{bmatrix}.}$$

The code computes $\rho'(h)$ by a centered finite difference of RocketPy density. The mass is treated as known at each prediction time, not as a state variable.

Euler discretization yields

$$\boxed{A_{d,k}=I+\Delta t A_{c,k},\quad B_{d,k}=\Delta t B_{c,k}.}$$

In absolute coordinates the local affine model is

$$x_{k+1}\approx A_{d,k}x_k+B_{d,k}u_k+c_k,$$

$$c_k=F_k(\bar x_k,\bar u_k)-A_{d,k}\bar x_k-B_{d,k}\bar u_k.$$

In deviation coordinates $\delta x_k=x_k-\bar x_k$, $\delta u_k=u_k-\bar u_k$, assuming the nominal trajectory follows the same Euler predictor:

$$\boxed{\delta x_{k+1}=A_{d,k}\delta x_k+B_{d,k}\delta u_k.}$$

This is **linear time-varying (LTV)** dynamics: the matrices change with the nominal trajectory but remain constant *within one QP solve*.

## 4. Build the condensed state sensitivities

The control vector is $U=[u_0,\ldots,u_{N-1}]^T$. At the measured initial state, $\delta x_0=0$. Define

$$S_k=\frac{\partial x_k}{\partial U}\in\mathbb R^{2\times N},\qquad S_0=0.$$

From the discrete linearized dynamics,

$$\boxed{S_{k+1}=A_{d,k}S_k+B_{d,k}e_k^T,}$$

where $e_k$ is the $k$-th standard basis vector in $\mathbb R^N$. Thus $S_N$ maps deployment changes to changes in terminal altitude and velocity:

$$x_N(U)\approx\bar x_N+S_N(U-\bar U).$$

This requires one nominal trajectory propagation and one $2\times N$ sensitivity propagation, rather than re-running a full trajectory for each decision variable.

## 5. Predict apogee after the optimization horizon

A short horizon often ends *before* apogee. Define a terminal-to-apogee map

$$\Phi(h_N,v_N,u_{N-1},t_N)=$$

"altitude at $v=0$ when the nonlinear coast predictor is continued from $(h_N,v_N)$, holding the last deployment fixed." The code integrates until vertical velocity crosses zero and interpolates the final fractional timestep using constant acceleration over that final interval:

$$\tau=-v/a,\qquad h_\mathrm{apo}\approx h+v\tau+\tfrac12a\tau^2.$$

The hold-last-command assumption is an approximation: it does not enforce actuator rate limits after the horizon. A longer prediction horizon or a terminal policy with explicit rate limits would improve this.

Linearize the terminal map at $(\bar h_N,\bar v_N,\bar u_{N-1})$:

$$\Phi\approx\bar\Phi+\Phi_h\delta h_N+\Phi_v\delta v_N+\Phi_u\delta u_{N-1}.$$

The terminal derivatives $\Phi_h,\Phi_v,\Phi_u$ are computed by **six additional coast rollouts** (two for each of $h,v,u$, plus the nominal evaluation, seven total evaluations). The resulting deployment sensitivity is

$$\boxed{g^T=\begin{bmatrix}\Phi_h&\Phi_v\end{bmatrix}S_N+\Phi_u e_{N-1}^T.}$$

Therefore

$$\boxed{h_\mathrm{apo}(U)\approx\bar\Phi+g^T(U-\bar U)=b+g^TU,\quad b=\bar\Phi-g^T\bar U.}$$

If the nominal horizon itself crosses apogee, the code stops the nominal rollout and uses an approximate terminal-height sensitivity. This is **less accurate at the crossing event**, because the event time also changes with the controls. The controller stops issuing new optimized commands below `min_control_velocity` to reduce the impact, but this is not a complete treatment of event-time sensitivities.

## 6. Quadratic cost

Let the target ASL apogee be $\hat h=h_{\mathrm{elev}}+h_{\mathrm{target,AGL}}$. Normalize the predicted apogee error using a scale $H_s$ (100 m by default):

$$\epsilon(U)=\frac{b+g^TU-\hat h}{H_s}=e_0+G^TU,$$

$$e_0=\frac{b-\hat h}{H_s},\qquad G=\frac{g}{H_s}.$$

Let $u_{-1}$ be the current actuator command and define the difference matrix

$$D=\begin{bmatrix}1&0&\cdots&0\\-1&1&\cdots&0\\\vdots&\ddots&\ddots&\vdots\\0&\cdots&-1&1\end{bmatrix},\qquad d=[u_{-1},0,\ldots,0]^T.$$

Then $DU-d$ contains $[u_0-u_{-1},u_1-u_0,\ldots]^T$. Minimize

$$\boxed{J(U)=w_A(e_0+G^TU)^2+w_D\|DU-d\|_2^2+w_R\|U-\bar U\|_2^2.}$$

The final term is a local regularizer; it discourages large changes from the linearization trajectory. Expansion into OSQP's $\tfrac12U^TPU+q^TU$ form gives

$$\boxed{P=2\left(w_AGG^T+w_DD^TD+w_RI\right)}$$

$$\boxed{q=2\left(w_Ae_0G-w_DD^Td-w_R\bar U\right).}$$

For nonnegative weights, $P\succeq0$; for $w_R>0$, $P\succ0$. Terms independent of $U$ are discarded.

## 7. Linear constraints

Deployment range:

$$0\le u_k\le1.$$

Slew-rate limit:

$$-r\Delta t_k\le u_k-u_{k-1}\le r\Delta t_k.$$

The **first** step uses the actual controller interval $\Delta t_0=1/f_\mathrm{controller}$, while subsequent steps use the prediction interval. Set $R=[r\Delta t_0,r\Delta t,\ldots]^T$. In OSQP form:

$$\boxed{l\le A U\le u,\quad A=\begin{bmatrix}I\\D\end{bmatrix},\quad l=\begin{bmatrix}0\\d-R\end{bmatrix},\quad u=\begin{bmatrix}1\\d+R\end{bmatrix}.}$$

Here the $u$ in the last display denotes OSQP's **upper-bound vector**, not the scalar airbrake control input.

## 8. Receding-horizon control loop

1. Read RocketPy state `[x, y, z, vx, vy, vz, ...]`, taking $h=z$ (ASL) and $v=v_z$.
2. Wait until motor burnout; do not optimize during powered ascent. Stop optimizing near apogee.
3. Shift the previous optimized deployment schedule by one step (or start with constant current deployment).
4. Roll out the nominal nonlinear model over $N$ steps; calculate $A_{d,k}$, $B_{d,k}$, and $S_N$.
5. Roll out the nonlinear terminal coast to apogee and differentiate the terminal map.
6. Form $P,q,A,l,u$; update the existing OSQP workspace; warm-start the next solve.
7. Check OSQP's status; on success, command only the first optimized deployment; clamp to the actuator bounds and rate limit.
8. Repeat at the next controller sample.

**Why OSQP can solve this:** the *original* dynamics and terminal apogee map are nonlinear, but their first-order approximations make the apogee estimate affine in $U$. Squared affine error and squared deployment differences give a convex QP. The solve is only locally accurate; a single QP per tick is not a globally optimal nonlinear MPC solve.

## 9. Assumptions, limitations, and verification

- **Dependencies:** SciPy is used for *sparse matrix construction*, not for nonlinear optimization; OSQP solves the QP. The code does not require SciPy `minimize`.
- **Solver reuse:** The OSQP instance is created once. The upper-triangular $P$ sparsity pattern and constraint matrix $A$ stay fixed; the code updates $P$ values, $q$, $l$, and $u$ at every tick. OSQP warm-starts from the previous solution.
- **Prediction:** Forward Euler, vertical motion, simple constant drag coefficients, and finite-difference density slope; real RocketPy dynamics may differ.
- **Terminal policy:** The final command is held beyond the horizon; this is a modeling approximation and can violate physical slew constraints for hypothetical terminal scenarios.
- **Near-apogee:** A trajectory crossing the $v=0$ event inside the horizon has only an approximate sensitivity; investigate event-time differentiation or a shorter horizon.
- **Optimization:** Only one convex QP is solved per tick. Consider trust-region bounds or multiple sequential QP iterations if the linearization error is large.
- **Failure:** On solver failure the prototype holds the previous command. Flight software requires a defined fault-response policy and independent safety logic.
- **Actuator:** Actual position and actuator dynamics are not modeled. Use measured deployment rather than commanded deployment when available.
- **Testing:** Compare nonlinear RocketPy apogee for zero, full, and MPC-controlled airbrakes; sweep target altitudes and mass/density/drag uncertainties; test solver statuses and worst-case execution time.
- **Embedded:** `numpy`, `scipy`, `rocketpy`, and the Python wrapper do not run on the MCU. Port the small dynamics/linearization and QP-data construction to C, then use OSQP's embedded C/code-generation workflow. Verify code generation against the exact installed OSQP version and fixed sparsity pattern. Benchmarks and hardware-in-the-loop validation are mandatory before flight.

## 10. Embedded C implementation

The port is implemented in `airbrakes/src/mpc.c`, with its public API in
`airbrakes/include/mpc.h`. `simulations/codegen.py` generates the actual MPC
workspace and OSQP 1.1.3 sources under `airbrakes/lib/osqp_mpc`; the older
two-variable example remains excluded from the firmware build.

See [the embedded project README](../airbrakes/README.md) for generation,
PlatformIO builds, the state/configuration API, density-table preparation,
fault behavior, and board validation. Run `simulations/test_embedded_mpc.py`
with the simulations Python environment and native GCC to compare the actual C
controller against Python, including intermediate QP data and repeated ticks.
The port uses configurable constant coast mass and an altitude-density table or
callback, a fixed 50-iteration rho adaptation interval, and double precision.
