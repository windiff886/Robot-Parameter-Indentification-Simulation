# Robot Dynamics Parameter Identification System Unified Manual

> This document unifies and consolidates content within the `doc/` directory regarding dynamics modeling, excitation trajectories, data sampling, parameter identification, algorithm evaluation, and debugging insights. Reading this manual provides a comprehensive understanding of the mathematical models, engineering pipeline, and implementation boundaries of the current repository. Earlier documents are retained as archival references, while this manual serves as the primary technical reference moving forward.

This document synthesizes the following key topics:

- Dynamics equations and the physical interpretation of MuJoCo torque inputs
- Excitation trajectory generation and safety validation
- Linear regressor identification and nonlinear friction joint identification
- Evaluation methodologies for simulation and experimental modes
- Common numerical issues, debugging lessons, and engineering limitations

---

## 1. Objectives and Project Scope

### 1.1 Project Objectives

This project implements a robot dynamics parameter identification system based on **MuJoCo + Pure C++ + CMake**. Rather than focusing solely on trajectory tracking or simulation in isolation, its goal is to establish a complete closed loop:

1. Generate safe, highly excitative joint trajectories;
2. Execute trajectories in MuJoCo and record synchronous joint data;
3. Construct dynamics regressor matrices from recorded data;
4. Estimate dynamics parameters using multiple identification algorithms;
5. Evaluate torque prediction errors on an independent validation set;
6. Analyze parameter interpretability, numerical stability, and model limitations.

### 1.2 Supported Robot Platforms

The repository currently supports two robot platforms:

- `panda`: 7-DOF Franka Emika Panda
- `piper`: 6-DOF Piper robotic arm

Both platforms share the same overall experiment pipeline, but differ in:

- Degrees of freedom (DOF);
- MuJoCo XML model definitions;
- Number of rigid body parameters;
- Regressor matrix construction implementations.

### 1.3 System Overview

The core data flow in the current branch is structured as follows:

```text
config/*.yaml
    |
    v
run_experiment
    |
    +--> ForceController
    |        |
    |        +--> Generate excitation trajectory, check safety, output joint torques
    |
    +--> PandaSimulator / Piper Simulator
             |
             +--> Step MuJoCo simulation
             +--> Record q, qd, qdd, tau
             v
        data/benchmark_data.csv
             |
             v
          identify
             |
             +--> Data preprocessing
             +--> Construct observation matrix W
             +--> Solve parameters
             +--> Validation set evaluation
             v
       results/identification.yaml
```

From an architectural standpoint, the system is partitioned into five layers:

1. **Configuration Layer**: Experiment settings, identification configuration, robot model paths;
2. **Trajectory Layer**: Excitation trajectory generation and safety validation;
3. **Simulation Layer**: MuJoCo state stepping and synchronous data logging;
4. **Identification Layer**: Observation matrix assembly, parameter estimation, residual evaluation;
5. **Diagnostic Layer**: Diagnostic reports, model comparison, regressor consistency verification.

---

## 2. Notation, Coordinate Systems, and Data Formats

### 2.1 Basic Notation

Let the robot degrees of freedom be $n$, and the number of recorded samples be $K$. For any time instant $t_k$, define:

- $q(t_k) \in \mathbb{R}^n$: Joint positions
- $\dot{q}(t_k) \in \mathbb{R}^n$: Joint velocities
- $\ddot{q}(t_k) \in \mathbb{R}^n$: Joint accelerations
- $\tau(t_k) \in \mathbb{R}^n$: Joint torques

During offline identification, samples are stacked across time into matrices:

$$
Q =
\begin{bmatrix}
q(t_1) & q(t_2) & \cdots & q(t_K)
\end{bmatrix}
\in \mathbb{R}^{n \times K}
$$

$$
\dot{Q}, \ddot{Q} \in \mathbb{R}^{n \times K}
$$

The measured torque vector is unrolled across the joint dimensions as:

$$
\tau_{meas} =
\begin{bmatrix}
\tau(t_1) \\
\tau(t_2) \\
\vdots \\
\tau(t_K)
\end{bmatrix}
\in \mathbb{R}^{nK}
$$

### 2.2 Coordinate Frames and Parameter Definitions

In this project, dynamic parameters are not formulated using classical DH center-of-mass (COM) conventions, but are defined with respect to the **MuJoCo body-local coordinate frame**. For each rigid body, the standard inertial parameter vector is expressed as:

$$
\theta_{body} =
[m, mx, my, mz, I_{xx}, I_{xy}, I_{xz}, I_{yy}, I_{yz}, I_{zz}]^T
$$

Where:

- $m$: Body mass
- $(mx,my,mz)=m \cdot c_{local}$: First mass moments
- $c_{local}$: Position of body center of mass (COM) in the body-local frame
- $I_{**}$: Components of the inertia tensor expressed at the body origin

This formulation differs from parameterizations that define inertia tensors at the center of mass. When comparing parameters across models or literature, the reference frame and origin must be unified; otherwise, physically equivalent parameters will appear numerically disparate.

### 2.3 Data File Format

Experiment logs are stored in CSV format with typical columns:

```text
time, q0..q(n-1), qd0..qd(n-1), [qdd0..qdd(n-1)], tau0..tau(n-1)
```

The simulation recorder logs:

- `qpos`
- `qvel`
- `qacc`
- `data->ctrl`

Thus, in this repository, `tau` represents the **commanded torque written to the actuator control input**, rather than an idealized decomposition term extracted directly from MuJoCo internal variables.

---

## 3. Dynamics Equations in MuJoCo and Control Input Semantics

### 3.1 Generalized Coordinate Dynamics in MuJoCo

MuJoCo formulates and solves dynamics in generalized coordinates according to:

$$
M(q)\ddot{q} + qfrc_{bias}(q,\dot{q})
= qfrc_{actuator} + qfrc_{passive} + qfrc_{applied} + qfrc_{constraint}
$$

Where:

- $M(q)$: Generalized mass/inertia matrix
- $qfrc_{bias}$: Bias force, typically encompassing Coriolis, centrifugal, and gravitational effects
- $qfrc_{actuator}$: Generalized forces exerted by actuators
- $qfrc_{passive}$: Passive forces, such as joint damping, spring stiffness, and friction
- $qfrc_{constraint}$: Constraint forces arising from contacts, joint limits, and equality constraints

Neglecting external perturbations and constraint forces, and expanding the bias term, yields the familiar open-chain manipulator equation:

$$
M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q)
= \tau_{act} + \tau_{passive}
$$

### 3.2 Relationship Between Control Input `ctrl` and Generalized Forces

The simulation program assigns commanded values to `data->ctrl`. For joint `motor` actuators, this maps directly to generalized joint forces. If the transmission gear is identity and dynamic lag is absent, the relationship approximates:

$$
\tau_{act} \approx \tau_{ctrl}
$$

However, strictly speaking, discrepancies may arise due to:

- Actuator effort limits;
- Actuator gear ratios or transmissions;
- Tendon routing;
- Passive terms and constraint forces;
- Numerical characteristics of implicit integrators.

Consequently, having full access to the XML model does not automatically guarantee that logged `tau` matches idealized inverse dynamics.

### 3.3 Simplified Dynamics Formulation in This Repository

Based on the identification implementation, the system adopts the following engineering model:

$$
\tau = M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) - D\dot{q}
$$

Where:

- $D = \mathrm{diag}(d_1,\dots,d_n)$ is the joint viscous damping matrix;
- `armature` contributes directly to the diagonal of the generalized inertia matrix;
- Linear damping is explicitly included in the identification model;
- Nonlinear friction is modeled separately in `NLS_FRICTION`.

This formulation represents the engineering approximation used for identification, designed to match MuJoCo while maintaining tractability.

### 3.4 Impact of Constraints and Collisions

If an excitation trajectory encounters:

- Self-collisions;
- Environment collisions;
- Joint limit violations;
- Forceful equality constraint engagement;

Then `qfrc_constraint` introduces substantial non-modeled forces. Fitting the data using:

$$
\tau \approx M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) - D\dot{q}
$$

causes constraint forces to be erroneously absorbed into inertial and friction parameters, corrupting the estimates. Therefore, trajectory safety validation is not an optional auxiliary feature, but an essential prerequisite for valid identification.

---

## 4. Parameter Vectors and Regressor Matrices

### 4.1 Linear Parameter Vector

Using 10 standard inertial parameters per body, the total parameter vector comprises three components:

1. Rigid body inertial parameters
2. `armature` parameters
3. `damping` parameters

For $N_b$ identified rigid bodies, the linear parameter vector is:

$$
\beta_{lin} =
\begin{bmatrix}
\theta_{body,1} \\
\theta_{body,2} \\
\vdots \\
\theta_{body,N_b} \\
a_1,\dots,a_n \\
d_1,\dots,d_n
\end{bmatrix}
$$

Specifically:

- `piper`: $N_b = 6$, yielding $6 \times 10 + 6 + 6 = 72$ linear parameters
- `panda`: $N_b = 8$, yielding $8 \times 10 + 7 + 7 = 94$ linear parameters

### 4.2 Linear Regression Model

Under MuJoCo body-local parameter definitions, joint torques depend linearly on the parameters:

$$
\tau =
Y(q,\dot{q},\ddot{q})\theta
+ A\ddot{q}
- D\dot{q}
$$

Consolidating terms:

$$
\tau = W(q,\dot{q},\ddot{q}) \, \beta_{lin}
$$

Where:

- $\theta$: Concatenated rigid body inertial parameters
- $A=\mathrm{diag}(a_1,\dots,a_n)$: Armature
- $D=\mathrm{diag}(d_1,\dots,d_n)$: Damping
- $W$: Complete observation matrix

For a single sample $k$:

$$
\tau^{(k)} = W^{(k)} \beta_{lin}
$$

Stacking all $K$ samples yields:

$$
\underbrace{
\begin{bmatrix}
\tau^{(1)} \\
\tau^{(2)} \\
\vdots \\
\tau^{(K)}
\end{bmatrix}
}_{\tau_{meas}}
=
\underbrace{
\begin{bmatrix}
W^{(1)} \\
W^{(2)} \\
\vdots \\
W^{(K)}
\end{bmatrix}
}_{W}
\beta_{lin}
$$

### 4.3 Mathematical Derivation of Single Body Regressor Block

For a given rigid body, define:

- $R$: Rotation matrix from body-local frame to world frame
- $J_v, J_\omega$: Linear and angular velocity Jacobians evaluated at the body origin
- $\dot{J}_v, \dot{J}_\omega$: Corresponding time derivatives

Kinematics are first computed in the world frame:

$$
a_{world} = J_v \ddot{q} + \dot{J}_v \dot{q}
$$

$$
\omega_{world} = J_\omega \dot{q}
$$

$$
\alpha_{world} = J_\omega \ddot{q} + \dot{J}_\omega \dot{q}
$$

Transforming to the body-local frame:

$$
a_{local} = R^T a_{world}, \quad
\omega_{local} = R^T \omega_{world}, \quad
\alpha_{local} = R^T \alpha_{world}
$$

Gravity in the local frame is:

$$
g_{local} = R^T g_{world}
$$

Defining the net acceleration term:

$$
b_{local} = a_{local} - g_{local}
$$

The net force and torque acting on the body in its local frame satisfy:

$$
f_{local} = m b_{local} + K \, mc_{local}
$$

$$
n_{local}
= [mc_{local}]_\times b_{local}
+ I_{origin}\alpha_{local}
+ [\omega_{local}]_\times I_{origin}\omega_{local}
$$

Where:

$$
K = [\alpha_{local}]_\times + [\omega_{local}]_\times [\omega_{local}]_\times
$$

The contribution of this body to the generalized joint torques is:

$$
\tau_{body} = J_{v,local}^T f_{local} + J_{\omega,local}^T n_{local}
$$

Because this relationship is strictly linear with respect to:

$$
[m, mx, my, mz, I_{xx}, I_{xy}, I_{xz}, I_{yy}, I_{yz}, I_{zz}]
$$

the single-body regressor block is formulated as:

$$
Y_{body}(q,\dot{q},\ddot{q}) \in \mathbb{R}^{n \times 10}
$$

Concatenating regressor blocks across all bodies yields the inertial regressor matrix.

### 4.4 Regressor Columns for `armature` and `damping`

For each joint $i$:

- Armature corresponds to $\ddot{q}_i$
- Damping corresponds to $-\dot{q}_i$

Hence, the auxiliary regressor columns are:

$$
Y_{armature}(i,i) = \ddot{q}_i
$$

$$
Y_{damping}(i,i) = -\dot{q}_i
$$

### 4.5 Identifiability and Rank Deficiency

Even when dynamic equations are exact, not all parameters are independently identifiable. If:

- Certain link geometries remain unexcited by the chosen trajectory;
- Certain inertia components appear only in linear combinations;
- Regressor columns exhibit near-linear dependence;

Then the observation matrix $W$ will have null or near-null singular values. Different parameter vectors can produce nearly indistinguishable torque predictions—explaining why a fit can yield low torque residuals despite physically implausible parameter values.

---

## 5. Excitation Trajectory Design

### 5.1 Finite Fourier Series Trajectories

To excite position, velocity, and acceleration channels simultaneously, a finite Fourier series is employed for each joint:

$$
q_i(t) = q_{i,0} +
\sum_{k=1}^{N}
\left(
\frac{a_{i,k}}{\omega_f k}\sin(\omega_f k t)
- \frac{b_{i,k}}{\omega_f k}\cos(\omega_f k t)
\right)
$$

The corresponding velocities and accelerations are:

$$
\dot{q}_i(t) =
\sum_{k=1}^{N}
\left(
a_{i,k}\cos(\omega_f k t)
+ b_{i,k}\sin(\omega_f k t)
\right)
$$

$$
\ddot{q}_i(t) =
\sum_{k=1}^{N}
\omega_f k
\left(
b_{i,k}\cos(\omega_f k t)
- a_{i,k}\sin(\omega_f k t)
\right)
$$

Where:

- $q_{i,0}$: Joint initial offset
- $N$: Number of harmonics
- $\omega_f = \frac{2\pi}{T}$: Fundamental frequency
- $T$: Trajectory period

### 5.2 Trajectory Period and Torque Amplitude

The fundamental frequency is:

$$
\omega_f = \frac{2\pi}{T}
$$

Consequently:

- Velocity amplitude scales proportionally to $\omega_f$;
- Acceleration amplitude scales proportionally to $\omega_f^2$;
- In inertia-dominated regimes, required drive torque scales approximately with $\omega_f^2$.

Increasing the period $T$ reduces peak accelerations and torque demands, preventing motor saturation while trading off excitation strength. A deliberate compromise between safety and observability is required.

### 5.3 Initial Boundary Constraints

To guarantee smooth startup from rest, trajectories must satisfy:

$$
q_i(0) = q_{i,0}, \quad \dot{q}_i(0)=0, \quad \ddot{q}_i(0)=0
$$

Substituting $t=0$ into the Fourier expressions yields:

$$
\sum_{k=1}^{N} -\frac{b_{i,k}}{\omega_f k} = 0
$$

$$
\sum_{k=1}^{N} a_{i,k} = 0
$$

$$
\sum_{k=1}^{N} \omega_f k \, b_{i,k} = 0
$$

These constraints prevent initial velocity or acceleration discontinuities.

### 5.4 Trajectory Conditioning and Observability

Trajectory optimization aims to maximize linear independence across columns of $W$, measured via the condition number:

$$
\mathrm{cond}(W) = \frac{\sigma_{max}(W)}{\sigma_{min}(W)}
$$

Lower condition numbers reduce sensitivity to noise, whereas ill-conditioned matrices amplify minute noise into substantial parameter estimation errors.

---

## 6. Trajectory Safety Mechanisms

### 6.1 Generate-Check-Retry Strategy

Random Fourier coefficients frequently produce infeasible motions. The system applies a **Generate-Check-Retry** workflow:

1. Generate candidate Fourier trajectory;
2. Perform comprehensive safety checks;
3. Retry with new random coefficients upon failure;
4. Scale down motion amplitude if repeated failures occur;
5. Terminate upon finding a valid trajectory or reaching maximum retries.

### 6.2 Joint Limit Verification

For every joint $i$ and sampled timestamp $t$:

$$
q_{min,i} \le q_i(t) \le q_{max,i}
$$

This ensures geometric compliance within physical joint boundaries.

### 6.3 Collision Detection

Offline collision checks are evaluated directly in MuJoCo. For candidate joint configuration $q(t)$:

1. Assign $q(t)$ to `mjData->qpos`;
2. Invoke `mj_fwdPosition` to update geometry transforms;
3. Inspect contact manifolds and active contacts;
4. If `ncon > 0`, reject trajectory as unsafe.

This approach achieves higher fidelity than simplified capsule models by reusing MuJoCo native meshes, contact generation, and constraint formulation.

### 6.4 Impact of Safety Violations on Identification

If a colliding trajectory were executed, logged torques would contain:

- Constraint reaction forces;
- Contact impulses;
- Limit enforcement torques;
- Saturation-induced control distortion.

These unmodeled phenomena violate the $W\beta=\tau$ assumption, introducing severe bias. Safety verification is therefore integral to identification fidelity.

---

## 7. Data Sampling and Preprocessing

### 7.1 Logged Channels

The simulator records:

- Timestamp `time`
- Joint positions `q`
- Joint velocities `qd`
- Joint accelerations `qdd`
- Commanded control torques `tau`

Simulation logs provide access to `qacc` directly from MuJoCo, avoiding errors inherent to numerical differentiation of velocities.

### 7.2 Acceleration Reconstruction

When `qdd` is missing from CSV data, central differences are applied:

$$
\ddot{q}(t_i) \approx
\frac{\dot{q}(t_{i+1}) - \dot{q}(t_{i-1})}{t_{i+1} - t_{i-1}}
$$

Boundary values are approximated using one-sided differences or replicated endpoints.

### 7.3 Outlier Filtering

To eliminate spurious acceleration spikes caused by differentiated noise, threshold filtering is enforced:

$$
\|\ddot{q}(t_i)\|_\infty < \ddot{q}_{max}
$$

The default threshold is:

$$
\ddot{q}_{max} = 10.0 \text{ rad/s}^2
$$

Filtering prevents localized anomalies from disproportionately biasing least-squares objectives.

### 7.4 Train-Validation Splitting

Logged trajectories are partitioned chronologically:

- Training set: First 80%
- Validation set: Remaining 20%

Parameters are estimated on the training set and evaluated on the unseen validation set, preventing optimistic bias in reported errors.

---

## 8. Linear Parameter Identification Problem

### 8.1 Objective Formulation

The standard linear identification problem minimizes the residual sum of squares:

$$
\min_{\beta_{lin}} \frac{1}{2}\|W\beta_{lin}-\tau_{meas}\|_2^2
$$

With Tikhonov regularization:

$$
\min_{\beta_{lin}}
\|W\beta_{lin}-\tau_{meas}\|_2^2
+ \lambda \|\beta_{lin}\|_2^2
$$

Where $\lambda > 0$ denotes the regularization parameter.

### 8.2 Rationale for Regularization

When $W$ is ill-conditioned, the normal equations:

$$
(W^T W)\beta = W^T \tau
$$

become numerically unstable. Even small residuals can yield erratic parameter drifts along poorly excited directions. Regularization:

$$
(W^T W + \lambda I)\beta = W^T \tau
$$

suppresses numerical explosion, stabilizing parameter estimates.

### 8.3 Core Identification Algorithms

#### OLS (Ordinary Least Squares)

Implemented with ridge regularization:

$$
\hat{\beta}_{OLS}
=
(W^T W + \lambda I)^{-1}W^T\tau
$$

Simple and efficient, but sensitive to outliers.

#### WLS (Weighted Least Squares)

Initial OLS residuals are used to weight joints inversely by noise variance:

$$
\hat{\beta}_{WLS}
=
\arg\min_\beta
\| \Sigma^{-1/2}(W\beta-\tau)\|_2^2
$$

Effective when noise variance differs across joints, though less robust to heavy-tailed outliers.

#### IRLS (Iteratively Reweighted Least Squares)

Employs Huber loss weights:

$$
w_i =
\begin{cases}
1, & |r_i| \le \delta \\
\frac{\delta}{|r_i|}, & |r_i| > \delta
\end{cases}
$$

Where $r_i$ is the residual and $\delta$ is estimated via Median Absolute Deviation (MAD). Iterative updates solve:

$$
\hat{\beta}^{(k+1)}
=
\arg\min_\beta
\|W_k^{1/2}(W\beta-\tau)\|_2^2 + \lambda\|\beta\|_2^2
$$

Provides superior robustness against intermittent outlier samples.

#### TLS (Total Least Squares)

Accounts for errors in both $W$ and $\tau$ via Singular Value Decomposition (SVD) of the augmented matrix $[W \; | \; \tau]$. Suited for errors-in-variables scenarios, though sensitive to severe rank deficiency.

#### EKF / RLS (Recursive Filtering)

Treats parameters as static states updated sequentially:

$$
\beta_{k+1} = \beta_k + K_k(y_k - H_k\beta_k)
$$

Well suited for recursive/online identification, though sensitive to prior covariance and noise tuning.

#### ML (Maximum Likelihood)

Minimizes quadratic residuals:

$$
\min_\beta \|W\beta-\tau\|_2^2
$$

solved via Levenberg-Marquardt. It operates on identical information as linear estimators.

#### CLOE (Closed-Loop Output Error)

Theoretically writes estimated parameters back into the model to minimize trajectory tracking errors in closed-loop simulation. Currently maintained as a placeholder in benchmark comparisons.

---

## 9. Nonlinear Friction Identification

### 9.1 Rationale for Nonlinear Friction Models

Linear viscous damping:

$$
\tau_f = D\dot{q}
$$

cannot capture:

- Static breakaway friction;
- Coulomb friction plateaus;
- Stribeck effects;
- Smooth velocity-reversal transitions.

A nonlinear friction model is integrated via `NLS_FRICTION`.

### 9.2 Joint Friction Model Formulation

For joint $j$, friction torque is modeled as:

$$
\tau_{f,j}(\dot{q}_j)
=
g_{1,j}\left(\tanh(g_{2,j}\dot{q}_j)-\tanh(g_{3,j}\dot{q}_j)\right)
+ g_{4,j}\tanh(g_{5,j}\dot{q}_j)
+ g_{6,j}\dot{q}_j
$$

Where:

- $g_1$: Stribeck amplitude
- $g_2, g_3$: Stribeck shape coefficients
- $g_4$: Coulomb friction level
- $g_5$: Coulomb transition steepness
- $g_6$: Viscous friction coefficient

### 9.3 Combined Parameter Vector

The combined parameter vector is:

$$
\Theta =
\begin{bmatrix}
\beta_{rigid+armature} \\
\theta_{f,1} \\
\theta_{f,2} \\
\vdots \\
\theta_{f,n}
\end{bmatrix}
$$

Where each joint friction sub-vector is:

$$
\theta_{f,j} = [g_{1,j}, g_{2,j}, g_{3,j}, g_{4,j}, g_{5,j}, g_{6,j}]^T
$$

In `NLS_FRICTION`, parameters include rigid body parameters, armature, and nonlinear friction (without overlapping linear damping).

### 9.4 Nonlinear Least Squares Objective

Predicted joint torques are:

$$
\hat{\tau}(q,\dot{q},\ddot{q};\Theta)
=
W_{base}(q,\dot{q},\ddot{q})\beta_{base}
- \tau_f(\dot{q};\theta_f)
$$

The optimization problem solves:

$$
\hat{\Theta}
=
\arg\min_{\Theta}
\sum_{k=1}^{K}
\left\|
\tau^{(k)}_{meas}
- \hat{\tau}(q^{(k)},\dot{q}^{(k)},\ddot{q}^{(k)};\Theta)
\right\|_2^2
$$

### 9.5 Positivity Constraints via Reparameterization

To enforce positivity on friction parameters, an exponential reparameterization is applied:

$$
g_i = \exp(z_i)
$$

Optimization is performed over unconstrained variables $z_i$, ensuring $g_i > 0$.

### 9.6 Multi-Start Levenberg-Marquardt Optimization

The nonlinear optimization pipeline:

1. Initializes rigid body and armature parameters via linear regression;
2. Approximates Coulomb and viscous friction from linear residuals;
3. Forms multi-start initializations across scaling factors;
4. Executes Levenberg-Marquardt optimization.

This two-stage procedure mitigates local minima pitfalls while leveraging analytical linear solutions.

---

## 10. Evaluation Methods

### 10.1 Simulation Mode Evaluation

In simulation where true parameters $\beta_{true}$ are known, direct error metrics include:

#### Absolute Error

$$
E_{abs,i} = |\beta_{est,i} - \beta_{true,i}|
$$

#### Relative Error

$$
E_{rel,i} =
\frac{|\beta_{est,i} - \beta_{true,i}|}{|\beta_{true,i}|}
\times 100\%
$$

#### Parameter Norm Error

$$
E_{norm} = \|\beta_{est} - \beta_{true}\|_2
$$

### 10.2 Experimental Mode Evaluation

When ground-truth parameters are unavailable, cross-validation is evaluated on held-out trajectories:

$$
\tau_{pred} = W_{val}\beta_{est}
$$

$$
r = \tau_{meas,val} - \tau_{pred}
$$

Key validation metrics:

#### Root Mean Square Error (RMSE)

$$
RMSE = \sqrt{\frac{1}{N}\sum_{i=1}^{N} r_i^2}
$$

#### Maximum Absolute Error

$$
E_{max} = \max_i |r_i|
$$

### 10.3 Distinction Between Low RMSE and Parameter Fidelity

Crucially:

$$
\text{Low Torque Residuals} \not \Rightarrow \text{Physically Correct Parameters}
$$

Discrepancies arise because:

1. $W$ may exhibit rank deficiency;
2. Multiple parameter sets can produce near-identical outputs over validation trajectories;
3. Logged `tau` reflects actuator command signals rather than pure inverse dynamics;
4. Damping, unmodeled friction, and contacts may mutually compensate;
5. Unconstrained least squares does not enforce physical consistency (mass positivity, positive-definite inertia tensors).

Parameters must be assessed across three criteria:

- **Predictive Performance**: Low validation RMSE;
- **Numerical Stability**: Well-bounded parameter norms;
- **Physical Plausibility**: Positive masses, valid inertia tensors, and realistic damping values.

---

## 11. Common Numerical Issues and Debugging Insights

### 11.1 Acceleration Estimation Spikes

Numerical differentiation amplifies high-frequency noise:

$$
\ddot{q}(t_i) \approx
\frac{\dot{q}(t_{i+1}) - \dot{q}(t_{i-1})}{2\Delta t}
$$

At 1 kHz sampling, minor velocity noise generates spurious accelerations exceeding hundreds of $\text{rad/s}^2$, distorting inertial regression.

### 11.2 Regressor Rank Deficiency

Near-zero singular values in $W$ cause severe ill-conditioning:

- Parameter values explode in magnitude;
- Minor noise causes sign reversals;
- Large divergences emerge between OLS, WLS, and TLS solutions;
- Torque RMSE remains largely unchanged.

### 11.3 Sign Conventions

Consistent sign conventions across modules are critical:

$$
\tau = M\ddot{q} + C\dot{q} + g - D\dot{q}
$$

Inconsistent definitions between simulation logging and regressor construction cause negative identified damping values despite low prediction residuals.

### 11.4 Commanded Torque (`ctrl`) Semantics

Logged torques represent `data->ctrl` rather than isolated terms like `qfrc_bias` or `qfrc_inverse`. Identification estimates effective dynamics under control input, encompassing actuator transmissions and scaling.

### 11.5 Standard vs Base Parameter Sets

This system identifies full inertial parameter vectors rather than symbolic base parameter sets. This simplifies correspondence with MuJoCo XML files, but increases susceptibility to unidentifiable directions.

---

## 12. Code Entry Points and File Mapping

### 12.1 Executables

- `run_experiment`: Executes excitation trajectories and logs CSV data
- `identify`: Loads CSV data and executes benchmark identification algorithms
- `mujoco_identify`: Rapid identification using MuJoCo kinematics
- `dynamics_diagnostic`: Compares analytical dynamics terms with logged data
- `model_comparison`: Evaluates differences across dynamics models
- `regressor_test`: Validates regressor matrix consistency with MuJoCo dynamics

### 12.2 Key Source Locations

- `src/app/run_experiment.cpp`: Experiment scheduler entry point
- `src/force_node/src/force_controller.cpp`: Trajectory generation, PD control, safety validation
- `src/sim_com_node/src/panda_simulator.cpp`: MuJoCo stepping and CSV logging
- `src/identification/src/main.cpp`: Offline identification CLI entry point
- `src/identification/src/identification.cpp`: Preprocessing and identification pipeline
- `src/identification/src/algorithms.cpp`: Implementations of identification algorithms
- `src/identification/src/robot/*regressor.cpp`: Regressor matrix construction
- `src/identification/src/robot/*dynamics.cpp`: Analytical dynamics computations

### 12.3 Key Configuration Files

- `config/experiment.yaml`: Experiment execution settings
- `config/identification.yaml`: Identification configuration
- `config/force_controller_node.yaml`: Controller gains and trajectory parameters
- `config/panda_sim_node.yaml`: Simulator parameters

---

## 13. Recommended Reading Order

For onboarding to the codebase:

1. Sections 1–3: System scope and MuJoCo dynamics semantics;
2. Section 4: Parameter vectors and regressor derivations;
3. Sections 5–7: Trajectory generation, safety checks, and data collection;
4. Sections 8–9: Linear and nonlinear identification algorithms;
5. Sections 10–11: Evaluation metrics and debugging lessons;
6. Section 12: Source code and executable mapping.

---

## 14. Conclusion

The repository establishes an end-to-end parameter identification pipeline. Distinguishing between three fundamental concepts is essential:

1. **Known Dynamics Equations**: Enables correct regressor matrix formulation;
2. **Consistent Data Logging**: Aligns target measurements with model outputs;
3. **Parameter Identifiability**: Guarantees unique, stable recovery of physical parameters from data.

High predictive quality translates to physical parameter confidence only when all three conditions are satisfied. This manual unifies these principles into a coherent technical reference.
