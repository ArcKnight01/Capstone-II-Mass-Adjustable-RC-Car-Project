# EKF, Dynamics, and Odometry Math Explanation

This document explains the math currently implemented in `ExtendedKalmanFilter.py`, `DynamicsModel.py`, and `Odometry.py`, using the active values from the `config/` folder.

## 1. Frames, Signs, and Config Assumptions

From `config/frames.yaml` and the comments in the code:

- Body frame: `+X` forward, `+Y` left, `+Z` up
- World frame: local ENU, so `x = East`, `y = North`, `z = Up`
- Yaw `psi` is in radians
- Positive yaw is counter-clockwise from East
- Steering sign convention is currently `negative-left`

That steering convention means the logged steering angle is converted into the math sign used by the planar model with:

```text
delta_math = -delta_logged
```

So a left turn in the vehicle model is positive in the body-XY math sense, but the logged steering command is currently negative for left.

## 2. Config-Derived Parameters Used by the Model

From `config/vehicle.yaml`:

```text
mass m                             = 2.667 kg
wheelbase L                        = 0.3048 m
front CG distance a                = 0.159512 m
rear CG distance b                 = 0.140208 m
track width t                      = 0.3048 m
CG height h                        = 0.066309 m
yaw inertia I_z                    = 0.6 kg m^2    (placeholder / low confidence)
front cornering stiffness C_f      = 60.0 N/rad
rear cornering stiffness C_r       = 35.0 N/rad
longitudinal drag coefficient C_d  = 0.0
minimum safe longitudinal speed    = 0.3 m/s
```

The active tires are:

```text
front tire: S3 Soft Tires -> C_f = 60.0 N/rad
rear tire:  MC Hard Tires -> C_r = 35.0 N/rad
```

From `config/sensors.yaml` and `config/ekf.yaml`:

```text
GPS position sigma     = 2.5 m
GPS velocity sigma     = 0.05 m/s
IMU yaw sigma          = 0.026 rad
IMU yaw-rate sigma     = 0.0014 rad/s

model velocity sigma   = 1.5 m/s^2
model yaw-accel sigma  = 2.0 rad/s^2
accel bias walk sigma  = 0.05 m/s^3
gyro bias walk sigma   = 0.01 rad/s^2
```

From `config/ekf.yaml`, the current EKF state is:

```text
x =
[ p_e_m,
  p_n_m,
  yaw_rad,
  v_x_body_mps,
  v_y_body_mps,
  yaw_rate_radps,
  accel_bias_x_mps2,
  gyro_bias_z_radps ]^T
```

The dynamics plan in `config/dynamics.yaml` includes roll and pitch, but the current EKF implementation does not estimate them yet. They are still useful conceptually in the dynamics and odometry explanation below.

## 3. DynamicsModel.py: Nonlinear Vehicle Motion Model

`DynamicsModel.py` is the nonlinear process model used by the EKF predict step. It is a dynamic single-track model, sometimes called a dynamic bicycle model.

### 3.1 State and Input Vectors

The EKF-facing state vector used by the dynamics bridge is:

```text
x =
[ p_e,
  p_n,
  psi,
  v_x,
  v_y,
  r,
  b_ax,
  b_gz ]^T
```

where:

- `p_e`, `p_n` are East/North position in meters
- `psi` is yaw angle
- `v_x`, `v_y` are body-frame longitudinal and lateral velocity
- `r` is yaw rate
- `b_ax` is longitudinal accelerometer bias
- `b_gz` is gyro z-axis bias

The predict-step input is:

```text
u =
[ a_x_meas,
  delta_logged ]^T
```

where:

- `a_x_meas` is measured IMU longitudinal acceleration
- `delta_logged` is the logged steering angle

The code converts steering before using it:

```text
delta_math = steering_math_sign_multiplier * delta_logged
           = -delta_logged
```

### 3.2 Body to World Velocity Rotation

The code uses the 2D rotation:

```text
R_body_to_world(psi) =
[ cos(psi)  -sin(psi)
  sin(psi)   cos(psi) ]
```

So the world-frame velocity is:

```text
[ v_e ]   [ cos(psi)  -sin(psi) ] [ v_x ]
[ v_n ] = [ sin(psi)   cos(psi) ] [ v_y ]
```

which gives:

```text
v_e = v_x cos(psi) - v_y sin(psi)
v_n = v_x sin(psi) + v_y cos(psi)
```

This directly drives the position states:

```text
dot(p_e) = v_e = v_x cos(psi) - v_y sin(psi)
dot(p_n) = v_n = v_x sin(psi) + v_y cos(psi)
dot(psi) = r
```

### 3.3 Slip Angle Equations

The model uses a minimum longitudinal speed to avoid division by zero:

```text
v_x_safe = sign(v_x) * max(|v_x|, 0.3)
```

Then the front and rear slip angles are computed with the small-angle single-track form:

```text
alpha_f = delta_math - (v_y + a r) / v_x_safe
alpha_r = -(v_y - b r) / v_x_safe
```

Substituting the current config values:

```text
alpha_f = delta_math - (v_y + 0.159512 r) / v_x_safe
alpha_r = -(v_y - 0.140208 r) / v_x_safe
```

### 3.4 Linear Lateral Tire Forces

The code uses a linear tire model:

```text
F_yf = 2 C_f alpha_f
F_yr = 2 C_r alpha_r
```

With the current config:

```text
F_yf = 2 * 60.0 * alpha_f = 120 alpha_f
F_yr = 2 * 35.0 * alpha_r = 70 alpha_r
```

### 3.5 Longitudinal Acceleration Bias Correction

The predict model uses the IMU longitudinal acceleration as a known input, but subtracts the estimated accelerometer bias:

```text
a_x_corr = a_x_meas - b_ax
```

### 3.6 Continuous-Time State Derivative

The nonlinear dynamics implemented in `compute_planar_state_derivative(...)` are:

```text
dot(p_e) = v_x cos(psi) - v_y sin(psi)
dot(p_n) = v_x sin(psi) + v_y cos(psi)
dot(psi) = r

dot(v_x) = a_x_corr + v_y r - C_d v_x |v_x|
dot(v_y) = -v_x r + (F_yf cos(delta_math) + F_yr) / m
dot(r)   = (a F_yf cos(delta_math) - b F_yr) / I_z

dot(b_ax) = 0
dot(b_gz) = 0
```

Because `C_d = 0.0` in the active config, the current longitudinal equation reduces to:

```text
dot(v_x) = a_x_corr + v_y r
```

Substituting the current parameter values gives:

```text
dot(v_y) = -v_x r + (120 alpha_f cos(delta_math) + 70 alpha_r) / 2.667

dot(r)   = (0.159512 * 120 alpha_f cos(delta_math) - 0.140208 * 70 alpha_r) / 0.6
```

### 3.7 Discrete-Time Propagation Used by the EKF

`predict_ekf_state(...)` uses first-order Euler integration:

```text
x_k^- = f_d(x_{k-1}^+, u_k)
      = x_{k-1}^+ + f(x_{k-1}^+, u_k) dt
```

The yaw angle is wrapped after propagation:

```text
psi <- atan2(sin(psi), cos(psi))
```

### 3.8 Load Transfer Equations

`DynamicsModel.py` also computes first-pass load transfer:

```text
W = m g
Delta_W_x = W (A_x h / L)
Delta_W_y = W (A_y h / t)
```

With the current config:

```text
W = 2.667 * 9.80665 = 26.1543 N
```

So:

```text
Delta_W_x = 26.1543 * (A_x * 0.066309 / 0.3048)
Delta_W_y = 26.1543 * (A_y * 0.066309 / 0.3048)
```

Since `L = t = 0.3048 m` here, both scale factors are the same:

```text
Delta_W_x ~= 5.687 * A_x
Delta_W_y ~= 5.687 * A_y
```

where `A_x` and `A_y` are in `m/s^2`, giving load transfer in newtons.

### 3.9 Handling Classification and Ackermann Steering

The current heuristic handling classifier is:

```text
if |alpha_f| > |alpha_r| + 1 deg -> understeer
if |alpha_r| > |alpha_f| + 1 deg -> oversteer
otherwise                        -> neutral
```

The first-pass steering target is Ackermann geometry:

```text
delta_ack = atan(L / R)
```

Using the active wheelbase:

```text
delta_ack = atan(0.3048 / R)
```

## 4. ExtendedKalmanFilter.py: EKF Math

`ExtendedKalmanFilter.py` wraps the nonlinear vehicle model in the standard EKF equations:

```text
x_k = f(x_{k-1}, u_k) + w_k
z_k = h(x_k) + v_k
```

with:

```text
w_k ~ N(0, Q_k)
v_k ~ N(0, R_k)
```

### 4.1 State Vector and Input Vector

The filter uses:

```text
x =
[ p_e,
  p_n,
  psi,
  v_x,
  v_y,
  r,
  b_ax,
  b_gz ]^T
```

and:

```text
u =
[ a_x_meas,
  delta_logged ]^T
```

### 4.2 Predict Step

The code implements:

```text
x_k^- = f_d(x_{k-1}^+, u_k)
F_k   = d f_d / d x
P_k^- = F_k P_{k-1}^+ F_k^T + Q_k
```

Important implementation detail:

- `f_d(...)` comes from `DynamicsModel.predict_ekf_state(...)`
- `F_k` is computed numerically with a central-difference Jacobian

The numerical Jacobian is:

```text
F_k[:,i] = (f_d(x + eps e_i, u) - f_d(x - eps e_i, u)) / (2 eps)
```

with `eps = 1e-6`.

### 4.3 Initial Covariance Matrix P0

From `config/ekf.yaml`:

```text
position sigma     = 10.0 m
yaw sigma          = 0.5235987756 rad
velocity sigma     = 2.0 m/s
yaw-rate sigma     = 0.3490658504 rad/s
accel-bias sigma   = 1.0 m/s^2
gyro-bias sigma    = 0.0872664626 rad/s
```

So the initial covariance is:

```text
P0 = diag(
  10.0^2,
  10.0^2,
  0.5235987756^2,
  2.0^2,
  2.0^2,
  0.3490658504^2,
  1.0^2,
  0.0872664626^2
)
```

Numerically:

```text
P0 = diag(
  100.0,
  100.0,
  0.2741556778,
  4.0,
  4.0,
  0.1218469679,
  1.0,
  0.0076154355
)
```

### 4.4 Process Noise Matrix Q

`default_process_noise(...)` constructs:

```text
velocity_var   = (model_velocity_sigma_mps2 * dt)^2
yaw_var        = (model_yaw_accel_sigma_radps2 * dt)^2
accel_bias_var = (accel_bias_walk_sigma_mps3 * dt)^2
gyro_bias_var  = (gyro_bias_walk_sigma_radps2 * dt)^2
```

Using the config values:

```text
velocity_var   = (1.5 dt)^2
yaw_var        = (2.0 dt)^2
accel_bias_var = (0.05 dt)^2
gyro_bias_var  = (0.01 dt)^2
```

So:

```text
Q(dt) = diag(
  1e-4,
  1e-4,
  (2.0 dt)^2,
  (1.5 dt)^2,
  (1.5 dt)^2,
  (2.0 dt)^2,
  (0.05 dt)^2,
  (0.01 dt)^2
)
```

Example for `dt = 0.1 s`:

```text
Q(0.1) = diag(
  1.0e-4,
  1.0e-4,
  4.0e-2,
  2.25e-2,
  2.25e-2,
  4.0e-2,
  2.5e-5,
  1.0e-6
)
```

### 4.5 GPS Initialization and Local Coordinates

The filter can initialize from GPS. It defines a local ENU origin from the first good GPS sample and converts latitude/longitude into local meters using:

```text
meters_per_degree_lat = 111320
meters_per_degree_lon = 111320 cos(lat0)

east  = (lon - lon0) meters_per_degree_lon
north = (lat - lat0) meters_per_degree_lat
```

So:

```text
[ p_e ]
[ p_n ]
```

is measured relative to the first accepted GPS fix.

GPS track angle is converted into ENU yaw with:

```text
psi = wrap(pi/2 - deg2rad(track_deg))
```

Then GPS speed and track become world velocity:

```text
v_e = speed cos(psi)
v_n = speed sin(psi)
```

And if the filter needs initial body-frame velocity, it uses the inverse rotation:

```text
[ v_x ]   [  cos(psi)   sin(psi) ] [ v_e ]
[ v_y ] = [ -sin(psi)   cos(psi) ] [ v_n ]
```

### 4.6 Measurement Models h(x)

The EKF currently supports four measurement models.

#### 4.6.1 GPS Position

The measurement is:

```text
z_gps_pos = [ p_e, p_n ]^T + v
```

So:

```text
h_gps_pos(x) =
[ p_e ]
[ p_n ]
```

and the measurement Jacobian is:

```text
H_gps_pos =
[ 1 0 0 0 0 0 0 0
  0 1 0 0 0 0 0 0 ]
```

The default covariance from config is:

```text
R_gps_pos = diag(2.5^2, 2.5^2)
          = diag(6.25, 6.25)
```

If gpsd reports `epx_m` and `epy_m`, the code overrides this with:

```text
R_gps_pos = diag(epx_m^2, epy_m^2)
```

#### 4.6.2 GPS Velocity

The EKF stores velocity in the body frame, but GPS reports world-frame velocity. So the measurement function is:

```text
h_gps_vel(x) =
[ v_x cos(psi) - v_y sin(psi)
  v_x sin(psi) + v_y cos(psi) ]
```

The code computes the Jacobian numerically during the update, but the equivalent analytic Jacobian is:

```text
H_gps_vel =
[ 0 0  -v_x sin(psi) - v_y cos(psi)   cos(psi)  -sin(psi)  0 0 0
  0 0   v_x cos(psi) - v_y sin(psi)   sin(psi)   cos(psi)  0 0 0 ]
```

The default covariance is:

```text
R_gps_vel = diag(0.05^2, 0.05^2)
          = diag(0.0025, 0.0025)
```

If gpsd reports `eps_m_s`, the code uses:

```text
R_gps_vel = diag(eps_m_s^2, eps_m_s^2)
```

The velocity update is gated and skipped if:

```text
speed_m_s < 0.2
```

from `config/ekf.yaml`.

#### 4.6.3 IMU Yaw

The yaw measurement model is:

```text
z_yaw = psi + v
```

So:

```text
h_yaw(x) = [ psi ]
```

and:

```text
H_yaw = [ 0 0 1 0 0 0 0 0 ]
```

The covariance is:

```text
R_yaw = [ 0.026^2 ] = [ 0.000676 ]
```

The innovation is wrapped:

```text
y <- wrap(z - h(x))
```

so angle differences stay in `[-pi, pi)`.

#### 4.6.4 IMU Yaw Rate

The gyro z-axis measurement includes true yaw rate plus the gyro bias state:

```text
z_r = r + b_gz + v
```

So:

```text
h_r(x) = [ r + b_gz ]
```

and:

```text
H_r = [ 0 0 0 0 0 1 0 1 ]
```

The covariance is:

```text
R_r = [ 0.0014^2 ] = [ 1.96e-6 ]
```

### 4.7 EKF Update Equations

For any measurement model, the update is:

```text
y_k   = z_k - h(x_k^-)
S_k   = H_k P_k^- H_k^T + R_k
K_k   = P_k^- H_k^T S_k^-1
x_k^+ = x_k^- + K_k y_k
```

The covariance update uses the Joseph form:

```text
P_k^+ = (I - K_k H_k) P_k^- (I - K_k H_k)^T + K_k R_k K_k^T
```

This is more numerically stable than the minimal:

```text
P <- (I - K H) P
```

The code also solves for `K` with `np.linalg.solve(...)` instead of explicitly forming `S^-1`.

## 5. Odometry.py: Open-Loop Integration Math

`Odometry.py` is a separate, non-probabilistic integration path. It does not maintain a covariance matrix. Instead, it integrates IMU-derived acceleration and orientation directly.

### 5.1 Raw and Filtered Sensor Channels

The file gathers:

- raw gyro
- raw acceleration
- magnetometer
- gravity vector
- linear acceleration
- quaternion
- Euler angles

The body linear acceleration can be filtered in two stages:

```text
body_accel_filtered = moving_average(raw_body_accel)
body_accel_smoothed = low_pass(body_accel_filtered, dt)
```

If filtering is disabled, the value passes through unchanged.

### 5.2 Quaternion Rotation Matrix

The odometry path stores a body-to-world rotation matrix built from the IMU quaternion:

```text
R_bw = quaternion_rotation_matrix(q)
```

Then it rotates the measured body-frame linear acceleration into the world frame:

```text
a_world = R_bw a_body
```

Written as vectors:

```text
[ a_x_world ]   [ r11 r12 r13 ] [ a_x_body ]
[ a_y_world ] = [ r21 r22 r23 ] [ a_y_body ]
[ a_z_world ]   [ r31 r32 r33 ] [ a_z_body ]
```

This is the key matrix operation in `Odometry.update(...)`.

### 5.3 Trapezoidal Integration for Velocity

Velocity is updated with the trapezoidal rule:

```text
v_k = v_{k-1} + 0.5 (a_k + a_{k-1}) dt
```

Component-wise:

```text
v_x,k = v_x,k-1 + 0.5 (a_x,k + a_x,k-1) dt
v_y,k = v_y,k-1 + 0.5 (a_y,k + a_y,k-1) dt
v_z,k = v_z,k-1 + 0.5 (a_z,k + a_z,k-1) dt
```

### 5.4 Trapezoidal Integration for Position

Position is then updated the same way using velocity:

```text
p_k = p_{k-1} + 0.5 (v_k + v_{k-1}) dt
```

Component-wise:

```text
x_k = x_k-1 + 0.5 (v_x,k + v_x,k-1) dt
y_k = y_k-1 + 0.5 (v_y,k + v_y,k-1) dt
z_k = z_k-1 + 0.5 (v_z,k + v_z,k-1) dt
```

So the odometry stack is:

```text
body acceleration
 -> rotate to world frame
 -> integrate to velocity
 -> integrate to position
```

without any correction step like the EKF has.

### 5.5 Relative Orientation

`Odometry.py` stores:

```text
absolute_orientation = current IMU Euler angles
relative_orientation = absolute_orientation - zeroed_orientation
```

So the relative orientation is just the current orientation offset from the IMU's zero reference.

### 5.6 North Estimation from Gravity and Magnetometer

`find_north()` uses normalized gravity and magnetometer vectors:

```text
g_hat = g / ||g||
m_hat = m / ||m||
```

Then:

```text
east  = g_hat x m_hat
north = east x g_hat
```

The method then computes:

```text
roll_N  = atan2(north_z, north_y)
pitch_N = atan2(north_z, north_x)
yaw_N   = atan2(north_x, north_y)
```

and converts the result to degrees.

## 6. How the Three Files Fit Together

The current architecture is:

```text
Odometry.py
  - direct IMU integration
  - no covariance
  - useful for raw dead reckoning and debugging

DynamicsModel.py
  - nonlinear vehicle model
  - provides f(x, u)
  - computes slip angles, tire forces, load transfer, and handling indicators

ExtendedKalmanFilter.py
  - wraps the dynamics model in an EKF
  - predicts with f(x, u)
  - corrects with GPS and IMU measurements through h(x)
  - tracks uncertainty with P, Q, and R matrices
```

In short:

```text
Odometry = direct integration
Dynamics = physics model
EKF      = physics model + sensor fusion + covariance propagation
```

## 7. Most Important Matrices in the Current Implementation

For quick reference, the most important matrices are:

```text
State vector:
x = [p_e, p_n, psi, v_x, v_y, r, b_ax, b_gz]^T

Input vector:
u = [a_x_meas, delta_logged]^T

2D body-to-world rotation:
R(psi) =
[ cos(psi)  -sin(psi)
  sin(psi)   cos(psi) ]

Initial covariance:
P0 = diag(100, 100, 0.2741556778, 4, 4, 0.1218469679, 1, 0.0076154355)

Process noise:
Q(dt) = diag(1e-4, 1e-4, (2dt)^2, (1.5dt)^2, (1.5dt)^2, (2dt)^2, (0.05dt)^2, (0.01dt)^2)

GPS position Jacobian:
H_gps_pos =
[ 1 0 0 0 0 0 0 0
  0 1 0 0 0 0 0 0 ]

GPS velocity measurement:
h_gps_vel(x) =
[ v_x cos(psi) - v_y sin(psi)
  v_x sin(psi) + v_y cos(psi) ]

Yaw Jacobian:
H_yaw = [ 0 0 1 0 0 0 0 0 ]

Yaw-rate Jacobian:
H_r   = [ 0 0 0 0 0 1 0 1 ]

GPS position covariance:
R_gps_pos = diag(6.25, 6.25)

GPS velocity covariance:
R_gps_vel = diag(0.0025, 0.0025)

Yaw covariance:
R_yaw = [0.000676]

Yaw-rate covariance:
R_r = [1.96e-6]
```

This is the math the current code is actually implementing today.
