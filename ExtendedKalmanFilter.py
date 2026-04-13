from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable, Mapping

import numpy as np

from ConfigLoader import get_section, load_config_bundle, read_nested
from DynamicsModel import DynamicsModel, VehicleParameterDefinition

#TODO possibly integrate WelfordsOnlineAlgorithm.py for online estimation of mean, variance, and standard deviation of sensor noise, which could be used to adaptively tune the EKF measurement noise covariance R based on recent sensor performance.

"""
Extended Kalman Filter (EKF) for the RC car navigation project.

Architecture overview
---------------------
DynamicsModel (DynamicsModel.py)
    Owns the nonlinear process model  f(x, u)  and its discrete form  f_d.
    It is intentionally separate so the vehicle dynamics can be developed,
    tuned, and tested independently from the filter machinery.

ExtendedKalmanFilter (this file)
    Owns the filter state x and covariance P.
    It calls DynamicsModel for predict and applies sensor corrections in update.

The nonlinear EKF equations:

    Process model:     x_k = f(x_{k-1}, u_k) + w_k     w ~ N(0, Q)
    Measurement model: z_k = h(x_k)          + v_k     v ~ N(0, R)

Predict step  (run every loop iteration, driven by IMU + steering):

    x^-_k = f_d(x^+_{k-1}, u_k)              -- forward-Euler via DynamicsModel
    F_k   = d f_d / d x                       -- Jacobian (numerical central diff)
    P^-_k = F_k P^+_{k-1} F_k^T + Q_k        -- covariance prediction

    F_k maps how a small perturbation in the current state estimate propagates
    through the nonlinear dynamics into the next predicted state.  It plays the
    same role as the state-transition matrix A in a linear Kalman filter.

Update step  (run whenever a sensor measurement arrives):

    y_k   = z_k - h(x^-_k)                   -- innovation (residual)
    H_k   = d h / d x |_{x^-_k}              -- measurement Jacobian
    S_k   = H_k P^-_k H_k^T + R_k            -- innovation covariance
    K_k   = P^-_k H_k^T S_k^{-1}             -- Kalman gain
    x^+_k = x^-_k + K_k y_k
    P^+_k = (I - K_k H_k) P^-_k (I - K_k H_k)^T + K_k R_k K_k^T

    The last line is the Joseph form.  It is slightly more expensive than the
    minimal P^+ = (I - K H) P^- but stays symmetric and positive-definite under
    floating-point rounding, which matters for long runs.

    Conceptually: the Kalman gain K_k decides how much to trust the sensor (z_k)
    versus the model prediction (x^-_k).  If R is small (sensor trusted) K is
    large and the state is pulled toward the measurement.  If P^- is small
    (model trusted) K is small and the measurement has little influence.

State vector (11 elements):
    x = [p_e, p_n, psi, phi, theta, v_x, v_y, r, b_ax, b_ay, b_gz]^T
    p_e, p_n  -- local ENU East/North position (m)
    psi       -- yaw angle, CCW from East (rad)
    phi       -- roll angle (rad)
    theta     -- pitch angle (rad)
    v_x, v_y  -- body-frame longitudinal/lateral velocity (m/s)
    r         -- yaw rate (rad/s)
    b_ax      -- IMU longitudinal accelerometer bias (m/s^2)
    b_ay      -- IMU lateral accelerometer bias (m/s^2)
    b_gz      -- IMU gyro z-axis bias (rad/s)

Control input:
    u = [a_x_meas, delta_logged]^T

Sensor updates active:
    - GPS position       (h = [p_e, p_n]^T,                    analytic H)
    - GPS velocity       (h = R(psi) [v_x, v_y]^T,             analytic H)
    - IMU yaw            (h = [psi],                            analytic H)
    - IMU yaw rate       (h = [r + b_gz],                       analytic H)
    - IMU roll           (h = [phi],                            analytic H)
    - IMU pitch          (h = [theta],                          analytic H)
    - IMU lateral accel  (h = [(F_yf cos delta + F_yr)/m + b_ay], numerical H)

Frame conventions:
    - Body: +X forward, +Y left, +Z up
    - World: local ENU  (x = East, y = North)
    - Steering: configurable sign mapping (see config/frames.yaml)
"""


STATE_VECTOR_2D = (
    "p_e_m",              # 0
    "p_n_m",              # 1
    "yaw_rad",            # 2
    "roll_rad",           # 3
    "pitch_rad",          # 4
    "v_x_body_mps",       # 5
    "v_y_body_mps",       # 6
    "yaw_rate_radps",     # 7
    "accel_bias_x_mps2",  # 8
    "accel_bias_y_mps2",  # 9
    "gyro_bias_z_radps",  # 10
)

INPUT_VECTOR_2D = (
    "imu_accel_x_body_mps2",
    "steering_angle_rad",
)

DEFAULT_STEERING_SIGN_CONVENTION = "negative-left"


@dataclass(frozen=True)
class NoiseParameters:
    """Default process and measurement noise assumptions."""

    gps_position_sigma_m: float = 2.5
    gps_velocity_sigma_mps: float = 0.05
    imu_yaw_sigma_rad: float = 0.026
    imu_roll_sigma_rad: float = 0.026
    imu_pitch_sigma_rad: float = 0.026
    imu_yaw_rate_sigma_radps: float = 0.0014
    imu_lateral_accel_sigma_mps2: float = 0.012

    accel_bias_walk_sigma_mps3: float = 0.05
    gyro_bias_walk_sigma_radps2: float = 0.01
    model_velocity_sigma_mps2: float = 1.5
    model_yaw_accel_sigma_radps2: float = 2.0

    @classmethod
    def from_config_bundle(cls, config_bundle: Mapping[str, Mapping[str, object]]) -> "NoiseParameters":
        ekf_cfg = get_section(config_bundle.get("ekf"), "ekf")
        sensors_cfg = config_bundle.get("sensors", {})
        imu_cfg = get_section(sensors_cfg.get("imu"), "imu")
        gps_cfg = get_section(sensors_cfg.get("gps"), "gps")

        measurement_noise = get_section(ekf_cfg.get("measurement_noise"), "measurement_noise")
        process_noise = get_section(ekf_cfg.get("process_noise"), "process_noise")
        imu_noise = get_section(imu_cfg.get("noise"), "noise")
        gps_noise = get_section(gps_cfg.get("noise"), "noise")

        return cls(
            gps_position_sigma_m=float(
                measurement_noise.get(
                    "gps_position_sigma_m",
                    read_nested(gps_noise, "position_sigma_m", default=2.5),
                )
            ),
            gps_velocity_sigma_mps=float(
                measurement_noise.get(
                    "gps_velocity_sigma_mps",
                    read_nested(gps_noise, "velocity_sigma_mps", default=0.05),
                )
            ),
            imu_yaw_sigma_rad=float(
                measurement_noise.get(
                    "imu_yaw_sigma_rad",
                    read_nested(imu_noise, "yaw_sigma_rad", default=0.026),
                )
            ),
            imu_roll_sigma_rad=float(
                measurement_noise.get(
                    "imu_roll_sigma_rad",
                    read_nested(imu_noise, "roll_sigma_rad", default=0.026),
                )
            ),
            imu_pitch_sigma_rad=float(
                measurement_noise.get(
                    "imu_pitch_sigma_rad",
                    read_nested(imu_noise, "pitch_sigma_rad", default=0.026),
                )
            ),
            imu_yaw_rate_sigma_radps=float(
                measurement_noise.get(
                    "imu_yaw_rate_sigma_radps",
                    read_nested(imu_noise, "yaw_rate_sigma_radps", default=0.0014),
                )
            ),
            imu_lateral_accel_sigma_mps2=float(
                measurement_noise.get(
                    "imu_lateral_accel_sigma_mps2",
                    read_nested(imu_noise, "lateral_accel_sigma_mps2", default=0.012),
                )
            ),
            accel_bias_walk_sigma_mps3=float(process_noise.get("accel_bias_walk_sigma_mps3", 0.05)),
            gyro_bias_walk_sigma_radps2=float(process_noise.get("gyro_bias_walk_sigma_radps2", 0.01)),
            model_velocity_sigma_mps2=float(process_noise.get("model_velocity_sigma_mps2", 1.5)),
            model_yaw_accel_sigma_radps2=float(process_noise.get("model_yaw_accel_sigma_radps2", 2.0)),
        )


@dataclass(frozen=True)
class GeodeticOrigin:
    """Reference latitude/longitude for local ENU conversion."""

    latitude_deg: float
    longitude_deg: float


@dataclass(frozen=True)
class EKFEstimate:
    """Snapshot of the current EKF state and covariance."""

    state: np.ndarray
    covariance: np.ndarray


@dataclass(frozen=True)
class MeasurementDefinition:
    """Human-readable description of an EKF measurement channel."""

    name: str
    source_file: str
    source_fields: tuple[str, ...]
    equation: str
    covariance_hint: str


def build_initial_covariance_from_config(config_bundle: Mapping[str, Mapping[str, object]]) -> np.ndarray:
    """
    Build the EKF initial covariance matrix from ``config/ekf.yaml``.
    """
    ekf_cfg = get_section(config_bundle.get("ekf"), "ekf")
    covariance_cfg = get_section(ekf_cfg.get("initial_covariance"), "initial_covariance")

    position_sigma = float(covariance_cfg.get("position_sigma_m", 10.0))
    yaw_sigma = float(covariance_cfg.get("yaw_sigma_rad", math.radians(30.0)))
    roll_sigma = float(covariance_cfg.get("roll_sigma_rad", math.radians(10.0)))
    pitch_sigma = float(covariance_cfg.get("pitch_sigma_rad", math.radians(10.0)))
    velocity_sigma = float(covariance_cfg.get("velocity_sigma_mps", 2.0))
    yaw_rate_sigma = float(covariance_cfg.get("yaw_rate_sigma_radps", math.radians(20.0)))
    accel_bias_sigma = float(covariance_cfg.get("accel_bias_sigma_mps2", 1.0))
    gyro_bias_sigma = float(covariance_cfg.get("gyro_bias_sigma_radps", math.radians(5.0)))

    return np.diag(
        [
            position_sigma**2,   # p_e
            position_sigma**2,   # p_n
            yaw_sigma**2,        # psi
            roll_sigma**2,       # phi
            pitch_sigma**2,      # theta
            velocity_sigma**2,   # v_x
            velocity_sigma**2,   # v_y
            yaw_rate_sigma**2,   # r
            accel_bias_sigma**2, # b_ax
            accel_bias_sigma**2, # b_ay
            gyro_bias_sigma**2,  # b_gz
        ]
    )


MEASUREMENT_MODELS = (
    MeasurementDefinition(
        name="gps_position",
        source_file="GPS_Util.py",
        source_fields=("latitude", "longitude", "epx_m", "epy_m"),
        equation="z_pos = [p_e, p_n]^T + v",
        covariance_hint="Use gps epx/epy when present, else 2.5 m std dev",
    ),
    MeasurementDefinition(
        name="gps_velocity",
        source_file="GPS_Util.py",
        source_fields=("speed_m_s", "track_deg", "eps_m_s"),
        equation="z_vel = R(psi)[v_x, v_y]^T + v",
        covariance_hint="Use gps eps when present, else 0.05 m/s std dev",
    ),
    MeasurementDefinition(
        name="imu_yaw",
        source_file="IMU.py",
        source_fields=("get_euler_angles()[2]",),
        equation="z_yaw = psi + v",
        covariance_hint="Approx 0.026 rad std dev",
    ),
    MeasurementDefinition(
        name="imu_roll",
        source_file="IMU.py",
        source_fields=("get_euler_angles()[0]",),
        equation="z_roll = phi + v",
        covariance_hint="Approx 0.026 rad std dev",
    ),
    MeasurementDefinition(
        name="imu_pitch",
        source_file="IMU.py",
        source_fields=("get_euler_angles()[1]",),
        equation="z_pitch = theta + v",
        covariance_hint="Approx 0.026 rad std dev",
    ),
    MeasurementDefinition(
        name="imu_yaw_rate",
        source_file="IMU.py",
        source_fields=("get_raw_gyro()[2]",),
        equation="z_r = r + b_gz + v",
        covariance_hint="Approx 0.0014 rad/s std dev after unit conversion",
    ),
    MeasurementDefinition(
        name="imu_lateral_accel",
        source_file="IMU.py",
        source_fields=("get_linear_acceleration()[1]",),
        equation="z_ay = (F_yf*cos(delta) + F_yr)/m + b_ay + v",
        covariance_hint="Approx 0.012 m/s^2 std dev; skip below low-speed threshold",
    ),
)


def wrap_angle_rad(angle_rad: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def steering_math_sign_multiplier(steering_sign_convention: str) -> float:
    """
    Convert the logged steering sign into standard body-XY math rotation sign.
    """
    if steering_sign_convention == "negative-left":
        return -1.0
    if steering_sign_convention == "positive-left":
        return 1.0
    raise ValueError(f"Unsupported steering sign convention: {steering_sign_convention}")


def steering_measurement_to_math_angle_rad(
    steering_angle_rad: float,
    steering_sign_convention: str = DEFAULT_STEERING_SIGN_CONVENTION,
) -> float:
    """Convert a logged steering angle into the body-XY math sign convention."""
    return steering_math_sign_multiplier(steering_sign_convention) * float(steering_angle_rad)


def body_velocity_to_world_velocity(vx_body_mps: float, vy_body_mps: float, yaw_rad: float) -> np.ndarray:
    """Convert body-frame velocity to local ENU world-frame velocity."""
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    return np.array(
        [
            vx_body_mps * c - vy_body_mps * s,
            vx_body_mps * s + vy_body_mps * c,
        ],
        dtype=float,
    )


def gps_track_deg_to_yaw_rad(track_deg: float) -> float:
    """Convert gpsd course-over-ground to ENU yaw."""
    return wrap_angle_rad((math.pi / 2.0) - math.radians(float(track_deg)))


def gps_speed_track_to_enu_velocity(speed_m_s: float, track_deg: float) -> np.ndarray:
    """Convert gpsd speed and course-over-ground into ENU velocity."""
    yaw_rad = gps_track_deg_to_yaw_rad(track_deg)
    return np.array(
        [
            float(speed_m_s) * math.cos(yaw_rad),
            float(speed_m_s) * math.sin(yaw_rad),
        ],
        dtype=float,
    )


def geodetic_to_local_xy_m(
    latitude_deg: float,
    longitude_deg: float,
    origin: GeodeticOrigin,
) -> np.ndarray:
    """Convert latitude/longitude into local East/North meter offsets."""
    lat0_rad = math.radians(origin.latitude_deg)
    meters_per_degree_lat = 111_320.0
    meters_per_degree_lon = 111_320.0 * math.cos(lat0_rad)
    east_m = (float(longitude_deg) - origin.longitude_deg) * meters_per_degree_lon
    north_m = (float(latitude_deg) - origin.latitude_deg) * meters_per_degree_lat
    return np.array([east_m, north_m], dtype=float)


def numerical_jacobian(
    fn: Callable[[np.ndarray], np.ndarray],
    x0: np.ndarray,
    epsilon: float = 1e-6,
) -> np.ndarray:
    """Central-difference Jacobian helper for nonlinear EKF models."""
    x = np.asarray(x0, dtype=float).reshape(-1)
    y0 = np.asarray(fn(x), dtype=float).reshape(-1)
    jac = np.zeros((y0.shape[0], x.shape[0]), dtype=float)

    for index in range(x.shape[0]):
        delta = np.zeros_like(x)
        delta[index] = epsilon
        y_plus = np.asarray(fn(x + delta), dtype=float).reshape(-1)
        y_minus = np.asarray(fn(x - delta), dtype=float).reshape(-1)
        jac[:, index] = (y_plus - y_minus) / (2.0 * epsilon)

    return jac


def default_process_noise(
    noise: NoiseParameters = NoiseParameters(),
    dt: float = 0.1,
) -> np.ndarray:
    """Build a simple diagonal process-noise matrix for the dynamic state."""
    if dt <= 0.0:
        raise ValueError("dt must be positive.")

    velocity_var = (noise.model_velocity_sigma_mps2 * dt) ** 2
    yaw_var = (noise.model_yaw_accel_sigma_radps2 * dt) ** 2
    accel_bias_var = (noise.accel_bias_walk_sigma_mps3 * dt) ** 2
    gyro_bias_var = (noise.gyro_bias_walk_sigma_radps2 * dt) ** 2
    # Roll and pitch are driven directly by IMU measurements, but the predict
    # step models them as random walks to let covariance grow between IMU ticks.
    attitude_var = (noise.imu_roll_sigma_rad * dt) ** 2

    return np.diag(
        [
            1e-4,             # p_e   -- position integration error is small
            1e-4,             # p_n
            yaw_var,          # psi
            attitude_var,     # phi   (roll)
            attitude_var,     # theta (pitch)
            velocity_var,     # v_x
            velocity_var,     # v_y
            yaw_var,          # r     (yaw rate)
            accel_bias_var,   # b_ax
            accel_bias_var,   # b_ay
            gyro_bias_var,    # b_gz
        ]
    )


def default_measurement_noise(noise: NoiseParameters = NoiseParameters()) -> dict[str, np.ndarray]:
    """Return default measurement covariance matrices."""
    return {
        "gps_position": np.diag([noise.gps_position_sigma_m**2, noise.gps_position_sigma_m**2]),
        "gps_velocity": np.diag([noise.gps_velocity_sigma_mps**2, noise.gps_velocity_sigma_mps**2]),
        "imu_yaw": np.array([[noise.imu_yaw_sigma_rad**2]], dtype=float),
        "imu_roll": np.array([[noise.imu_roll_sigma_rad**2]], dtype=float),
        "imu_pitch": np.array([[noise.imu_pitch_sigma_rad**2]], dtype=float),
        "imu_yaw_rate": np.array([[noise.imu_yaw_rate_sigma_radps**2]], dtype=float),
        "imu_lateral_accel": np.array([[noise.imu_lateral_accel_sigma_mps2**2]], dtype=float),
    }


def gps_measurement_noise_from_data(
    gps_data: Mapping[str, object],
    noise: NoiseParameters = NoiseParameters(),
) -> dict[str, np.ndarray]:
    """Build GPS covariance matrices using gpsd-reported accuracy when available."""
    sigma_e = float(gps_data.get("epx_m") or noise.gps_position_sigma_m)
    sigma_n = float(gps_data.get("epy_m") or noise.gps_position_sigma_m)
    sigma_speed = float(gps_data.get("eps_m_s") or noise.gps_velocity_sigma_mps)

    return {
        "gps_position": np.diag([sigma_e**2, sigma_n**2]),
        "gps_velocity": np.diag([sigma_speed**2, sigma_speed**2]),
    }


def h_gps_position_2d(state: np.ndarray) -> np.ndarray:
    """
    Linear measurement model for GPS position.

    In EKF notation:

        z_pos = h_pos(x) + v
              = [p_e, p_n]^T + v
    """
    x = np.asarray(state, dtype=float).reshape(-1)
    return x[0:2].copy()


def h_gps_velocity_2d(state: np.ndarray) -> np.ndarray:
    """
    Nonlinear measurement model for GPS velocity in the ENU/world frame.

    GPS reports velocity in the world frame, but the state stores velocity in
    the body frame, so we apply the yaw-dependent rotation R(psi):

        z_vel = h_vel(x) + v
              = R(psi) [v_x, v_y]^T + v
              = [v_x*cos(psi) - v_y*sin(psi),
                 v_x*sin(psi) + v_y*cos(psi)]^T + v
    """
    x = np.asarray(state, dtype=float).reshape(-1)
    _p_e, _p_n, psi, _phi, _theta, v_x, v_y, _r, _b_ax, _b_ay, _b_gz = x
    return body_velocity_to_world_velocity(v_x, v_y, psi)


def h_gps_velocity_jacobian_2d(state: np.ndarray) -> np.ndarray:
    """
    Analytic Jacobian of h_gps_velocity_2d with respect to the state vector.

    Because h_vel is nonlinear in psi, v_x, and v_y, the EKF needs its
    derivative H_k = dh/dx evaluated at the current predicted state.

    Differentiating h_vel = [v_x*cos(psi) - v_y*sin(psi),
                              v_x*sin(psi) + v_y*cos(psi)] row by row:

        d(h[0])/d(psi) = -v_x*sin(psi) - v_y*cos(psi)
        d(h[0])/d(v_x) =  cos(psi)
        d(h[0])/d(v_y) = -sin(psi)

        d(h[1])/d(psi) =  v_x*cos(psi) - v_y*sin(psi)
        d(h[1])/d(v_x) =  sin(psi)
        d(h[1])/d(v_y) =  cos(psi)

    All other partial derivatives are zero.  Providing this analytically
    avoids 16 extra DynamicsModel evaluations that a numerical Jacobian would
    require (central differences over 8 state dimensions, 2 function outputs).
    """
    x = np.asarray(state, dtype=float).reshape(-1)
    _p_e, _p_n, psi, _phi, _theta, v_x, v_y, _r, _b_ax, _b_ay, _b_gz = x
    c, s = math.cos(psi), math.sin(psi)

    h = np.zeros((2, len(STATE_VECTOR_2D)), dtype=float)
    # Column indices: 0=p_e, 1=p_n, 2=psi, 3=phi, 4=theta, 5=v_x, 6=v_y, 7=r, 8=b_ax, 9=b_ay, 10=b_gz
    h[0, 2] = -v_x * s - v_y * c   # d(v_e)/d(psi)
    h[0, 5] = c                     # d(v_e)/d(v_x)
    h[0, 6] = -s                    # d(v_e)/d(v_y)
    h[1, 2] = v_x * c - v_y * s    # d(v_n)/d(psi)
    h[1, 5] = s                     # d(v_n)/d(v_x)
    h[1, 6] = c                     # d(v_n)/d(v_y)
    return h


def h_imu_yaw_2d(state: np.ndarray) -> np.ndarray:
    """
    Direct heading measurement model.

        z_yaw = h_yaw(x) + v
              = [psi] + v
    """
    x = np.asarray(state, dtype=float).reshape(-1)
    return np.array([wrap_angle_rad(x[2])], dtype=float)


def h_imu_yaw_rate_2d(state: np.ndarray) -> np.ndarray:
    """
    Gyroscope z-axis measurement model.

    The gyro sees yaw rate plus the estimated gyro bias state:

        z_r = h_r(x) + v
            = [r + b_gz] + v
    """
    x = np.asarray(state, dtype=float).reshape(-1)
    return np.array([x[7] + x[10]], dtype=float)


def h_imu_roll_2d(state: np.ndarray) -> np.ndarray:
    """
    Direct roll measurement model.

    The BNO055 NDOF fusion reports a full Euler orientation tuple
    (roll, pitch, yaw).  Roll maps directly to the phi state:

        z_roll = h_roll(x) + v
               = [phi] + v
    """
    x = np.asarray(state, dtype=float).reshape(-1)
    return np.array([wrap_angle_rad(x[3])], dtype=float)


def h_imu_pitch_2d(state: np.ndarray) -> np.ndarray:
    """
    Direct pitch measurement model.

    The BNO055 NDOF fusion reports a full Euler orientation tuple
    (roll, pitch, yaw).  Pitch maps directly to the theta state:

        z_pitch = h_pitch(x) + v
                = [theta] + v
    """
    x = np.asarray(state, dtype=float).reshape(-1)
    return np.array([x[4]], dtype=float)


def h_imu_lateral_accel_2d(
    state: np.ndarray,
    steering_angle_logged_rad: float,
    dynamics_model: "DynamicsModel",
) -> np.ndarray:
    """
    Nonlinear lateral body-frame acceleration measurement model.

    The BNO055 gravity-compensated linear acceleration in the body Y-axis is:

        a_linear_y = dot_v_y + v_x * r
                   = (F_yf * cos(delta) + F_yr) / m + b_ay

    where:
        F_yf = 2 * C_f * alpha_f     -- front lateral tire force
        F_yr = 2 * C_r * alpha_r     -- rear lateral tire force
        alpha_f = delta_math - (v_y + a*r) / v_x_safe
        alpha_r = -(v_y - b*r) / v_x_safe

    ``steering_angle_logged_rad`` is the raw logged steering angle;
    ``compute_slip_angles`` applies the configured sign convention internally.

    The bias term b_ay (index 9) captures any systematic IMU offset in
    the lateral axis.  This update is evaluated using a numerical Jacobian
    because the tire-force equations are nonlinear in v_x, v_y, and r.
    """
    x = np.asarray(state, dtype=float).reshape(-1)
    _p_e, _p_n, _psi, _phi, _theta, v_x, v_y, r, _b_ax, b_ay, _b_gz = x

    state_dict = {
        "v_x_body_mps": float(v_x),
        "v_y_body_mps": float(v_y),
        "yaw_rate_radps": float(r),
    }
    input_dict = {
        "steering_angle_rad": float(steering_angle_logged_rad),
    }
    slip = dynamics_model.compute_slip_angles(state_dict, input_dict)
    forces = dynamics_model.compute_lateral_tire_forces(slip)

    # The math steering angle (after sign conversion) is needed for the cos(delta) projection.
    delta_math = slip["steering_angle_math_rad"]
    cos_delta = math.cos(delta_math)
    m = dynamics_model.vehicle_parameters.mass_kg
    a_lat = (forces["front_lateral_force_n"] * cos_delta + forces["rear_lateral_force_n"]) / m
    return np.array([a_lat + float(b_ay)], dtype=float)


class ExtendedKalmanFilter:
    """
    Dynamic-vehicle EKF starter for this RC car project.

    The state tracks:
    - position in local ENU
    - yaw angle
    - body longitudinal and lateral velocity
    - yaw rate
    - IMU longitudinal accel bias
    - IMU yaw-rate bias
    """

    def __init__(
        self,
        vehicle: VehicleParameterDefinition | None = None,
        noise: NoiseParameters = NoiseParameters(),
        steering_sign_convention: str = DEFAULT_STEERING_SIGN_CONVENTION,
        low_speed_velocity_update_threshold_mps: float = 0.2,
        initial_covariance: np.ndarray | None = None,
        config_bundle: Mapping[str, Mapping[str, object]] | None = None,
        dynamics_model: DynamicsModel | None = None,
    ) -> None:
        self.config_bundle = dict(config_bundle) if config_bundle is not None else None
        self.dynamics_model = (
            dynamics_model
            if dynamics_model is not None
            else DynamicsModel.from_config_bundle(self.config_bundle)
        )
        # Resolve vehicle parameters: caller-supplied > from config bundle > dynamics model > defaults.
        # VehicleParameterDefinition is the shared type used by DynamicsModel, so no conversion needed.
        if vehicle is not None:
            resolved_vehicle = vehicle
        elif self.config_bundle is not None:
            resolved_vehicle = VehicleParameterDefinition.from_config_bundle(self.config_bundle)
        else:
            resolved_vehicle = self.dynamics_model.vehicle_parameters

        self.vehicle = resolved_vehicle
        self.dynamics_model.set_vehicle_parameters(self.vehicle)
        self.noise = noise
        self.steering_sign_convention = steering_sign_convention
        self.low_speed_velocity_update_threshold_mps = float(low_speed_velocity_update_threshold_mps)

        self.state = np.zeros(len(STATE_VECTOR_2D), dtype=float)
        self.covariance = (
            np.asarray(initial_covariance, dtype=float)
            if initial_covariance is not None
            else np.diag(
                [
                    10.0**2,                    # p_e
                    10.0**2,                    # p_n
                    math.radians(30.0) ** 2,   # psi
                    math.radians(10.0) ** 2,   # phi (roll)
                    math.radians(10.0) ** 2,   # theta (pitch)
                    2.0**2,                    # v_x
                    2.0**2,                    # v_y
                    math.radians(20.0) ** 2,   # r
                    1.0**2,                    # b_ax
                    1.0**2,                    # b_ay
                    math.radians(5.0) ** 2,    # b_gz
                ]
            )
        )
        self.origin: GeodeticOrigin | None = None
        self.initialized = False

    @classmethod
    def from_config_bundle(
        cls,
        config_bundle: Mapping[str, Mapping[str, object]] | None = None,
        dynamics_model: DynamicsModel | None = None,
    ) -> "ExtendedKalmanFilter":
        """
        Construct an EKF instance directly from the YAML config bundle.
        """
        bundle = load_config_bundle() if config_bundle is None else dict(config_bundle)
        ekf_cfg = get_section(bundle.get("ekf"), "ekf")
        frames_cfg = bundle.get("frames", {})
        steering_sign_convention = str(
            read_nested(
                frames_cfg,
                "steering_convention",
                "selected_sign",
                default=ekf_cfg.get("steering_sign_convention", DEFAULT_STEERING_SIGN_CONVENTION),
            )
        )
        shared_dynamics_model = dynamics_model or DynamicsModel.from_config_bundle(bundle)

        return cls(
            vehicle=VehicleParameterDefinition.from_config_bundle(bundle),
            noise=NoiseParameters.from_config_bundle(bundle),
            steering_sign_convention=steering_sign_convention,
            low_speed_velocity_update_threshold_mps=float(
                ekf_cfg.get("low_speed_velocity_update_threshold_mps", 0.2)
            ),
            initial_covariance=build_initial_covariance_from_config(bundle),
            config_bundle=bundle,
            dynamics_model=shared_dynamics_model,
        )

    @classmethod
    def from_default_config(cls) -> "ExtendedKalmanFilter":
        """Construct an EKF from the repository's default YAML bundle."""
        return cls.from_config_bundle(load_config_bundle())

    def get_estimate(self) -> EKFEstimate:
        """Return a copy of the current state estimate."""
        return EKFEstimate(self.state.copy(), self.covariance.copy())

    def set_geodetic_origin(self, latitude_deg: float, longitude_deg: float) -> GeodeticOrigin:
        """Set the local reference origin used for GPS conversion."""
        self.origin = GeodeticOrigin(float(latitude_deg), float(longitude_deg))
        return self.origin

    def initialize(
        self,
        position_east_m: float = 0.0,
        position_north_m: float = 0.0,
        yaw_rad: float = 0.0,
        roll_rad: float = 0.0,
        pitch_rad: float = 0.0,
        velocity_x_body_mps: float = 0.0,
        velocity_y_body_mps: float = 0.0,
        yaw_rate_radps: float = 0.0,
        covariance: np.ndarray | None = None,
    ) -> None:
        """Initialize the EKF state directly in the chosen local/world frames."""
        self.state = np.array(
            [
                float(position_east_m),          # p_e
                float(position_north_m),          # p_n
                wrap_angle_rad(float(yaw_rad)),   # psi
                wrap_angle_rad(float(roll_rad)),  # phi
                float(pitch_rad),                 # theta
                float(velocity_x_body_mps),       # v_x
                float(velocity_y_body_mps),       # v_y
                float(yaw_rate_radps),            # r
                0.0,                              # b_ax
                0.0,                              # b_ay
                0.0,                              # b_gz
            ],
            dtype=float,
        )
        if covariance is not None:
            candidate = np.asarray(covariance, dtype=float)
            expected_shape = (len(STATE_VECTOR_2D), len(STATE_VECTOR_2D))
            if candidate.shape != expected_shape:
                raise ValueError(f"Expected covariance shape {expected_shape}, got {candidate.shape}.")
            self.covariance = candidate.copy()
        self.initialized = True

    def initialize_from_gps(
        self,
        latitude_deg: float,
        longitude_deg: float,
        speed_m_s: float | None = None,
        track_deg: float | None = None,
        yaw_rad: float | None = None,
    ) -> None:
        """Initialize from the first good GPS sample."""
        origin = self.set_geodetic_origin(latitude_deg, longitude_deg)
        position = geodetic_to_local_xy_m(latitude_deg, longitude_deg, origin)

        if speed_m_s is not None and track_deg is not None:
            velocity_world = gps_speed_track_to_enu_velocity(speed_m_s, track_deg)
            initial_yaw = (
                wrap_angle_rad(float(yaw_rad))
                if yaw_rad is not None
                else gps_track_deg_to_yaw_rad(track_deg)
            )
            c = math.cos(initial_yaw)
            s = math.sin(initial_yaw)
            velocity_body = np.array(
                [
                    velocity_world[0] * c + velocity_world[1] * s,
                    -velocity_world[0] * s + velocity_world[1] * c,
                ],
                dtype=float,
            )
        else:
            initial_yaw = wrap_angle_rad(float(yaw_rad)) if yaw_rad is not None else 0.0
            velocity_body = np.zeros(2, dtype=float)

        self.initialize(
            position_east_m=position[0],
            position_north_m=position[1],
            yaw_rad=initial_yaw,
            velocity_x_body_mps=velocity_body[0],
            velocity_y_body_mps=velocity_body[1],
            yaw_rate_radps=0.0,
        )

    def propagate_process_model(
        self,
        state: np.ndarray,
        accel_body_x_mps2: float,
        steering_angle_rad: float,
        dt: float,
    ) -> np.ndarray:
        """
        Evaluate the discrete nonlinear process model f_d(x, u).

        This is the point where the EKF and the vehicle dynamics model meet:
        the EKF owns the estimate and covariance, while ``DynamicsModel`` owns
        the motion model used to propagate the state forward in time.
        """
        return self.dynamics_model.predict_ekf_state(
            state=np.asarray(state, dtype=float).reshape(-1),
            accel_body_x_mps2=float(accel_body_x_mps2),
            steering_angle_rad=float(steering_angle_rad),
            dt=dt,
            steering_sign_convention=self.steering_sign_convention,
        )

    def predict(
        self,
        accel_body_x_mps2: float,
        dt: float,
        steering_angle_rad: float = 0.0,
        process_noise: np.ndarray | None = None,
    ) -> EKFEstimate:
        """
        Predict one EKF step with the nonlinear vehicle process model.

        Matrix view of what happens here:

            u_k   = [a_x_meas, delta]^T
            x^-_k = f_d(x^+_{k-1}, u_k)
            F_k   = d f_d / d x
            P^-_k = F_k P^+_{k-1} F_k^T + Q_k

        In this project:
        - the motion/dynamics model provides f_d(x, u)
        - the IMU longitudinal acceleration is used as part of u_k
        - steering angle is the other main input exciting lateral and yaw motion
        - Q_k captures unmodeled dynamics and bias drift
        """
        if not self.initialized:
            raise RuntimeError("EKF must be initialized before predict().")
        if dt <= 0.0:
            raise ValueError("dt must be positive.")

        control = np.array(
            [
                float(accel_body_x_mps2),
                float(steering_angle_rad),
            ],
            dtype=float,
        )

        # Linearize the discrete nonlinear process model around the current
        # estimate. This gives the EKF state-transition Jacobian F_k.
        transition = numerical_jacobian(
            lambda candidate_state: self.propagate_process_model(
                candidate_state,
                accel_body_x_mps2=control[0],
                steering_angle_rad=control[1],
                dt=dt,
            ),
            self.state,
        )

        # Q_k encodes how much uncertainty we inject during prediction due to
        # model mismatch, unmodeled tire behavior, and slow IMU bias drift.
        q = (
            default_process_noise(self.noise, dt)
            if process_noise is None
            else np.asarray(process_noise, dtype=float)
        )

        # x^-_k = f_d(x^+_{k-1}, u_k)
        self.state = self.propagate_process_model(
            self.state,
            accel_body_x_mps2=control[0],
            steering_angle_rad=control[1],
            dt=dt,
        )
        # P^-_k = F_k P^+_{k-1} F_k^T + Q_k
        self.covariance = transition @ self.covariance @ transition.T + q
        self.state[2] = wrap_angle_rad(self.state[2])   # psi
        self.state[3] = wrap_angle_rad(self.state[3])   # phi (roll)
        return self.get_estimate()

    def update(
        self,
        measurement: np.ndarray,
        measurement_model: Callable[[np.ndarray], np.ndarray],
        measurement_jacobian: np.ndarray | None,
        measurement_noise: np.ndarray,
        angle_index: int | None = None,
    ) -> EKFEstimate:
        """
        Generic EKF measurement update.

        Matrix view of what happens here:

            y_k   = z_k - h(x^-_k)
            H_k   = d h / d x
            S_k   = H_k P^-_k H_k^T + R_k
            K_k   = P^-_k H_k^T S_k^{-1}
            x^+_k = x^-_k + K_k y_k
            P^+_k = (I - K_k H_k) P^-_k (I - K_k H_k)^T + K_k R_k K_k^T

        The sensor model enters through ``measurement_model``, which is the
        nonlinear measurement function h(x). GPS position, GPS velocity, yaw,
        and gyro z-rate each use a different h(x) and R matrix.
        """
        if not self.initialized:
            raise RuntimeError("EKF must be initialized before update().")

        # z_k and h(x^-_k)
        z = np.asarray(measurement, dtype=float).reshape(-1)
        h_x = np.asarray(measurement_model(self.state), dtype=float).reshape(-1)

        # Innovation / residual: y_k = z_k - h(x^-_k)
        innovation = z - h_x
        if angle_index is not None:
            innovation[angle_index] = wrap_angle_rad(innovation[angle_index])

        # H_k is either supplied analytically or computed numerically from h(x).
        h = (
            numerical_jacobian(measurement_model, self.state)
            if measurement_jacobian is None
            else np.asarray(measurement_jacobian, dtype=float)
        )
        r = np.asarray(measurement_noise, dtype=float)

        # S_k = H_k P^-_k H_k^T + R_k
        ph_t = self.covariance @ h.T
        s = h @ ph_t + r

        # K_k = P^-_k H_k^T S_k^{-1}
        # Use ``solve`` instead of an explicit inverse for better numerical
        # behavior. This is one of the small implementation details that tends
        # to matter in real filters.
        k = np.linalg.solve(s.T, ph_t.T).T

        # x^+_k = x^-_k + K_k y_k
        self.state = self.state + k @ innovation
        self.state[2] = wrap_angle_rad(self.state[2])   # psi
        self.state[3] = wrap_angle_rad(self.state[3])   # phi (roll)

        # Joseph-form covariance update. This preserves symmetry and tends to be
        # safer numerically than the minimal P = (I - K H) P form.
        identity = np.eye(self.covariance.shape[0], dtype=float)
        correction = identity - k @ h
        self.covariance = correction @ self.covariance @ correction.T + k @ r @ k.T
        return self.get_estimate()

    def update_gps_position(
        self,
        latitude_deg: float,
        longitude_deg: float,
        measurement_noise: np.ndarray | None = None,
    ) -> EKFEstimate:
        """Update with GPS position."""
        if self.origin is None:
            self.set_geodetic_origin(latitude_deg, longitude_deg)

        assert self.origin is not None
        position = geodetic_to_local_xy_m(latitude_deg, longitude_deg, self.origin)
        h = np.zeros((2, len(STATE_VECTOR_2D)), dtype=float)
        h[0, 0] = 1.0
        h[1, 1] = 1.0
        r = (
            default_measurement_noise(self.noise)["gps_position"]
            if measurement_noise is None
            else np.asarray(measurement_noise, dtype=float)
        )
        return self.update(position, h_gps_position_2d, h, r)

    def update_gps_velocity(
        self,
        speed_m_s: float,
        track_deg: float,
        measurement_noise: np.ndarray | None = None,
    ) -> EKFEstimate | None:
        """Update with GPS world-frame velocity when speed is high enough."""
        if float(speed_m_s) < self.low_speed_velocity_update_threshold_mps:
            return None

        velocity_world = gps_speed_track_to_enu_velocity(speed_m_s, track_deg)
        r = (
            default_measurement_noise(self.noise)["gps_velocity"]
            if measurement_noise is None
            else np.asarray(measurement_noise, dtype=float)
        )
        # Use the analytic Jacobian — avoids 16 numerical DynamicsModel calls
        # per GPS velocity update.  See h_gps_velocity_jacobian_2d for the derivation.
        return self.update(velocity_world, h_gps_velocity_2d, h_gps_velocity_jacobian_2d(self.state), r)

    def update_yaw(
        self,
        yaw_rad: float,
        measurement_noise: np.ndarray | None = None,
    ) -> EKFEstimate:
        """Update with IMU fused yaw or another heading estimate."""
        z = np.array([wrap_angle_rad(float(yaw_rad))], dtype=float)
        h = np.zeros((1, len(STATE_VECTOR_2D)), dtype=float)
        h[0, 2] = 1.0
        r = (
            default_measurement_noise(self.noise)["imu_yaw"]
            if measurement_noise is None
            else np.asarray(measurement_noise, dtype=float)
        )
        return self.update(z, h_imu_yaw_2d, h, r, angle_index=0)

    def update_yaw_rate(
        self,
        yaw_rate_radps: float,
        measurement_noise: np.ndarray | None = None,
    ) -> EKFEstimate:
        """Update with gyro z-rate."""
        z = np.array([float(yaw_rate_radps)], dtype=float)
        h = np.zeros((1, len(STATE_VECTOR_2D)), dtype=float)
        h[0, 7] = 1.0   # r   (yaw_rate_radps)
        h[0, 10] = 1.0  # b_gz (gyro z bias)
        r = (
            default_measurement_noise(self.noise)["imu_yaw_rate"]
            if measurement_noise is None
            else np.asarray(measurement_noise, dtype=float)
        )
        return self.update(z, h_imu_yaw_rate_2d, h, r)

    def update_roll(
        self,
        roll_rad: float,
        measurement_noise: np.ndarray | None = None,
    ) -> EKFEstimate:
        """Update with BNO055 fused roll angle."""
        z = np.array([wrap_angle_rad(float(roll_rad))], dtype=float)
        h = np.zeros((1, len(STATE_VECTOR_2D)), dtype=float)
        h[0, 3] = 1.0   # phi (roll_rad)
        r = (
            default_measurement_noise(self.noise)["imu_roll"]
            if measurement_noise is None
            else np.asarray(measurement_noise, dtype=float)
        )
        return self.update(z, h_imu_roll_2d, h, r, angle_index=0)

    def update_pitch(
        self,
        pitch_rad: float,
        measurement_noise: np.ndarray | None = None,
    ) -> EKFEstimate:
        """Update with BNO055 fused pitch angle."""
        z = np.array([float(pitch_rad)], dtype=float)
        h = np.zeros((1, len(STATE_VECTOR_2D)), dtype=float)
        h[0, 4] = 1.0   # theta (pitch_rad)
        r = (
            default_measurement_noise(self.noise)["imu_pitch"]
            if measurement_noise is None
            else np.asarray(measurement_noise, dtype=float)
        )
        return self.update(z, h_imu_pitch_2d, h, r)

    def update_lateral_accel(
        self,
        lateral_accel_mps2: float,
        steering_angle_logged_rad: float,
        measurement_noise: np.ndarray | None = None,
    ) -> EKFEstimate:
        """
        Update with BNO055 gravity-compensated lateral body acceleration.

        The measurement model is nonlinear (tire forces depend on v_x, v_y, r),
        so this update uses a numerical Jacobian computed via central differences.
        At low speed the tire-force model is unreliable; the update is skipped
        when the estimated longitudinal speed is below the low-speed threshold.
        """
        if abs(self.state[5]) < self.low_speed_velocity_update_threshold_mps:
            return self.get_estimate()

        z = np.array([float(lateral_accel_mps2)], dtype=float)
        h_fn = lambda s: h_imu_lateral_accel_2d(s, steering_angle_logged_rad, self.dynamics_model)
        r = (
            default_measurement_noise(self.noise)["imu_lateral_accel"]
            if measurement_noise is None
            else np.asarray(measurement_noise, dtype=float)
        )
        # measurement_jacobian=None tells update() to compute it numerically via h_fn
        return self.update(z, h_fn, None, r)

    def update_from_gps_data(self, gps_data: Mapping[str, object]) -> list[EKFEstimate]:
        """Apply whichever GPS measurements are available in a gpsd-style dict."""
        estimates: list[EKFEstimate] = []
        covariance_overrides = gps_measurement_noise_from_data(gps_data, self.noise)

        latitude = gps_data.get("latitude")
        longitude = gps_data.get("longitude")
        if gps_data.get("has_2d_fix") and latitude is not None and longitude is not None:
            estimates.append(
                self.update_gps_position(
                    float(latitude),
                    float(longitude),
                    measurement_noise=covariance_overrides["gps_position"],
                )
            )

        speed_m_s = gps_data.get("speed_m_s")
        track_deg = gps_data.get("track_deg")
        if speed_m_s is not None and track_deg is not None:
            estimate = self.update_gps_velocity(
                float(speed_m_s),
                float(track_deg),
                measurement_noise=covariance_overrides["gps_velocity"],
            )
            if estimate is not None:
                estimates.append(estimate)

        return estimates


EKF_PSEUDOCODE = """
Startup:
    1. Wait for a good GPS fix.
    2. Define a local ENU origin from the first trustworthy GPS sample.
    3. Initialize the dynamic state:
         x = [p_e, p_n, psi, phi, theta, v_x, v_y, r, b_ax, b_ay, b_gz]^T
    4. Seed yaw, roll, pitch from IMU heading.

Main loop (run at the IMU rate, ~100 Hz):
    1. dt = t_now - t_prev
    2. Read steering angle delta (logged or RC-receiver angle).
    3. Read IMU linear accel x (body) and gyro z.
    4. Predict with:
         u = [a_x_meas, delta]^T
    5. Update with IMU roll    (get_euler_angles()[0]).
    6. Update with IMU pitch   (get_euler_angles()[1]).
    7. Update with IMU yaw     (get_euler_angles()[2])  -- when magnetics are trustworthy.
    8. Update with IMU yaw rate (get_raw_gyro()[2]).
    9. Update with IMU lateral accel (get_linear_acceleration()[1]) -- above low-speed threshold.
    10. Update with GPS position  -- when GPS fix is valid.
    11. Update with GPS velocity  -- above low-speed threshold.

Sensor priority notes:
    - IMU orientation (roll, pitch, yaw) runs at 100 Hz; GPS at 1-10 Hz.
    - GPS position sigma ~ 2.5 m; R_gps_pos is deliberately large to reflect this.
    - GPS velocity sigma ~ 0.05 m/s; GPS velocity is much more reliable than GPS position.
    - Lateral acceleration ties the tire dynamics directly into the velocity estimates.
"""


if __name__ == "__main__":
    ekf = ExtendedKalmanFilter.from_default_config()
    print(f"State vector ({len(STATE_VECTOR_2D)} states):")
    for i, name in enumerate(STATE_VECTOR_2D):
        print(f"  [{i:2d}] {name}")
    print("Input vector:", INPUT_VECTOR_2D)
    print("Config-driven vehicle parameters:")
    print(f"  mass_kg={ekf.vehicle.mass_kg}")
    print(f"  lf_m={ekf.vehicle.front_cg_distance_m}")
    print(f"  lr_m={ekf.vehicle.rear_cg_distance_m}")
    print(f"  Cf={ekf.vehicle.front_cornering_stiffness_nprad}")
    print(f"  Cr={ekf.vehicle.rear_cornering_stiffness_nprad}")
    print("Measurement models:")
    for measurement in MEASUREMENT_MODELS:
        print(f"  - {measurement.name}: {measurement.equation}")
