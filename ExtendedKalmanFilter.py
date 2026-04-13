from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable, Mapping

import numpy as np

from ConfigLoader import get_section, load_config_bundle, read_nested
from DynamicsModel import DynamicsModel, VehicleParameterDefinition

"""
Extended Kalman filter starter for the RC car project.

This version replaces the earlier kinematic/bicycle-style propagation with a
dynamic planar vehicle model that is closer to the MathWorks reference example:
https://www.mathworks.com/help/ident/ug/modeling-a-vehicle-dynamics-system.html

The MathWorks example uses longitudinal velocity, lateral velocity, and yaw
rate as the core vehicle-dynamics states. That is a better conceptual match for
your project than a purely kinematic steering model.

Reference / attribution note:
- The overall EKF class lifecycle here is still inspired by the public
  Janudis/Extended-Kalman-Filter-GPS_IMU repository and the paper cited in its
  README, but the actual propagation model below is adapted toward the
  higher-fidelity dynamic-vehicle structure discussed in the MathWorks example.

Project frame conventions:
- Body frame: +X forward, +Y left, +Z up
- Local world frame: x = East, y = North
- Yaw psi is radians, counter-clockwise from East

Steering sign note:
- The project steering sign may still be uncertain, so it remains configurable.
- In the body frame above, positive planar math rotation means left.

Notation used in the comments below follows the EKF presentation style used in
Roger Labbe's "Kalman and Bayesian Filters in Python":
- repo README:
  https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/README.md
- EKF chapter notebook:
  https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/11-Extended-Kalman-Filters.ipynb
- nonlinear-filter design appendix:
  https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/Appendix-G-Designing-Nonlinear-Kalman-Filters.ipynb

For this project the nonlinear EKF is organized as:

    x_k   = f(x_{k-1}, u_k) + w_k
    z_k   = h(x_k) + v_k

where
- x is the state vector we want to estimate
- u is the known input applied during the predict step
- z is a sensor measurement used in the update step
- w ~ N(0, Q) is process noise
- v ~ N(0, R) is measurement noise

Predict step:

    x^-_k = f(x^+_{k-1}, u_k)
    F_k   = d f / d x |_{x^+_{k-1}, u_k}
    P^-_k = F_k P^+_{k-1} F_k^T + Q_k

Update step:

    y_k   = z_k - h(x^-_k)               # innovation / residual
    H_k   = d h / d x |_{x^-_k}
    S_k   = H_k P^-_k H_k^T + R_k        # innovation covariance
    K_k   = P^-_k H_k^T S_k^{-1}         # Kalman gain
    x^+_k = x^-_k + K_k y_k
    P^+_k = (I - K_k H_k) P^-_k (I - K_k H_k)^T + K_k R_k K_k^T

The last covariance equation is the Joseph form. It is a bit more expensive
than the minimal textbook form, but it is numerically safer.
"""


STATE_VECTOR_2D = (
    "p_e_m",
    "p_n_m",
    "yaw_rad",
    "v_x_body_mps",
    "v_y_body_mps",
    "yaw_rate_radps",
    "accel_bias_x_mps2",
    "gyro_bias_z_radps",
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
    imu_yaw_rate_sigma_radps: float = 0.0014

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
            imu_yaw_rate_sigma_radps=float(
                measurement_noise.get(
                    "imu_yaw_rate_sigma_radps",
                    read_nested(imu_noise, "yaw_rate_sigma_radps", default=0.0014),
                )
            ),
            accel_bias_walk_sigma_mps3=float(process_noise.get("accel_bias_walk_sigma_mps3", 0.05)),
            gyro_bias_walk_sigma_radps2=float(process_noise.get("gyro_bias_walk_sigma_radps2", 0.01)),
            model_velocity_sigma_mps2=float(process_noise.get("model_velocity_sigma_mps2", 1.5)),
            model_yaw_accel_sigma_radps2=float(process_noise.get("model_yaw_accel_sigma_radps2", 2.0)),
        )


@dataclass(frozen=True)
class VehicleParameters:
    """
    Dynamic planar vehicle parameters.

    These are placeholders until you identify values for the actual RC car.
    """

    mass_kg: float = 5.0
    yaw_inertia_kgm2: float = 0.6
    lf_m: float = 0.18
    lr_m: float = 0.18
    cornering_stiffness_front_nprad: float = 45.0
    cornering_stiffness_rear_nprad: float = 45.0
    drag_coefficient_longitudinal: float = 0.0
    min_longitudinal_speed_mps: float = 0.3

    @classmethod
    def from_config_bundle(cls, config_bundle: Mapping[str, Mapping[str, object]]) -> "VehicleParameters":
        vehicle_cfg = config_bundle.get("vehicle", {})
        geometry_cfg = get_section(vehicle_cfg.get("geometry"), "geometry")
        mass_cfg = get_section(vehicle_cfg.get("mass_properties"), "mass_properties")
        tire_cfg = get_section(vehicle_cfg.get("tire_model"), "tire_model")
        longitudinal_cfg = get_section(vehicle_cfg.get("longitudinal_model"), "longitudinal_model")

        active_tires = get_section(tire_cfg.get("active_configuration"), "active_configuration")
        tire_catalog = get_section(tire_cfg.get("tire_catalog"), "tire_catalog")
        axle_defaults = get_section(tire_cfg.get("axle_level_defaults"), "axle_level_defaults")

        front_tire_name = active_tires.get("front_tire_type")
        rear_tire_name = active_tires.get("rear_tire_type")
        front_tire_cfg = get_section(tire_catalog.get(front_tire_name), str(front_tire_name)) if front_tire_name else {}
        rear_tire_cfg = get_section(tire_catalog.get(rear_tire_name), str(rear_tire_name)) if rear_tire_name else {}

        front_cornering = read_nested(
            front_tire_cfg,
            "cornering_stiffness_nprad",
            default=read_nested(
                axle_defaults,
                "front_cornering_stiffness_nprad",
                default=45.0,
                unwrap_value=True,
            ),
        )
        rear_cornering = read_nested(
            rear_tire_cfg,
            "cornering_stiffness_nprad",
            default=read_nested(
                axle_defaults,
                "rear_cornering_stiffness_nprad",
                default=45.0,
                unwrap_value=True,
            ),
        )

        return cls(
            mass_kg=float(read_nested(mass_cfg, "selected_mass_kg", default=2.667, unwrap_value=True)),
            yaw_inertia_kgm2=float(read_nested(mass_cfg, "yaw_inertia_kgm2", default=0.6, unwrap_value=True)),
            lf_m=float(read_nested(geometry_cfg, "front_cg_distance_m", default=0.18, unwrap_value=True)),
            lr_m=float(read_nested(geometry_cfg, "rear_cg_distance_m", default=0.18, unwrap_value=True)),
            cornering_stiffness_front_nprad=float(front_cornering),
            cornering_stiffness_rear_nprad=float(rear_cornering),
            drag_coefficient_longitudinal=float(
                read_nested(longitudinal_cfg, "drag_coefficient_longitudinal", default=0.0, unwrap_value=True)
            ),
            min_longitudinal_speed_mps=float(
                read_nested(longitudinal_cfg, "min_longitudinal_speed_mps", default=0.3, unwrap_value=True)
            ),
        )

    @classmethod
    def from_dynamics_vehicle_parameters(
        cls,
        vehicle_parameters: VehicleParameterDefinition,
    ) -> "VehicleParameters":
        """Create EKF vehicle parameters from the shared dynamics-model parameter set."""
        return cls(
            mass_kg=float(vehicle_parameters.mass_kg),
            yaw_inertia_kgm2=float(vehicle_parameters.yaw_inertia_kgm2),
            lf_m=float(vehicle_parameters.front_cg_distance_m),
            lr_m=float(vehicle_parameters.rear_cg_distance_m),
            cornering_stiffness_front_nprad=float(vehicle_parameters.front_cornering_stiffness_nprad),
            cornering_stiffness_rear_nprad=float(vehicle_parameters.rear_cornering_stiffness_nprad),
            drag_coefficient_longitudinal=float(vehicle_parameters.longitudinal_drag_coefficient),
            min_longitudinal_speed_mps=float(vehicle_parameters.min_longitudinal_speed_mps),
        )

    def to_dynamics_vehicle_parameters(
        self,
        base_parameters: VehicleParameterDefinition | None = None,
    ) -> VehicleParameterDefinition:
        """Convert EKF vehicle parameters into the shared dynamics-model structure."""
        return VehicleParameterDefinition(
            mass_kg=float(self.mass_kg),
            wheelbase_m=float(
                base_parameters.wheelbase_m if base_parameters is not None else (self.lf_m + self.lr_m)
            ),
            front_cg_distance_m=float(self.lf_m),
            rear_cg_distance_m=float(self.lr_m),
            track_width_m=float(base_parameters.track_width_m if base_parameters is not None else 0.30),
            cg_height_m=float(base_parameters.cg_height_m if base_parameters is not None else 0.06),
            yaw_inertia_kgm2=float(self.yaw_inertia_kgm2),
            front_cornering_stiffness_nprad=float(self.cornering_stiffness_front_nprad),
            rear_cornering_stiffness_nprad=float(self.cornering_stiffness_rear_nprad),
            longitudinal_drag_coefficient=float(self.drag_coefficient_longitudinal),
            min_longitudinal_speed_mps=float(self.min_longitudinal_speed_mps),
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
    velocity_sigma = float(covariance_cfg.get("velocity_sigma_mps", 2.0))
    yaw_rate_sigma = float(covariance_cfg.get("yaw_rate_sigma_radps", math.radians(20.0)))
    accel_bias_sigma = float(covariance_cfg.get("accel_bias_sigma_mps2", 1.0))
    gyro_bias_sigma = float(covariance_cfg.get("gyro_bias_sigma_radps", math.radians(5.0)))

    return np.diag(
        [
            position_sigma**2,
            position_sigma**2,
            yaw_sigma**2,
            velocity_sigma**2,
            velocity_sigma**2,
            yaw_rate_sigma**2,
            accel_bias_sigma**2,
            gyro_bias_sigma**2,
        ]
    )


MEASUREMENT_MODELS = (
    MeasurementDefinition(
        name="gps_position",
        source_file="GPS_System.py",
        source_fields=("latitude", "longitude", "epx_m", "epy_m"),
        equation="z_pos = [p_e, p_n]^T + v",
        covariance_hint="Use gps epx/epy when present, else 2.5 m std dev",
    ),
    MeasurementDefinition(
        name="gps_velocity",
        source_file="GPS_System.py",
        source_fields=("speed_m_s", "track_deg", "eps_m_s"),
        equation="z_vel = [v_e, v_n]^T + v",
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
        name="imu_yaw_rate",
        source_file="IMU.py",
        source_fields=("get_raw_gyro()[2]",),
        equation="z_r = r + b_gz + v",
        covariance_hint="Approx 0.0014 rad/s std dev after unit conversion",
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


def dynamic_vehicle_state_derivative_2d(
    state: np.ndarray,
    control: np.ndarray,
    vehicle: VehicleParameters,
    steering_sign_convention: str = DEFAULT_STEERING_SIGN_CONVENTION,
) -> np.ndarray:
    """
    Backward-compatible wrapper around the shared nonlinear process model.

    In EKF notation this function returns the continuous-time derivative
    associated with x_dot = f(x, u). The actual math now lives in
    ``DynamicsModel.derivative_from_ekf_state(...)`` so the EKF and the
    standalone dynamics model share one motion model.

    State:
        x = [p_e, p_n, psi, v_x, v_y, r, b_ax, b_gz]^T

    Control:
        u = [a_x_meas, delta_logged]^T
    """
    x = np.asarray(state, dtype=float).reshape(-1)
    u = np.asarray(control, dtype=float).reshape(-1)

    if x.shape[0] != len(STATE_VECTOR_2D):
        raise ValueError(f"Expected state size {len(STATE_VECTOR_2D)}, got {x.shape[0]}.")
    if u.shape[0] != len(INPUT_VECTOR_2D):
        raise ValueError(f"Expected control size {len(INPUT_VECTOR_2D)}, got {u.shape[0]}.")

    model = DynamicsModel.from_config_bundle({})
    model.set_vehicle_parameters(vehicle.to_dynamics_vehicle_parameters(model.vehicle_parameters))
    return model.derivative_from_ekf_state(
        state=x,
        accel_body_x_mps2=float(u[0]),
        steering_angle_rad=float(u[1]),
        steering_sign_convention=steering_sign_convention,
    )


def predict_state_2d(
    state: np.ndarray,
    control: np.ndarray,
    dt: float,
    vehicle: VehicleParameters,
    steering_sign_convention: str = DEFAULT_STEERING_SIGN_CONVENTION,
) -> np.ndarray:
    """
    Discrete-time propagation helper for the nonlinear motion model.

    This implements the discrete process map used by the EKF predict step:

        x^-_k = f_d(x^+_{k-1}, u_k)

    using a first-order Euler discretization of the continuous dynamics:

        f_d(x, u) ~= x + f(x, u) * dt
    """
    if dt <= 0.0:
        raise ValueError("dt must be positive.")

    model = DynamicsModel.from_config_bundle({})
    model.set_vehicle_parameters(vehicle.to_dynamics_vehicle_parameters(model.vehicle_parameters))
    return model.predict_ekf_state(
        state=state,
        accel_body_x_mps2=float(np.asarray(control, dtype=float).reshape(-1)[0]),
        steering_angle_rad=float(np.asarray(control, dtype=float).reshape(-1)[1]),
        dt=dt,
        steering_sign_convention=steering_sign_convention,
    )


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

    return np.diag(
        [
            1e-4,
            1e-4,
            yaw_var,
            velocity_var,
            velocity_var,
            yaw_var,
            accel_bias_var,
            gyro_bias_var,
        ]
    )


def default_measurement_noise(noise: NoiseParameters = NoiseParameters()) -> dict[str, np.ndarray]:
    """Return default measurement covariance matrices."""
    return {
        "gps_position": np.diag([noise.gps_position_sigma_m**2, noise.gps_position_sigma_m**2]),
        "gps_velocity": np.diag([noise.gps_velocity_sigma_mps**2, noise.gps_velocity_sigma_mps**2]),
        "imu_yaw": np.array([[noise.imu_yaw_sigma_rad**2]], dtype=float),
        "imu_yaw_rate": np.array([[noise.imu_yaw_rate_sigma_radps**2]], dtype=float),
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

    The state stores velocity in the body frame, so this measurement function
    applies the yaw-dependent rotation:

        z_vel = h_vel(x) + v
              = R_body_to_world(psi) [v_x, v_y]^T + v
    """
    x = np.asarray(state, dtype=float).reshape(-1)
    _p_e, _p_n, psi, v_x, v_y, _r, _b_ax, _b_gz = x
    return body_velocity_to_world_velocity(v_x, v_y, psi)


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
    return np.array([x[5] + x[7]], dtype=float)


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
        vehicle: VehicleParameters | None = None,
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
        if vehicle is None:
            if dynamics_model is not None:
                resolved_vehicle = VehicleParameters.from_dynamics_vehicle_parameters(
                    self.dynamics_model.vehicle_parameters
                )
            elif self.config_bundle is not None:
                resolved_vehicle = VehicleParameters.from_config_bundle(self.config_bundle)
            else:
                resolved_vehicle = VehicleParameters()
        else:
            resolved_vehicle = vehicle

        self.vehicle = resolved_vehicle
        self.dynamics_model.set_vehicle_parameters(
            self.vehicle.to_dynamics_vehicle_parameters(self.dynamics_model.vehicle_parameters)
        )
        self.noise = noise
        self.steering_sign_convention = steering_sign_convention
        self.low_speed_velocity_update_threshold_mps = float(low_speed_velocity_update_threshold_mps)

        self.state = np.zeros(len(STATE_VECTOR_2D), dtype=float)
        self.covariance = (
            np.asarray(initial_covariance, dtype=float)
            if initial_covariance is not None
            else np.diag(
                [
                    10.0**2,
                    10.0**2,
                    math.radians(30.0) ** 2,
                    2.0**2,
                    2.0**2,
                    math.radians(20.0) ** 2,
                    1.0**2,
                    math.radians(5.0) ** 2,
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
            vehicle=VehicleParameters.from_config_bundle(bundle),
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
        velocity_x_body_mps: float = 0.0,
        velocity_y_body_mps: float = 0.0,
        yaw_rate_radps: float = 0.0,
        covariance: np.ndarray | None = None,
    ) -> None:
        """Initialize the EKF state directly in the chosen local/world frames."""
        self.state = np.array(
            [
                float(position_east_m),
                float(position_north_m),
                wrap_angle_rad(float(yaw_rad)),
                float(velocity_x_body_mps),
                float(velocity_y_body_mps),
                float(yaw_rate_radps),
                0.0,
                0.0,
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
        self.state[2] = wrap_angle_rad(self.state[2])
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
        self.state[2] = wrap_angle_rad(self.state[2])

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
        return self.update(velocity_world, h_gps_velocity_2d, None, r)

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
        h[0, 5] = 1.0
        h[0, 7] = 1.0
        r = (
            default_measurement_noise(self.noise)["imu_yaw_rate"]
            if measurement_noise is None
            else np.asarray(measurement_noise, dtype=float)
        )
        return self.update(z, h_imu_yaw_rate_2d, h, r)

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
         x = [p_e, p_n, psi, v_x, v_y, r, b_ax, b_gz]^T
    4. Seed yaw from IMU heading or GPS track if available.

Main loop:
    1. dt = t_now - t_prev
    2. Read steering angle delta.
    3. Read IMU linear accel x and gyro z.
    4. Predict with:
         u = [a_x_meas, delta]^T
    5. Update with GPS position if fix exists.
    6. Update with GPS velocity if speed is above threshold.
    7. Update with IMU yaw rate from gyro z.
    8. Optionally update with fused IMU yaw when magnetics are trustworthy.

This model is closer to the MathWorks vehicle-dynamics example because it
propagates longitudinal velocity, lateral velocity, and yaw rate as dynamic
states rather than treating steering as purely geometric curvature.
"""


if __name__ == "__main__":
    ekf = ExtendedKalmanFilter.from_default_config()
    print("State vector:", STATE_VECTOR_2D)
    print("Input vector:", INPUT_VECTOR_2D)
    print("Config-driven vehicle parameters:")
    print(f"  mass_kg={ekf.vehicle.mass_kg}")
    print(f"  lf_m={ekf.vehicle.lf_m}")
    print(f"  lr_m={ekf.vehicle.lr_m}")
    print(f"  Cf={ekf.vehicle.cornering_stiffness_front_nprad}")
    print(f"  Cr={ekf.vehicle.cornering_stiffness_rear_nprad}")
    print("Measurement models:")
    for measurement in MEASUREMENT_MODELS:
        print(f"  - {measurement.name}: {measurement.equation}")
