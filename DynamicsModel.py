from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Mapping

import numpy as np

from ConfigLoader import get_section, load_config_bundle, read_nested

"""
Dynamic single-track (bicycle) vehicle model for the Traxxas Rustler 4x4 RC car.

This module is the *process model* side of the EKF.  It implements the
continuous-time nonlinear derivative

    x_dot = f(x, u)

and a forward-Euler discrete approximation used by the EKF predict step:

    x_k^- ~= x_{k-1}^+ + f(x_{k-1}^+, u_k) * dt

The EKF then linearises f around the current estimate to get the
state-transition Jacobian F_k, which it uses to propagate covariance:

    P_k^- = F_k P_{k-1}^+ F_k^T + Q_k

Primary modelling reference:
  MathWorks, "Modeling a Vehicle Dynamics System"
  https://www.mathworks.com/help/ident/ug/modeling-a-vehicle-dynamics-system.html

Frame conventions (project-wide):
  - Body frame: +X forward, +Y left, +Z up
  - World frame: local ENU  (x = East, y = North)
  - Yaw psi is radians, counter-clockwise from East

EKF state vector used by derivative_from_ekf_state / predict_ekf_state:

    x = [p_e, p_n, psi, phi, theta, v_x, v_y, r, b_ax, b_ay, b_gz]^T
         0    1    2    3    4      5    6    7   8     9     10

    p_e, p_n  -- local East/North position (m)
    psi       -- yaw angle (rad)
    phi       -- roll angle (rad); random-walk in predict, corrected by IMU
    theta     -- pitch angle (rad); random-walk in predict, corrected by IMU
    v_x, v_y  -- body-frame longitudinal and lateral velocity (m/s)
    r         -- yaw rate (rad/s)
    b_ax      -- IMU longitudinal accelerometer bias (m/s^2)
    b_ay      -- IMU lateral accelerometer bias (m/s^2)
    b_gz      -- IMU gyro z-axis bias (rad/s)

Control input vector:

    u = [a_x_meas, delta_logged]^T

    a_x_meas     -- IMU longitudinal acceleration including bias (m/s^2)
    delta_logged -- steering angle from the RC receiver (rad, sign per convention)

Planar dynamics (single-track / bicycle model):

    dot_p_e  = v_x*cos(psi) - v_y*sin(psi)
    dot_p_n  = v_x*sin(psi) + v_y*cos(psi)
    dot_psi  = r
    dot_v_x  = (a_x_meas - b_ax) + v_y*r - C_d*v_x*|v_x|
    dot_v_y  = -v_x*r + (F_yf*cos(delta) + F_yr) / m
    dot_r    = (a*F_yf*cos(delta) - b*F_yr) / I_z
    dot_b_ax = 0   (random-walk bias; Q injects drift uncertainty)
    dot_b_gz = 0

Linear tire forces (first-pass; can be replaced with a nonlinear model later):

    F_yf = 2*C_f*alpha_f
    F_yr = 2*C_r*alpha_r

Small-angle slip angles (linearised about zero sideslip):

    alpha_f = delta_math - (v_y + a*r) / v_x_safe
    alpha_r =            - (v_y - b*r) / v_x_safe

where delta_math is the steering angle in the standard body-XY sign convention
(positive = left turn) and v_x_safe clamps |v_x| >= min_speed to avoid
division by zero at a standstill.

Vehicle parameters (from config/vehicle.yaml):
    m    -- mass (kg)
    a    -- front CG distance (m)
    b    -- rear CG distance (m)
    C_f  -- front cornering stiffness (N/rad)
    C_r  -- rear cornering stiffness (N/rad)
    I_z  -- yaw inertia (kg*m^2)
    C_d  -- longitudinal drag coefficient (currently 0)
"""


@dataclass(frozen=True)
class VehicleParameterDefinition:
    """Vehicle properties required by the dynamics model."""

    mass_kg: float = 5.0
    wheelbase_m: float = 0.36
    front_cg_distance_m: float = 0.18
    rear_cg_distance_m: float = 0.18
    track_width_m: float = 0.30
    cg_height_m: float = 0.06
    yaw_inertia_kgm2: float = 0.6
    front_cornering_stiffness_nprad: float = 45.0
    rear_cornering_stiffness_nprad: float = 45.0
    longitudinal_drag_coefficient: float = 0.0
    min_longitudinal_speed_mps: float = 0.3

    @classmethod
    def from_config_bundle(
        cls,
        config_bundle: dict[str, dict[str, Any]],
    ) -> "VehicleParameterDefinition":
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

        front_cg_distance_m = float(
            read_nested(geometry_cfg, "front_cg_distance_m", default=0.18, unwrap_value=True)
        )
        rear_cg_distance_m = float(
            read_nested(geometry_cfg, "rear_cg_distance_m", default=0.18, unwrap_value=True)
        )
        wheelbase_m = float(read_nested(geometry_cfg, "wheelbase_m", default=0.0, unwrap_value=True))
        if wheelbase_m <= 0.0:
            wheelbase_m = front_cg_distance_m + rear_cg_distance_m

        return cls(
            mass_kg=float(read_nested(mass_cfg, "selected_mass_kg", default=5.0, unwrap_value=True)),
            wheelbase_m=wheelbase_m,
            front_cg_distance_m=front_cg_distance_m,
            rear_cg_distance_m=rear_cg_distance_m,
            track_width_m=float(read_nested(geometry_cfg, "track_width_m", default=0.30, unwrap_value=True)),
            cg_height_m=float(
                read_nested(geometry_cfg, "cg_height_experimental_m", default=0.06, unwrap_value=True)
            ),
            yaw_inertia_kgm2=float(read_nested(mass_cfg, "yaw_inertia_kgm2", default=0.6, unwrap_value=True)),
            front_cornering_stiffness_nprad=float(front_cornering),
            rear_cornering_stiffness_nprad=float(rear_cornering),
            longitudinal_drag_coefficient=float(
                read_nested(longitudinal_cfg, "drag_coefficient_longitudinal", default=0.0, unwrap_value=True)
            ),
            min_longitudinal_speed_mps=float(
                read_nested(longitudinal_cfg, "min_longitudinal_speed_mps", default=0.3, unwrap_value=True)
            ),
        )

    @property
    def lf_m(self) -> float:
        """Compatibility alias used by the EKF process model."""
        return self.front_cg_distance_m

    @property
    def lr_m(self) -> float:
        """Compatibility alias used by the EKF process model."""
        return self.rear_cg_distance_m

    @property
    def cornering_stiffness_front_nprad(self) -> float:
        """Compatibility alias used by the EKF process model."""
        return self.front_cornering_stiffness_nprad

    @property
    def cornering_stiffness_rear_nprad(self) -> float:
        """Compatibility alias used by the EKF process model."""
        return self.rear_cornering_stiffness_nprad

    @property
    def drag_coefficient_longitudinal(self) -> float:
        """Compatibility alias used by the EKF process model."""
        return self.longitudinal_drag_coefficient


@dataclass(frozen=True)
class DynamicsRuntimeConfig:
    """Dynamic-model runtime settings loaded from YAML."""

    model_family: str = "dynamic_single_track"
    steering_sign_convention: str = "negative-left"
    reference_models: tuple[str, ...] = ()
    supporting_references: tuple[str, ...] = ()
    integration_plan: tuple[str, ...] = ()
    state_vector: tuple[str, ...] = ()
    required_inputs: tuple[str, ...] = ()
    optional_inputs: tuple[str, ...] = ()
    derived_quantities: tuple[str, ...] = ()
    equations: dict[str, Any] = field(default_factory=dict)
    handling_logic: dict[str, Any] = field(default_factory=dict)
    validation_checks: tuple[str, ...] = ()
    missing_information: tuple[str, ...] = ()

    @classmethod
    def from_config_bundle(
        cls,
        config_bundle: dict[str, dict[str, Any]],
    ) -> "DynamicsRuntimeConfig":
        dynamics_file = config_bundle.get("dynamics", {})
        dynamics_cfg = get_section(dynamics_file.get("dynamics"), "dynamics")
        frames_cfg = config_bundle.get("frames", {})

        state_vector_cfg = get_section(dynamics_file.get("state_vector"), "state_vector")
        inputs_cfg = get_section(dynamics_file.get("inputs"), "inputs")
        derived_cfg = get_section(dynamics_file.get("derived_quantities"), "derived_quantities")
        equations_cfg = get_section(dynamics_file.get("equations"), "equations")
        handling_cfg = get_section(dynamics_file.get("handling_logic"), "handling_logic")
        validation_cfg = get_section(dynamics_file.get("validation"), "validation")

        return cls(
            model_family=str(dynamics_cfg.get("model_family", "dynamic_single_track")),
            steering_sign_convention=str(
                read_nested(frames_cfg, "steering_convention", "selected_sign", default="negative-left")
            ),
            reference_models=tuple(dynamics_cfg.get("reference_models", ())),
            supporting_references=tuple(dynamics_cfg.get("supporting_references", ())),
            integration_plan=tuple(dynamics_cfg.get("integration_plan", ())),
            state_vector=tuple(state_vector_cfg.get("states", ())),
            required_inputs=tuple(inputs_cfg.get("required_now", ())),
            optional_inputs=tuple(inputs_cfg.get("optional_later", ())),
            derived_quantities=tuple(derived_cfg.get("compute", ())),
            equations=dict(equations_cfg),
            handling_logic=dict(handling_cfg),
            validation_checks=tuple(validation_cfg.get("checks", ())),
            missing_information=tuple(dynamics_file.get("missing_information", ())),
        )


def read_float(mapping: Mapping[str, Any], *keys: str, default: float = 0.0) -> float:
    """Return the first present numeric mapping value from a list of keys."""
    for key in keys:
        if key in mapping and mapping[key] is not None:
            return float(mapping[key])
    return float(default)


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
    steering_sign_convention: str = "negative-left",
) -> float:
    """Convert a logged steering angle into the body-XY math sign convention."""
    return steering_math_sign_multiplier(steering_sign_convention) * float(steering_angle_rad)


def math_angle_to_steering_measurement_rad(
    steering_angle_math_rad: float,
    steering_sign_convention: str = "negative-left",
) -> float:
    """Convert a body-XY math steering angle back to the logged sign convention."""
    return steering_math_sign_multiplier(steering_sign_convention) * float(steering_angle_math_rad)


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


class DynamicsModel:
    """
    Placeholder class describing how the production dynamics model should look.

    The methods below are intentionally pseudocode-oriented. They define the API
    and reasoning flow we want before the full equations are implemented.
    """

    def __init__(
        self,
        config: dict[str, Any] | None = None,
        config_bundle: dict[str, dict[str, Any]] | None = None,
    ) -> None:
        self.config_bundle = load_config_bundle() if config_bundle is None else dict(config_bundle)
        self.config = dict(config) if config is not None else dict(self.config_bundle.get("dynamics", {}))
        self.vehicle_parameters = VehicleParameterDefinition.from_config_bundle(self.config_bundle)
        self.runtime = DynamicsRuntimeConfig.from_config_bundle(self.config_bundle)

    @classmethod
    def from_config_bundle(
        cls,
        config_bundle: dict[str, dict[str, Any]] | None = None,
    ) -> "DynamicsModel":
        """Construct a dynamics-model wrapper from the YAML config bundle."""
        bundle = load_config_bundle() if config_bundle is None else dict(config_bundle)
        return cls(config_bundle=bundle)

    def set_vehicle_parameters(self, vehicle_parameters: VehicleParameterDefinition) -> None:
        """
        Replace the active vehicle-parameter set without rebuilding the model.
        """
        self.vehicle_parameters = vehicle_parameters

    def describe_configuration(self) -> dict[str, Any]:
        """
        Return the key integrated settings used by the dynamics model.
        """
        return {
            "model_family": self.runtime.model_family,
            "steering_sign_convention": self.runtime.steering_sign_convention,
            "reference_models": list(self.runtime.reference_models),
            "supporting_references": list(self.runtime.supporting_references),
            "integration_plan": list(self.runtime.integration_plan),
            "state_vector": list(self.runtime.state_vector),
            "required_inputs": list(self.runtime.required_inputs),
            "optional_inputs": list(self.runtime.optional_inputs),
            "derived_quantities": list(self.runtime.derived_quantities),
            "equations": dict(self.runtime.equations),
            "handling_logic": dict(self.runtime.handling_logic),
            "validation_checks": list(self.runtime.validation_checks),
            "missing_information": list(self.runtime.missing_information),
            "vehicle_parameters": {
                "mass_kg": self.vehicle_parameters.mass_kg,
                "wheelbase_m": self.vehicle_parameters.wheelbase_m,
                "front_cg_distance_m": self.vehicle_parameters.front_cg_distance_m,
                "rear_cg_distance_m": self.vehicle_parameters.rear_cg_distance_m,
                "track_width_m": self.vehicle_parameters.track_width_m,
                "cg_height_m": self.vehicle_parameters.cg_height_m,
                "yaw_inertia_kgm2": self.vehicle_parameters.yaw_inertia_kgm2,
                "front_cornering_stiffness_nprad": self.vehicle_parameters.front_cornering_stiffness_nprad,
                "rear_cornering_stiffness_nprad": self.vehicle_parameters.rear_cornering_stiffness_nprad,
            },
        }

    def compute_slip_angles(
        self,
        state: Mapping[str, float],
        inputs: Mapping[str, float],
        steering_sign_convention: str | None = None,
    ) -> dict[str, float]:
        """
        Compute front and rear tire slip angles for the single-track model.

        The slip angle of a tire is the angle between the direction the tire is
        *pointing* and the direction it is actually *moving*.  A non-zero slip
        angle generates lateral (cornering) force — the fundamental mechanism
        behind steering response and handling balance.

        Using the small-angle linearisation (adequate when sideslip is small):

            alpha_f = delta_math - (v_y + a*r) / v_x
            alpha_r =            - (v_y - b*r) / v_x

        The numerator (v_y + a*r) is the lateral velocity of the *front axle*
        in the body frame — the sum of the CG lateral velocity and the
        contribution from rotation (a * yaw_rate).  Dividing by v_x converts
        it to an angle.  The front steering angle delta_math then represents
        how much the front wheel is turned away from that travel direction.

        The rear axle has no steering, so alpha_r is purely the sideslip at
        the rear (v_y - b*r), negated by sign convention.

        v_x_safe clamps the denominator away from zero so the equations stay
        finite at a standstill.  The sign of v_x is preserved so that reversing
        produces sensible (negative-vx) slip angles.
        """
        sign_convention = steering_sign_convention or self.runtime.steering_sign_convention
        v_x = read_float(state, "v_x_body_mps", "longitudinal_velocity_mps", "v_x", default=0.0)
        v_y = read_float(state, "v_y_body_mps", "lateral_velocity_mps", "v_y", default=0.0)
        yaw_rate = read_float(state, "yaw_rate_radps", "r", default=0.0)
        steering_logged_rad = read_float(inputs, "steering_angle_rad", default=0.0)

        # Convert the logged steering sign into the standard body-XY math convention
        # (positive delta_math = left turn = positive yaw direction).
        steering_math_rad = steering_measurement_to_math_angle_rad(steering_logged_rad, sign_convention)

        # Clamp |v_x| to avoid division by zero; preserve the sign so reverse works.
        v_x_safe = math.copysign(
            max(abs(v_x), self.vehicle_parameters.min_longitudinal_speed_mps),
            v_x if v_x != 0.0 else 1.0,
        )

        # Front slip: steer angle minus the angle at which the front axle is
        # actually moving laterally (= (v_y + a*r) / v_x in small-angle form).
        alpha_f = steering_math_rad - (
            v_y + self.vehicle_parameters.front_cg_distance_m * yaw_rate
        ) / v_x_safe

        # Rear slip: purely from lateral motion of the rear axle.
        # The rear wheels point straight ahead, so there is no steering term.
        alpha_r = -(
            v_y - self.vehicle_parameters.rear_cg_distance_m * yaw_rate
        ) / v_x_safe

        return {
            "front_slip_angle_rad": alpha_f,
            "rear_slip_angle_rad": alpha_r,
            "steering_angle_logged_rad": steering_logged_rad,
            "steering_angle_math_rad": steering_math_rad,
            "safe_longitudinal_velocity_mps": v_x_safe,
        }

    def compute_lateral_tire_forces(self, slip_angles: Mapping[str, float]) -> dict[str, float]:
        """
        Compute first-pass linear lateral tire forces from slip angles.
        """
        front_slip_angle = read_float(slip_angles, "front_slip_angle_rad", default=0.0)
        rear_slip_angle = read_float(slip_angles, "rear_slip_angle_rad", default=0.0)

        front_lateral_force = 2.0 * self.vehicle_parameters.front_cornering_stiffness_nprad * front_slip_angle
        rear_lateral_force = 2.0 * self.vehicle_parameters.rear_cornering_stiffness_nprad * rear_slip_angle

        return {
            "front_lateral_force_n": front_lateral_force,
            "rear_lateral_force_n": rear_lateral_force,
            "total_lateral_force_n": front_lateral_force + rear_lateral_force,
        }

    def compute_planar_state_derivative(
        self,
        state: Mapping[str, float],
        inputs: Mapping[str, float],
        steering_sign_convention: str | None = None,
    ) -> dict[str, float]:
        """
        Evaluate the continuous-time nonlinear derivative  x_dot = f(x, u).

        This is the core of the process model.  Every term comes from Newton's
        second law applied to the single-track vehicle:

        Position kinematics — rotate body-frame velocity into world frame:
            dot_p_e = v_x*cos(psi) - v_y*sin(psi)
            dot_p_n = v_x*sin(psi) + v_y*cos(psi)
            dot_psi = r

        Longitudinal dynamics (Newton F=ma along body X):
            dot_v_x = a_x_corr + v_y*r - C_d*v_x*|v_x|

            - a_x_corr = a_x_meas - b_ax  (IMU reading minus estimated bias)
            - v_y*r is the centripetal acceleration coupling: when the car
              rotates, lateral velocity spills into the longitudinal direction
            - C_d*v_x*|v_x| is aerodynamic drag (currently zero in config)

        Lateral dynamics (Newton F=ma along body Y):
            dot_v_y = -v_x*r + (F_yf*cos(delta) + F_yr) / m

            - -v_x*r is the centripetal term in the rotating body frame
              (equal and opposite to v_y*r above but for the lateral axis)
            - F_yf*cos(delta) projects the front tire force along body Y
              (small delta means cos ≈ 1, so this is approximately just F_yf)
            - F_yr is the rear lateral tire force

        Yaw dynamics (moment equation about the vertical CG axis):
            dot_r = (a*F_yf*cos(delta) - b*F_yr) / I_z

            - a * F_yf creates a yaw moment that turns the car in the same
              direction as the front wheels are steered (stabilising under
              normal cornering)
            - b * F_yr opposes that moment (rear grip resists yaw)
            - The ratio of these determines oversteer vs understeer tendency

        Bias states are modelled as random-walk constants (dot = 0).  The
        process-noise matrix Q tells the EKF how fast they are allowed to drift.

        Note: GPS and IMU sensor corrections do *not* appear here.  This is the
        pure physics prediction.  The EKF update step applies sensor corrections
        on top of whatever this function predicts.
        """
        yaw_rad = read_float(state, "yaw_rad", "psi", default=0.0)
        v_x = read_float(state, "v_x_body_mps", "longitudinal_velocity_mps", "v_x", default=0.0)
        v_y = read_float(state, "v_y_body_mps", "lateral_velocity_mps", "v_y", default=0.0)
        yaw_rate = read_float(state, "yaw_rate_radps", "r", default=0.0)
        accel_bias_x = read_float(state, "accel_bias_x_mps2", default=0.0)

        slip_angles = self.compute_slip_angles(
            state,
            inputs,
            steering_sign_convention=steering_sign_convention,
        )
        tire_forces = self.compute_lateral_tire_forces(slip_angles)
        steering_math_rad = slip_angles["steering_angle_math_rad"]

        mass_kg = max(self.vehicle_parameters.mass_kg, 1e-9)
        yaw_inertia_kgm2 = max(self.vehicle_parameters.yaw_inertia_kgm2, 1e-9)
        longitudinal_drag = self.vehicle_parameters.longitudinal_drag_coefficient
        accel_body_x_mps2 = read_float(inputs, "imu_longitudinal_accel_mps2", "a_x_input_mps2", default=0.0)

        # Subtract the estimated accelerometer bias before using the IMU reading
        # as a forcing input.  The bias state is corrected by the EKF update step.
        corrected_accel_body_x_mps2 = accel_body_x_mps2 - accel_bias_x

        # Rotate body velocity into world frame for the position derivatives.
        world_velocity = body_velocity_to_world_velocity(v_x, v_y, yaw_rad)
        front_lateral_force = tire_forces["front_lateral_force_n"]
        rear_lateral_force = tire_forces["rear_lateral_force_n"]

        # cos(delta) projects front tire force onto body-Y and creates the
        # yaw moment arm.  For typical RC car steering (|delta| < 60 deg),
        # cos(delta) is between 0.5 and 1.0, so the approximation cos ≈ 1 is
        # acceptable at small angles but not at full lock.
        cos_delta = math.cos(steering_math_rad)

        # dot_v_x: longitudinal accel + centripetal coupling - drag
        dot_v_x = corrected_accel_body_x_mps2 + v_y * yaw_rate - longitudinal_drag * v_x * abs(v_x)

        # dot_v_y: centripetal term + net lateral tire force / mass
        dot_v_y = -v_x * yaw_rate + (front_lateral_force * cos_delta + rear_lateral_force) / mass_kg

        # dot_r: net yaw torque / yaw inertia
        dot_yaw_rate = (
            self.vehicle_parameters.front_cg_distance_m * front_lateral_force * cos_delta
            - self.vehicle_parameters.rear_cg_distance_m * rear_lateral_force
        ) / yaw_inertia_kgm2

        return {
            "p_e_m": world_velocity[0],
            "p_n_m": world_velocity[1],
            "yaw_rad": yaw_rate,        # dot_psi = r
            "v_x_body_mps": dot_v_x,
            "v_y_body_mps": dot_v_y,
            "yaw_rate_radps": dot_yaw_rate,
            "roll_rad": 0.0,            # placeholder — driven by IMU for now
            "pitch_rad": 0.0,           # placeholder — driven by IMU for now
            "accel_bias_x_mps2": 0.0,  # random walk; Q handles uncertainty
            "gyro_bias_z_radps": 0.0,  # random walk; Q handles uncertainty
        }

    def compute_body_accelerations(
        self,
        state: Mapping[str, float],
        inputs: Mapping[str, float],
        steering_sign_convention: str | None = None,
    ) -> dict[str, float]:
        """
        Estimate body-frame accelerations associated with the current model state and inputs.
        """
        slip_angles = self.compute_slip_angles(
            state,
            inputs,
            steering_sign_convention=steering_sign_convention,
        )
        tire_forces = self.compute_lateral_tire_forces(slip_angles)
        mass_kg = max(self.vehicle_parameters.mass_kg, 1e-9)
        accel_bias_x = read_float(state, "accel_bias_x_mps2", default=0.0)
        accel_body_x_mps2 = read_float(inputs, "imu_longitudinal_accel_mps2", "a_x_input_mps2", default=0.0) - accel_bias_x
        accel_body_y_mps2 = (
            tire_forces["front_lateral_force_n"] * math.cos(slip_angles["steering_angle_math_rad"])
            + tire_forces["rear_lateral_force_n"]
        ) / mass_kg

        return {
            "longitudinal_acceleration_mps2": accel_body_x_mps2,
            "lateral_acceleration_mps2": accel_body_y_mps2,
        }

    def compute_load_transfer(
        self,
        state: Mapping[str, float],
        inputs: Mapping[str, float],
        steering_sign_convention: str | None = None,
    ) -> dict[str, float]:
        """
        Estimate first-pass longitudinal and lateral load transfer.
        """
        body_acceleration = self.compute_body_accelerations(
            state,
            inputs,
            steering_sign_convention=steering_sign_convention,
        )
        weight_n = self.vehicle_parameters.mass_kg * 9.80665
        wheelbase_m = max(self.vehicle_parameters.wheelbase_m, 1e-9)
        track_width_m = max(self.vehicle_parameters.track_width_m, 1e-9)
        cg_height_m = self.vehicle_parameters.cg_height_m

        longitudinal_load_transfer_n = (
            weight_n * body_acceleration["longitudinal_acceleration_mps2"] * cg_height_m / wheelbase_m
        )
        lateral_load_transfer_n = (
            weight_n * body_acceleration["lateral_acceleration_mps2"] * cg_height_m / track_width_m
        )

        return {
            "longitudinal_load_transfer_n": longitudinal_load_transfer_n,
            "lateral_load_transfer_n": lateral_load_transfer_n,
        }

    def classify_handling_balance(self, slip_angles: Mapping[str, float]) -> str:
        """
        Compare front and rear slip-angle magnitude to classify handling balance.
        """
        front_slip = abs(read_float(slip_angles, "front_slip_angle_rad", default=0.0))
        rear_slip = abs(read_float(slip_angles, "rear_slip_angle_rad", default=0.0))
        heuristic_margin_rad = math.radians(1.0)

        if front_slip > rear_slip + heuristic_margin_rad:
            return "understeer"
        if rear_slip > front_slip + heuristic_margin_rad:
            return "oversteer"
        return "neutral"

    def estimate_optimal_steering_angle(
        self,
        desired_turn_radius_m: float,
        speed_mps: float,
        steering_sign_convention: str | None = None,
    ) -> float:
        """
        Return a first-pass steering recommendation based on Ackermann geometry.
        """
        del speed_mps  # Reserved for later dynamic correction terms.

        sign_convention = steering_sign_convention or self.runtime.steering_sign_convention
        signed_turn_radius_m = float(desired_turn_radius_m)
        if not math.isfinite(signed_turn_radius_m) or abs(signed_turn_radius_m) < 1e-9:
            return 0.0

        steering_math_rad = math.atan2(
            self.vehicle_parameters.wheelbase_m,
            abs(signed_turn_radius_m),
        ) * math.copysign(1.0, signed_turn_radius_m)
        return math_angle_to_steering_measurement_rad(steering_math_rad, sign_convention)

    def derivative_from_ekf_state(
        self,
        state: np.ndarray,
        accel_body_x_mps2: float,
        steering_angle_rad: float,
        steering_sign_convention: str | None = None,
    ) -> np.ndarray:
        """
        Compute the EKF-compatible derivative vector from the shared motion model.

        This is the bridge from the named vehicle-dynamics state to the EKF's
        packed matrix/vector form:

            x = [p_e, p_n, psi, phi, theta, v_x, v_y, r, b_ax, b_ay, b_gz]^T
            0    1    2    3    4     5      6    7   8    9     10

        phi (roll) and theta (pitch) are currently measured by the BNO055 and
        injected directly through update_roll / update_pitch; their derivatives
        are zero in the predict step (random-walk model — uncertainty grows via Q).
        b_ay is likewise a random-walk bias; its derivative is zero.

        The EKF needs this vector form so it can build Jacobians and propagate
        covariance, but the underlying equations still come from the vehicle
        dynamics model.
        """
        candidate_state = np.asarray(state, dtype=float).reshape(-1)
        if candidate_state.shape[0] != 11:
            raise ValueError(f"Expected EKF state size 11, got {candidate_state.shape[0]}.")

        state_dict = {
            "p_e_m": candidate_state[0],
            "p_n_m": candidate_state[1],
            "yaw_rad": candidate_state[2],
            # phi (index 3) and theta (index 4) are not inputs to the planar model
            "v_x_body_mps": candidate_state[5],
            "v_y_body_mps": candidate_state[6],
            "yaw_rate_radps": candidate_state[7],
            "accel_bias_x_mps2": candidate_state[8],
            "gyro_bias_z_radps": candidate_state[10],
        }
        input_dict = {
            "imu_longitudinal_accel_mps2": float(accel_body_x_mps2),
            "steering_angle_rad": float(steering_angle_rad),
        }
        derivative = self.compute_planar_state_derivative(
            state_dict,
            input_dict,
            steering_sign_convention=steering_sign_convention,
        )

        return np.array(
            [
                derivative["p_e_m"],           # 0  dot_p_e
                derivative["p_n_m"],           # 1  dot_p_n
                derivative["yaw_rad"],         # 2  dot_psi
                0.0,                           # 3  dot_phi   -- random walk, driven by IMU
                0.0,                           # 4  dot_theta -- random walk, driven by IMU
                derivative["v_x_body_mps"],    # 5  dot_v_x
                derivative["v_y_body_mps"],    # 6  dot_v_y
                derivative["yaw_rate_radps"],  # 7  dot_r
                derivative["accel_bias_x_mps2"], # 8 dot_b_ax  -- random walk
                0.0,                           # 9  dot_b_ay  -- random walk
                derivative["gyro_bias_z_radps"], # 10 dot_b_gz -- random walk
            ],
            dtype=float,
        )

    def predict_ekf_state(
        self,
        state: np.ndarray,
        accel_body_x_mps2: float,
        steering_angle_rad: float,
        dt: float,
        steering_sign_convention: str | None = None,
    ) -> np.ndarray:
        """
        Discretize the motion model for the EKF predict step.

        This is the discrete approximation used by the filter:

            x^-_k = f_d(x^+_{k-1}, u_k)
                  ~= x^+_{k-1} + f(x^+_{k-1}, u_k) * dt

        The EKF then linearizes this discrete mapping to obtain F_k for the
        covariance prediction.
        """
        if dt <= 0.0:
            raise ValueError("dt must be positive.")

        predicted_state = np.asarray(state, dtype=float).reshape(-1) + self.derivative_from_ekf_state(
            state=state,
            accel_body_x_mps2=accel_body_x_mps2,
            steering_angle_rad=steering_angle_rad,
            steering_sign_convention=steering_sign_convention,
        ) * dt
        predicted_state[2] = wrap_angle_rad(predicted_state[2])
        return predicted_state

    def step(self, state: Mapping[str, float], inputs: Mapping[str, float], dt: float) -> dict[str, Any]:
        """
        Advance the nonlinear vehicle model one step and return derived dynamics quantities.

        This method is useful for analysis and logging outside the EKF because
        it shows the same motion model alongside derived quantities such as:
        - slip angles
        - tire forces
        - load transfer
        - handling-balance classification

        In EKF terms:
        - ``predict_ekf_state`` is the part that directly feeds the filter
        - ``step`` is the richer post-processing view built on the same model
        """
        if dt <= 0.0:
            raise ValueError("dt must be positive.")

        derivative = self.compute_planar_state_derivative(state, inputs)
        slip_angles = self.compute_slip_angles(state, inputs)
        tire_forces = self.compute_lateral_tire_forces(slip_angles)
        body_acceleration = self.compute_body_accelerations(state, inputs)
        load_transfer = self.compute_load_transfer(state, inputs)
        handling_balance = self.classify_handling_balance(slip_angles)

        predicted_state = {
            "p_e_m": read_float(state, "p_e_m", "position_east_m", "p_e", default=0.0) + derivative["p_e_m"] * dt,
            "p_n_m": read_float(state, "p_n_m", "position_north_m", "p_n", default=0.0) + derivative["p_n_m"] * dt,
            "yaw_rad": wrap_angle_rad(read_float(state, "yaw_rad", "psi", default=0.0) + derivative["yaw_rad"] * dt),
            "v_x_body_mps": read_float(state, "v_x_body_mps", "longitudinal_velocity_mps", "v_x", default=0.0)
            + derivative["v_x_body_mps"] * dt,
            "v_y_body_mps": read_float(state, "v_y_body_mps", "lateral_velocity_mps", "v_y", default=0.0)
            + derivative["v_y_body_mps"] * dt,
            "yaw_rate_radps": read_float(state, "yaw_rate_radps", "r", default=0.0) + derivative["yaw_rate_radps"] * dt,
            "roll_rad": read_float(state, "roll_rad", "roll", default=0.0),
            "pitch_rad": read_float(state, "pitch_rad", "pitch", default=0.0),
            "accel_bias_x_mps2": read_float(state, "accel_bias_x_mps2", default=0.0),
            "gyro_bias_z_radps": read_float(state, "gyro_bias_z_radps", default=0.0),
        }

        speed_mps = math.hypot(predicted_state["v_x_body_mps"], predicted_state["v_y_body_mps"])
        yaw_rate_radps = predicted_state["yaw_rate_radps"]
        signed_turn_radius_m = math.inf if abs(yaw_rate_radps) < 1e-9 or speed_mps < 1e-9 else speed_mps / yaw_rate_radps
        ackermann_steering_angle_rad = self.estimate_optimal_steering_angle(signed_turn_radius_m, speed_mps)

        return {
            "state": predicted_state,
            "state_derivative": derivative,
            "speed_mps": speed_mps,
            "slip_angles": slip_angles,
            "tire_forces": tire_forces,
            "body_acceleration": body_acceleration,
            "load_transfer": load_transfer,
            "handling_balance": handling_balance,
            "understeer_indicator": handling_balance == "understeer",
            "oversteer_indicator": handling_balance == "oversteer",
            "ackermann_steering_angle_rad": ackermann_steering_angle_rad,
            "optimal_steering_angle_rad": ackermann_steering_angle_rad,
        }


if __name__ == "__main__":
    model = DynamicsModel.from_config_bundle()
    print("DynamicsModel loaded from config")
    print("Vehicle parameters:", model.describe_configuration()["vehicle_parameters"])
    print("Steering convention:", model.describe_configuration()["steering_sign_convention"])

    # Quick smoke test: step the model from rest with a small steering input.
    import numpy as np
    x0 = np.zeros(8, dtype=float)
    x0[3] = 2.0  # start at 2 m/s longitudinal
    x1 = model.predict_ekf_state(
        state=x0,
        accel_body_x_mps2=0.5,
        steering_angle_rad=0.1,
        dt=0.05,
    )
    print("State after one 50 ms step:", x1)
