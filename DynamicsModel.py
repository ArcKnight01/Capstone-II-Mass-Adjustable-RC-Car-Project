from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Mapping

import numpy as np

from ConfigLoader import get_section, load_config_bundle, read_nested

"""
Dynamics model pseudocode scaffold for the Traxxas Rustler 4x4 project.

Primary reference:
- MathWorks, "Modeling a Vehicle Dynamics System"
  https://www.mathworks.com/help/ident/ug/modeling-a-vehicle-dynamics-system.html

Additional local context used to shape this scaffold:
- RC_vehicle_dynamics_tables_summary.md
- RC_vehicle_dynamics_tables.xlsx / Equations.xlsx derived notes
- project frame convention: +X forward, +Y left, +Z up

This file is intentionally pseudocode-first. The goal is to define what the
model should represent, what signals it needs, and what downstream quantities it
should produce before we harden the equations into production code.

What we want to observe or estimate:
- pitch
- yaw
- roll
- braking force
- motor / torque input
- speed
- acceleration

What we want to use the model for:
- oversteer detection
- understeer detection
- slip-angle estimation
- load-transfer estimation
- optimal steering-angle estimation

Core modeling note:
- The MathWorks reference is centered on dynamic states v_x, v_y, and yaw rate r.
- For this RC car project, that should be the backbone of the planar model.
- Roll and pitch should initially be treated as measured / derived attitude
  channels from the IMU and then connected into load-transfer logic.

Kalman-filter role:
- This file is the process / motion model side of the EKF.
- In the notation used by Roger Labbe's "Kalman and Bayesian Filters in Python":

      x_k = f(x_{k-1}, u_k) + w_k

  this file provides the nonlinear function f(x, u), or more precisely the
  continuous-time derivative x_dot = f(x, u) and the first-order discrete
  approximation:

      x_k ~= x_{k-1} + f(x_{k-1}, u_k) * dt

- The sensor model lives in the EKF measurement functions h(x), while this file
  explains how the vehicle state should evolve before sensor corrections.
"""


@dataclass(frozen=True)
class DynamicsStateDefinition:
    """Named state channels for the first-pass dynamics model."""

    position_east_m: float = 0.0
    position_north_m: float = 0.0
    yaw_rad: float = 0.0
    longitudinal_velocity_mps: float = 0.0
    lateral_velocity_mps: float = 0.0
    yaw_rate_radps: float = 0.0
    roll_rad: float = 0.0
    pitch_rad: float = 0.0


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
class InputSignalDefinition:
    """Signals we either already have or may want to add."""

    steering_angle_rad: float = 0.0
    throttle_command: float = 0.0
    brake_command: float = 0.0
    motor_torque_estimate_nm: float = 0.0
    front_left_wheel_slip: float = 0.0
    front_right_wheel_slip: float = 0.0
    rear_left_wheel_slip: float = 0.0
    rear_right_wheel_slip: float = 0.0
    imu_longitudinal_accel_mps2: float = 0.0
    imu_lateral_accel_mps2: float = 0.0
    imu_yaw_rate_radps: float = 0.0


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


STATE_VECTOR_PLAN = [
    "p_e",
    "p_n",
    "psi",
    "v_x",
    "v_y",
    "r",
    "roll",
    "pitch",
]

INPUT_VECTOR_PLAN = [
    "steering_angle",
    "throttle_or_motor_input",
    "brake_input",
    "optional_wheel_slips",
    "imu_longitudinal_accel",
    "imu_lateral_accel",
]

OUTPUT_VECTOR_PLAN = [
    "speed",
    "longitudinal_acceleration",
    "lateral_acceleration",
    "yaw_rate",
    "front_slip_angle",
    "rear_slip_angle",
    "lateral_load_transfer",
    "longitudinal_load_transfer",
    "understeer_gradient",
    "optimal_steering_angle",
]


DYNAMICS_MODEL_PSEUDOCODE = """
1. Load vehicle parameters
   Inputs needed from config / measurement:
   - mass m
   - wheelbase l
   - front CG distance a
   - rear CG distance b
   - track width t
   - CG height h
   - yaw inertia Iz
   - front cornering stiffness Cf
   - rear cornering stiffness Cr
   - drag coefficient Cd_long

2. Gather sensor / control inputs for the current timestep
   Required now:
   - steering angle delta
   - IMU yaw rate r_meas
   - IMU longitudinal acceleration a_x_meas
   - IMU lateral acceleration a_y_meas
   - GPS speed or fused speed estimate
   - yaw estimate from IMU/GPS fusion

   Optional but very useful later:
   - throttle / ESC command
   - brake command
   - motor torque estimate
   - wheel speeds or wheel slips

3. Define state vector
   x = [
       p_e,    # local East position
       p_n,    # local North position
       psi,    # yaw angle
       v_x,    # body longitudinal velocity
       v_y,    # body lateral velocity
       r,      # yaw rate
       phi,    # roll angle
       theta,  # pitch angle
   ]

4. Compute body-frame kinematics
   speed = sqrt(v_x^2 + v_y^2)
   world_velocity = R_body_to_world(psi) @ [v_x, v_y]
   p_e_dot = world_velocity_x
   p_n_dot = world_velocity_y
   psi_dot = r

5. Compute tire slip angles
   If v_x is too small:
       clamp v_x to a minimum safe value to avoid division by zero

   front_slip_angle alpha_f =
       delta - atan((v_y + a*r) / v_x)

   rear_slip_angle alpha_r =
       -atan((v_y - b*r) / v_x)

   Small-angle linearized form for fast model / estimator use:
       alpha_f ~= delta - (v_y + a*r)/v_x
       alpha_r ~= -(v_y - b*r)/v_x

6. Compute lateral tire forces
   First-pass linear tire model:
       F_yf = 2 * Cf * alpha_f
       F_yr = 2 * Cr * alpha_r

   Later upgrade path:
   - saturation / nonlinear tire model
   - separate left/right tire loads
   - surface-dependent stiffness

7. Compute longitudinal force model
   Option A: if throttle / motor input is not available yet
       approximate longitudinal excitation using IMU longitudinal acceleration
       and treat motor/brake force as an unknown effective input

   Option B: if throttle or motor torque is available
       map command -> drive torque -> wheel force
       subtract drag and rolling resistance

   Option C: if wheel speed data becomes available
       compute longitudinal slip and use a better tire-force model

8. Propagate planar dynamics
   Dynamic-state backbone inspired by the MathWorks reference:
       v_x_dot = v_y*r + (sum longitudinal forces)/m - drag_term
       v_y_dot = -v_x*r + (front lateral contribution + rear lateral contribution)/m
       r_dot   = (front yaw moment - rear yaw moment) / Iz

   First-pass single-track form:
       v_x_dot = a_x_input + v_y*r - drag_term
       v_y_dot = -v_x*r + (F_yf*cos(delta) + F_yr)/m
       r_dot   = (a*F_yf*cos(delta) - b*F_yr)/Iz

9. Attach roll and pitch channels
   Initial use:
   - take roll and pitch from IMU fused orientation
   - do not fully model suspension dynamics yet

   Later use:
   - create roll and pitch dynamic states driven by lateral and longitudinal acceleration
   - connect them to load transfer and tire normal loads

10. Estimate load transfer
   Longitudinal load transfer:
       delta_W_x = W * (A_x * h / l)

   Lateral load transfer:
       delta_W_y = W * (A_y * h / t)

   Use these to estimate:
   - front vs rear axle load changes under braking / acceleration
   - left vs right load changes in cornering
   - when inside wheels may begin to unload

11. Estimate handling balance
   Understeer / oversteer logic can start from:
   - compare demanded steering delta to kinematic/Ackermann steering for the same turn radius
   - compare measured yaw response vs expected yaw response
   - compare front and rear slip-angle trends

   Heuristic:
   - if |alpha_f| > |alpha_r| by a meaningful margin -> understeer tendency
   - if |alpha_r| > |alpha_f| by a meaningful margin -> oversteer tendency

12. Estimate optimal steering angle
   First pass:
   - compute kinematic / Ackermann steering for desired turn radius:
         delta_ack ~= atan(l / R)

   Dynamic correction:
   - increase or decrease delta based on speed, lateral acceleration, and understeer gradient

   Candidate output:
       delta_optimal = delta_ack + delta_dynamic_correction

13. Output quantities for controller / logger / visualization
   Save or return:
   - speed
   - a_x, a_y
   - yaw rate
   - front/rear slip angles
   - lateral and longitudinal load transfer
   - understeer / oversteer indicator
   - recommended steering angle

14. Validation workflow
   Before trusting the model:
   - verify steering sign convention
   - verify yaw sign convention
   - verify v_x remains positive in straight-line runs
   - compare predicted yaw rate against IMU gyro z
   - compare predicted lateral acceleration against IMU lateral accel
   - compare estimated turn radius against GPS track curvature
"""


MISSING_INFORMATION_CHECKLIST = [
    "Measured wheelbase and track width for the current Rustler setup",
    "CG location relative to front and rear axles for the as-run mass configuration",
    "CG height used for load-transfer calculations",
    "Yaw inertia estimate or measurement",
    "Front and rear cornering stiffness estimates for the current tire/surface setup",
    "Whether steering angle is true road-wheel angle or only servo command angle",
    "Whether throttle / ESC command can be logged reliably",
    "Whether brake command can be measured separately from throttle",
    "Whether wheel speed or motor RPM sensing will be added later",
    "Trusted sign conventions for steering, yaw, and lateral acceleration",
]


IMPLEMENTATION_ORDER = [
    "Phase 1: Build a planar dynamic single-track model with v_x, v_y, r, psi, p_e, p_n",
    "Phase 2: Feed roll and pitch from IMU measurements into load-transfer estimation",
    "Phase 3: Add throttle / brake or motor-force input when available",
    "Phase 4: Add wheel-speed-based longitudinal slip if sensors are added",
    "Phase 5: Tune cornering stiffness and yaw inertia from logged test data",
]


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
        Compute front and rear slip angles for the first-pass dynamic single-track model.
        """
        sign_convention = steering_sign_convention or self.runtime.steering_sign_convention
        v_x = read_float(state, "v_x_body_mps", "longitudinal_velocity_mps", "v_x", default=0.0)
        v_y = read_float(state, "v_y_body_mps", "lateral_velocity_mps", "v_y", default=0.0)
        yaw_rate = read_float(state, "yaw_rate_radps", "r", default=0.0)
        steering_logged_rad = read_float(inputs, "steering_angle_rad", default=0.0)
        steering_math_rad = steering_measurement_to_math_angle_rad(steering_logged_rad, sign_convention)
        v_x_safe = math.copysign(
            max(abs(v_x), self.vehicle_parameters.min_longitudinal_speed_mps),
            v_x if v_x != 0.0 else 1.0,
        )

        alpha_f = steering_math_rad - (
            v_y + self.vehicle_parameters.front_cg_distance_m * yaw_rate
        ) / v_x_safe
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
        Compute the continuous-time motion model x_dot = f(x, u).

        State used here:
            x = [p_e, p_n, psi, v_x, v_y, r, ...]^T

        Inputs used here:
            u = [delta, a_x_meas, ...]^T

        Interaction between the dynamics model and the sensor model:
        - The IMU longitudinal acceleration is treated as a known input during
          prediction.
        - Steering angle determines front tire slip angle and therefore lateral
          tire force.
        - GPS and IMU sensor models do not appear here; they correct this
          predicted motion later during the EKF update step.
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
        corrected_accel_body_x_mps2 = accel_body_x_mps2 - accel_bias_x

        world_velocity = body_velocity_to_world_velocity(v_x, v_y, yaw_rad)
        front_lateral_force = tire_forces["front_lateral_force_n"]
        rear_lateral_force = tire_forces["rear_lateral_force_n"]

        dot_v_x = corrected_accel_body_x_mps2 + v_y * yaw_rate - longitudinal_drag * v_x * abs(v_x)
        dot_v_y = -v_x * yaw_rate + (front_lateral_force * math.cos(steering_math_rad) + rear_lateral_force) / mass_kg
        dot_yaw_rate = (
            self.vehicle_parameters.front_cg_distance_m * front_lateral_force * math.cos(steering_math_rad)
            - self.vehicle_parameters.rear_cg_distance_m * rear_lateral_force
        ) / yaw_inertia_kgm2

        return {
            "p_e_m": world_velocity[0],
            "p_n_m": world_velocity[1],
            "yaw_rad": yaw_rate,
            "v_x_body_mps": dot_v_x,
            "v_y_body_mps": dot_v_y,
            "yaw_rate_radps": dot_yaw_rate,
            "roll_rad": 0.0,
            "pitch_rad": 0.0,
            "accel_bias_x_mps2": 0.0,
            "gyro_bias_z_radps": 0.0,
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

            x = [p_e, p_n, psi, v_x, v_y, r, b_ax, b_gz]^T

        The EKF needs this vector form so it can build Jacobians and propagate
        covariance, but the underlying equations still come from the vehicle
        dynamics model.
        """
        candidate_state = np.asarray(state, dtype=float).reshape(-1)
        if candidate_state.shape[0] != 8:
            raise ValueError(f"Expected EKF state size 8, got {candidate_state.shape[0]}.")

        state_dict = {
            "p_e_m": candidate_state[0],
            "p_n_m": candidate_state[1],
            "yaw_rad": candidate_state[2],
            "v_x_body_mps": candidate_state[3],
            "v_y_body_mps": candidate_state[4],
            "yaw_rate_radps": candidate_state[5],
            "accel_bias_x_mps2": candidate_state[6],
            "gyro_bias_z_radps": candidate_state[7],
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
                derivative["p_e_m"],
                derivative["p_n_m"],
                derivative["yaw_rad"],
                derivative["v_x_body_mps"],
                derivative["v_y_body_mps"],
                derivative["yaw_rate_radps"],
                derivative["accel_bias_x_mps2"],
                derivative["gyro_bias_z_radps"],
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
    print("Dynamics model pseudocode scaffold")
    print("State plan:", STATE_VECTOR_PLAN)
    print("Input plan:", INPUT_VECTOR_PLAN)
    print("Output plan:", OUTPUT_VECTOR_PLAN)
    print("Config-driven summary:", model.describe_configuration())
    print("\nImplementation order:")
    for item in IMPLEMENTATION_ORDER:
        print(f"  - {item}")
