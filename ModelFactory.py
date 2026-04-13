from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from ConfigLoader import get_section, load_config_bundle, read_nested
from DynamicsModel import DynamicsModel
from ExtendedKalmanFilter import ExtendedKalmanFilter


@dataclass(frozen=True)
class FrameRuntimeConfig:
    """
    Shared coordinate-frame and steering-sign conventions.
    """

    body_x_axis: str = "forward"
    body_y_axis: str = "left"
    body_z_axis: str = "up"
    world_type: str = "local_enu"
    world_x_axis: str = "east"
    world_y_axis: str = "north"
    world_z_axis: str = "up"
    yaw_positive_rotation: str = "counter_clockwise"
    yaw_zero_direction_world: str = "east"
    steering_sign_convention: str = "negative-left"
    steering_min_deg: float = -60.0
    steering_max_deg: float = 60.0

    @classmethod
    def from_config_bundle(
        cls,
        config_bundle: dict[str, dict[str, Any]],
    ) -> "FrameRuntimeConfig":
        frames_cfg = config_bundle.get("frames", {})
        body_cfg = get_section(frames_cfg.get("body_frame"), "body_frame")
        world_cfg = get_section(frames_cfg.get("world_frame"), "world_frame")
        heading_cfg = get_section(frames_cfg.get("heading_convention"), "heading_convention")
        steering_cfg = get_section(frames_cfg.get("steering_convention"), "steering_convention")
        steering_range_cfg = get_section(steering_cfg.get("mapped_angle_range_deg"), "mapped_angle_range_deg")

        return cls(
            body_x_axis=str(body_cfg.get("x_axis", "forward")),
            body_y_axis=str(body_cfg.get("y_axis", "left")),
            body_z_axis=str(body_cfg.get("z_axis", "up")),
            world_type=str(world_cfg.get("type", "local_enu")),
            world_x_axis=str(world_cfg.get("x_axis", "east")),
            world_y_axis=str(world_cfg.get("y_axis", "north")),
            world_z_axis=str(world_cfg.get("z_axis", "up")),
            yaw_positive_rotation=str(heading_cfg.get("positive_rotation", "counter_clockwise")),
            yaw_zero_direction_world=str(heading_cfg.get("zero_direction_world", "east")),
            steering_sign_convention=str(steering_cfg.get("selected_sign", "negative-left")),
            steering_min_deg=float(steering_range_cfg.get("min", -60.0)),
            steering_max_deg=float(steering_range_cfg.get("max", 60.0)),
        )


@dataclass(frozen=True)
class ControllerRuntimeConfig:
    """
    Runtime flags that determine how the controller should use the models.
    """

    enabled: bool = True
    verbose: bool = True
    loop_rate_hz: float = 10.0
    loop_delay_sec: float = 0.1
    logging_enabled: bool = True
    logging_directory: str = "data"
    logging_filename_stem: str = "data"
    planned_additional_columns: tuple[str, ...] = ()
    calibrate_imu_on_startup: bool = True
    reset_receiver_on_startup: bool = True
    initialize_ekf_from_first_good_gps_fix: bool = True
    enable_ekf: bool = True
    enable_dynamics_model: bool = True
    use_default_receiver_if_missing: bool = True
    use_default_odometry_if_missing: bool = True

    @classmethod
    def from_config_bundle(
        cls,
        config_bundle: dict[str, dict[str, Any]],
    ) -> "ControllerRuntimeConfig":
        controller_file = config_bundle.get("controller", {})
        controller_cfg = get_section(controller_file.get("controller"), "controller")
        logging_cfg = get_section(controller_file.get("logging"), "logging")
        startup_cfg = get_section(controller_file.get("startup"), "startup")
        runtime_cfg = get_section(controller_file.get("runtime"), "runtime")

        return cls(
            enabled=bool(controller_cfg.get("enabled", True)),
            verbose=bool(controller_cfg.get("verbose", True)),
            loop_rate_hz=float(controller_cfg.get("loop_rate_hz", 10.0)),
            loop_delay_sec=float(controller_cfg.get("loop_delay_sec", 0.1)),
            logging_enabled=bool(logging_cfg.get("enabled", True)),
            logging_directory=str(logging_cfg.get("directory", "data")),
            logging_filename_stem=str(logging_cfg.get("filename_stem", "data")),
            planned_additional_columns=tuple(logging_cfg.get("planned_additional_columns", ())),
            calibrate_imu_on_startup=bool(startup_cfg.get("calibrate_imu_on_startup", True)),
            reset_receiver_on_startup=bool(startup_cfg.get("reset_receiver_on_startup", True)),
            initialize_ekf_from_first_good_gps_fix=bool(
                startup_cfg.get("initialize_ekf_from_first_good_gps_fix", True)
            ),
            enable_ekf=bool(runtime_cfg.get("enable_ekf", True)),
            enable_dynamics_model=bool(runtime_cfg.get("enable_dynamics_model", True)),
            use_default_receiver_if_missing=bool(runtime_cfg.get("use_default_receiver_if_missing", True)),
            use_default_odometry_if_missing=bool(runtime_cfg.get("use_default_odometry_if_missing", True)),
        )


@dataclass(frozen=True)
class IntegratedModelBundle:
    """
    Shared runtime bundle for config-driven model construction.
    """

    config_bundle: dict[str, dict[str, Any]]
    frames: FrameRuntimeConfig
    controller: ControllerRuntimeConfig
    ekf: ExtendedKalmanFilter
    dynamics_model: DynamicsModel

    def describe(self) -> dict[str, Any]:
        """
        Return a compact integration summary for debugging and startup logs.
        """
        ekf_cfg = get_section(self.config_bundle.get("ekf"), "ekf")
        return {
            "frames": {
                "body_axes": [self.frames.body_x_axis, self.frames.body_y_axis, self.frames.body_z_axis],
                "world_frame": self.frames.world_type,
                "world_axes": [self.frames.world_x_axis, self.frames.world_y_axis, self.frames.world_z_axis],
                "steering_sign_convention": self.frames.steering_sign_convention,
                "steering_angle_range_deg": [self.frames.steering_min_deg, self.frames.steering_max_deg],
            },
            "controller": {
                "enabled": self.controller.enabled,
                "loop_rate_hz": self.controller.loop_rate_hz,
                "logging_enabled": self.controller.logging_enabled,
                "enable_ekf": self.controller.enable_ekf,
                "enable_dynamics_model": self.controller.enable_dynamics_model,
            },
            "ekf": {
                "enabled": bool(ekf_cfg.get("enabled", True)),
                "model_type": str(ekf_cfg.get("model_type", "")),
                "steering_sign_convention": self.ekf.steering_sign_convention,
                "low_speed_velocity_update_threshold_mps": self.ekf.low_speed_velocity_update_threshold_mps,
                "state_size": len(self.ekf.state),
            },
            "dynamics": self.dynamics_model.describe_configuration(),
            "consistency_checks": {
                "shared_vehicle_mass_kg": self.ekf.vehicle.mass_kg,
                "shared_front_cg_distance_m": self.ekf.vehicle.lf_m,
                "shared_rear_cg_distance_m": self.ekf.vehicle.lr_m,
                "ekf_and_dynamics_share_steering_sign": (
                    self.ekf.steering_sign_convention == self.dynamics_model.runtime.steering_sign_convention
                ),
                "ekf_and_frames_share_steering_sign": (
                    self.ekf.steering_sign_convention == self.frames.steering_sign_convention
                ),
                "dynamics_and_frames_share_steering_sign": (
                    self.dynamics_model.runtime.steering_sign_convention == self.frames.steering_sign_convention
                ),
                "throttle_planned_for_future_input": bool(
                    read_nested(self.config_bundle.get("ekf"), "future_inputs", "use_throttle_if_available", default=False)
                ),
            },
        }


def build_integrated_model_bundle(
    config_bundle: dict[str, dict[str, Any]] | None = None,
) -> IntegratedModelBundle:
    """
    Build the EKF and dynamics model from one shared YAML config bundle.
    """
    bundle = load_config_bundle() if config_bundle is None else dict(config_bundle)
    dynamics_model = DynamicsModel.from_config_bundle(bundle)
    return IntegratedModelBundle(
        config_bundle=bundle,
        frames=FrameRuntimeConfig.from_config_bundle(bundle),
        controller=ControllerRuntimeConfig.from_config_bundle(bundle),
        ekf=ExtendedKalmanFilter.from_config_bundle(bundle, dynamics_model=dynamics_model),
        dynamics_model=dynamics_model,
    )


if __name__ == "__main__":
    runtime_bundle = build_integrated_model_bundle()
    print("Integrated model bundle created from config:")
    print("  EKF vehicle mass (kg):", runtime_bundle.ekf.vehicle.mass_kg)
    print("  Dynamics model family:", runtime_bundle.dynamics_model.runtime.model_family)
    print("  Shared steering sign:", runtime_bundle.frames.steering_sign_convention)
