"""Reconstruct velocity and position from a legacy odometry CSV.

This script is intentionally temporary and mirrors the integration logic in
Odometry.py:

    velocity = previous_velocity + 0.5 * (acceleration + previous_acceleration) * dt
    position = previous_position + 0.5 * (velocity + previous_velocity) * dt

It reads the legacy ``data/data.csv`` format that contains time, body-frame
linear acceleration, and absolute orientation, rotates the acceleration into the
world frame, then writes an enriched CSV that can be used by ``Animate.py``.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

from MovingAverageFilter import MovingAverageFilter
from LowPassFilter import LowPassFilter
from HighPassFilter import HighPassFilter


REPO_ROOT = Path(__file__).resolve().parent
DEFAULT_INPUT = REPO_ROOT / "data" / "data.csv"
DEFAULT_OUTPUT = REPO_ROOT / "data" / "simulated_data.csv"
REQUIRED_COLUMNS = [
    "time",
    "acceleration_x",
    "acceleration_y",
    "acceleration_z",
    "roll",
    "pitch",
    "yaw",
]


def parse_args() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Rebuild integrated velocity and position from a legacy data.csv log."
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=DEFAULT_INPUT,
        help=f"Legacy CSV file to read. Default: {DEFAULT_INPUT}",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help=f"CSV file to write. Default: {DEFAULT_OUTPUT}",
    )
    parser.add_argument(
        "--filter-gyro",
        action="store_true",
        help="Apply a moving-average filter to gyro columns before exporting canonical IMU gyro outputs.",
    )
    parser.add_argument(
        "--gyro-filter-window-size",
        type=int,
        default=3,
        help="Moving-average window size for gyro filtering. Default: 3",
    )
    parser.add_argument(
        "--filter-linear-acceleration",
        action="store_true",
        help="Apply a moving-average filter to body-frame linear acceleration before rotation/integration.",
    )
    parser.add_argument(
        "--linear-acceleration-filter-window-size",
        type=int,
        default=3,
        help="Moving-average window size for linear-acceleration filtering. Default: 3",
    )
    parser.add_argument(
        "--low-pass-linear-acceleration",
        action="store_true",
        help="Apply a first-order low-pass filter to body-frame linear acceleration.",
    )
    parser.add_argument(
        "--linear-acceleration-low-pass-cutoff-hz",
        type=float,
        default=2.0,
        help="Cutoff frequency in hertz for linear-acceleration low-pass filtering. Default: 2.0",
    )
    return parser.parse_args()


def load_legacy_csv(path: Path) -> pd.DataFrame:
    """Load and validate a legacy log file."""
    df = pd.read_csv(path)
    missing = [column for column in REQUIRED_COLUMNS if column not in df.columns]
    if missing:
        raise ValueError(
            f"{path} is missing required columns: {', '.join(missing)}"
        )
    if df.empty:
        raise ValueError(f"{path} has no data rows to integrate.")
    return df


def numeric_series(df: pd.DataFrame, column: str) -> pd.Series:
    """Return a numeric version of a column."""
    return pd.to_numeric(df[column], errors="coerce")


def choose_positive_fallback(values: np.ndarray, default: float = 0.0) -> float:
    """Choose a fallback step size from the positive values in an array."""
    positive_values = values[np.isfinite(values) & (values > 0)]
    if len(positive_values) == 0:
        return default
    return float(np.median(positive_values))


def build_loop_dt(time_values: np.ndarray) -> np.ndarray:
    """Construct per-row dt values from the elapsed time column."""
    loop_dt = np.diff(time_values, prepend=0.0)
    fallback_dt = choose_positive_fallback(loop_dt, default=0.0)

    if not np.isfinite(loop_dt[0]) or loop_dt[0] < 0:
        loop_dt[0] = fallback_dt

    for index in range(1, len(loop_dt)):
        if not np.isfinite(loop_dt[index]) or loop_dt[index] <= 0:
            loop_dt[index] = fallback_dt

    return loop_dt


def wrap_angle_degrees(values: np.ndarray) -> np.ndarray:
    """Wrap degree values into the ``[-180, 180)`` interval."""
    return (values + 180.0) % 360.0 - 180.0


def euler_degrees_to_rotation_matrix(
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
) -> np.ndarray:
    """Build a body-to-world rotation matrix from absolute roll, pitch, and yaw."""
    roll = np.deg2rad(roll_deg)
    pitch = np.deg2rad(pitch_deg)
    yaw = np.deg2rad(yaw_deg)

    cr = np.cos(roll)
    sr = np.sin(roll)
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    cy = np.cos(yaw)
    sy = np.sin(yaw)

    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def euler_degrees_to_quaternion(
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
) -> np.ndarray:
    """Convert absolute roll, pitch, and yaw into a ``(w, x, y, z)`` quaternion."""
    roll = np.deg2rad(roll_deg) * 0.5
    pitch = np.deg2rad(pitch_deg) * 0.5
    yaw = np.deg2rad(yaw_deg) * 0.5

    cr = np.cos(roll)
    sr = np.sin(roll)
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    cy = np.cos(yaw)
    sy = np.sin(yaw)

    return np.array(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ],
        dtype=float,
    )


def rotate_body_acceleration_to_world(
    body_acceleration: np.ndarray,
    absolute_orientation: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Rotate body-frame acceleration into the world frame and derive quaternions."""
    world_acceleration = np.zeros_like(body_acceleration, dtype=float)
    quaternions = np.zeros((len(body_acceleration), 4), dtype=float)

    for index in range(len(body_acceleration)):
        roll_deg, pitch_deg, yaw_deg = absolute_orientation[index]
        rotation_matrix = euler_degrees_to_rotation_matrix(roll_deg, pitch_deg, yaw_deg)
        world_acceleration[index] = rotation_matrix @ body_acceleration[index]
        quaternions[index] = euler_degrees_to_quaternion(roll_deg, pitch_deg, yaw_deg)

    return world_acceleration, quaternions


def integrate_trapezoidal(acceleration: np.ndarray, loop_dt: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Integrate acceleration into velocity and position using the Odometry.py equations."""
    velocity = np.zeros_like(acceleration, dtype=float)
    position = np.zeros_like(acceleration, dtype=float)

    previous_acceleration = np.zeros(3, dtype=float)
    previous_velocity = np.zeros(3, dtype=float)
    previous_position = np.zeros(3, dtype=float)

    for index in range(len(acceleration)):
        current_acceleration = acceleration[index]
        dt = float(loop_dt[index])

        current_velocity = previous_velocity + 0.5 * (
            current_acceleration + previous_acceleration
        ) * dt
        current_position = previous_position + 0.5 * (
            current_velocity + previous_velocity
        ) * dt

        velocity[index] = current_velocity
        position[index] = current_position

        previous_acceleration = current_acceleration
        previous_velocity = current_velocity
        previous_position = current_position

    return velocity, position


def apply_vector_moving_average(values: np.ndarray, window_size: int) -> np.ndarray:
    """Apply one moving-average filter per axis to an ``(n, 3)`` array."""
    array = np.asarray(values, dtype=float)
    if array.ndim != 2 or array.shape[1] != 3:
        raise ValueError(f"Expected an (n, 3) array, got shape {array.shape}")

    filters = [MovingAverageFilter(window_size) for _ in range(3)]
    filtered = np.zeros_like(array, dtype=float)
    for index, row in enumerate(array):
        filtered[index] = [axis_filter.update(component) for axis_filter, component in zip(filters, row)]
    return filtered


def apply_vector_low_pass(values: np.ndarray, loop_dt: np.ndarray, cutoff_hz: float) -> np.ndarray:
    """Apply one first-order low-pass filter per axis to an ``(n, 3)`` array."""
    array = np.asarray(values, dtype=float)
    if array.ndim != 2 or array.shape[1] != 3:
        raise ValueError(f"Expected an (n, 3) array, got shape {array.shape}")

    filters = [LowPassFilter(cutoff_hz) for _ in range(3)]
    filtered = np.zeros_like(array, dtype=float)
    for index, row in enumerate(array):
        dt = float(loop_dt[index]) if index < len(loop_dt) else 0.0
        filtered[index] = [axis_filter.update(component, dt) for axis_filter, component in zip(filters, row)]
    return filtered


def build_simulated_dataframe(
    df: pd.DataFrame,
    *,
    filter_gyro: bool = False,
    gyro_filter_window_size: int = 3,
    filter_linear_acceleration: bool = False,
    linear_acceleration_filter_window_size: int = 3,
    low_pass_linear_acceleration: bool = False,
    linear_acceleration_low_pass_cutoff_hz: float = 2.0,
) -> pd.DataFrame:
    """Create an enriched dataframe with reconstructed odometry columns."""
    output = df.copy()

    time_values = numeric_series(df, "time").interpolate(limit_direction="both").to_numpy(dtype=float)
    loop_dt = build_loop_dt(time_values)

    raw_body_acceleration = np.column_stack(
        [
            numeric_series(df, "acceleration_x").fillna(0.0).to_numpy(dtype=float),
            numeric_series(df, "acceleration_y").fillna(0.0).to_numpy(dtype=float),
            numeric_series(df, "acceleration_z").fillna(0.0).to_numpy(dtype=float),
        ]
    )
    body_acceleration = (
        apply_vector_moving_average(
            raw_body_acceleration,
            linear_acceleration_filter_window_size,
        )
        if filter_linear_acceleration
        else raw_body_acceleration.copy()
    )
    body_acceleration = (
        apply_vector_low_pass(
            body_acceleration,
            loop_dt,
            linear_acceleration_low_pass_cutoff_hz,
        )
        if low_pass_linear_acceleration
        else body_acceleration
    )

    absolute_orientation = np.column_stack(
        [
            numeric_series(df, "roll").interpolate(limit_direction="both").to_numpy(dtype=float),
            numeric_series(df, "pitch").interpolate(limit_direction="both").to_numpy(dtype=float),
            numeric_series(df, "yaw").interpolate(limit_direction="both").to_numpy(dtype=float),
        ]
    )
    initial_orientation = absolute_orientation[0]
    relative_orientation = wrap_angle_degrees(absolute_orientation - initial_orientation)

    raw_world_acceleration, quaternions = rotate_body_acceleration_to_world(
        raw_body_acceleration,
        absolute_orientation,
    )
    world_acceleration, _ = rotate_body_acceleration_to_world(
        body_acceleration,
        absolute_orientation,
    )
    velocity, position = integrate_trapezoidal(world_acceleration, loop_dt)

    raw_angular_velocity = np.column_stack(
        [
            numeric_series(df, "omega_x").fillna(0.0).to_numpy(dtype=float) if "omega_x" in df.columns else np.zeros(len(df), dtype=float),
            numeric_series(df, "omega_y").fillna(0.0).to_numpy(dtype=float) if "omega_y" in df.columns else np.zeros(len(df), dtype=float),
            numeric_series(df, "omega_z").fillna(0.0).to_numpy(dtype=float) if "omega_z" in df.columns else np.zeros(len(df), dtype=float),
        ]
    )
    angular_velocity = (
        apply_vector_moving_average(
            raw_angular_velocity,
            gyro_filter_window_size,
        )
        if filter_gyro
        else raw_angular_velocity.copy()
    )

    output["runtime"] = time_values
    output["loop_dt"] = loop_dt
    output["receiver_angle"] = numeric_series(df, "steering angle") if "steering angle" in df.columns else np.nan

    output["imu_body_acceleration_raw_x"] = raw_body_acceleration[:, 0]
    output["imu_body_acceleration_raw_y"] = raw_body_acceleration[:, 1]
    output["imu_body_acceleration_raw_z"] = raw_body_acceleration[:, 2]
    output["imu_body_acceleration_filtered_x"] = body_acceleration[:, 0]
    output["imu_body_acceleration_filtered_y"] = body_acceleration[:, 1]
    output["imu_body_acceleration_filtered_z"] = body_acceleration[:, 2]
    output["imu_body_acceleration_x"] = body_acceleration[:, 0]
    output["imu_body_acceleration_y"] = body_acceleration[:, 1]
    output["imu_body_acceleration_z"] = body_acceleration[:, 2]

    output["imu_acceleration_raw_x"] = raw_world_acceleration[:, 0]
    output["imu_acceleration_raw_y"] = raw_world_acceleration[:, 1]
    output["imu_acceleration_raw_z"] = raw_world_acceleration[:, 2]
    output["imu_acceleration_x"] = world_acceleration[:, 0]
    output["imu_acceleration_y"] = world_acceleration[:, 1]
    output["imu_acceleration_z"] = world_acceleration[:, 2]

    output["imu_velocity_x"] = velocity[:, 0]
    output["imu_velocity_y"] = velocity[:, 1]
    output["imu_velocity_z"] = velocity[:, 2]

    output["imu_position_x"] = position[:, 0]
    output["imu_position_y"] = position[:, 1]
    output["imu_position_z"] = position[:, 2]

    output["imu_absolute_roll"] = absolute_orientation[:, 0]
    output["imu_absolute_pitch"] = absolute_orientation[:, 1]
    output["imu_absolute_yaw"] = absolute_orientation[:, 2]

    output["imu_relative_roll"] = relative_orientation[:, 0]
    output["imu_relative_pitch"] = relative_orientation[:, 1]
    output["imu_relative_yaw"] = relative_orientation[:, 2]

    output["imu_initial_roll"] = initial_orientation[0]
    output["imu_initial_pitch"] = initial_orientation[1]
    output["imu_initial_yaw"] = initial_orientation[2]

    output["imu_quaternion_w"] = quaternions[:, 0]
    output["imu_quaternion_x"] = quaternions[:, 1]
    output["imu_quaternion_y"] = quaternions[:, 2]
    output["imu_quaternion_z"] = quaternions[:, 3]

    output["imu_angular_velocity_raw_x"] = raw_angular_velocity[:, 0]
    output["imu_angular_velocity_raw_y"] = raw_angular_velocity[:, 1]
    output["imu_angular_velocity_raw_z"] = raw_angular_velocity[:, 2]
    output["imu_angular_velocity_filtered_x"] = angular_velocity[:, 0]
    output["imu_angular_velocity_filtered_y"] = angular_velocity[:, 1]
    output["imu_angular_velocity_filtered_z"] = angular_velocity[:, 2]
    output["imu_angular_velocity_x"] = angular_velocity[:, 0]
    output["imu_angular_velocity_y"] = angular_velocity[:, 1]
    output["imu_angular_velocity_z"] = angular_velocity[:, 2]

    output["imu_filter_gyro_enabled"] = bool(filter_gyro)
    output["imu_gyro_filter_window_size"] = int(gyro_filter_window_size)
    output["imu_filter_linear_acceleration_enabled"] = bool(filter_linear_acceleration)
    output["imu_linear_acceleration_filter_window_size"] = int(linear_acceleration_filter_window_size)
    output["imu_low_pass_linear_acceleration_enabled"] = bool(low_pass_linear_acceleration)
    output["imu_linear_acceleration_low_pass_cutoff_hz"] = float(linear_acceleration_low_pass_cutoff_hz)

    return output


def main() -> None:
    """Run the legacy integration pipeline."""
    args = parse_args()
    input_path = args.input.resolve()
    output_path = args.output.resolve()

    legacy_df = load_legacy_csv(input_path)
    simulated_df = build_simulated_dataframe(
        legacy_df,
        filter_gyro=args.filter_gyro,
        gyro_filter_window_size=args.gyro_filter_window_size,
        filter_linear_acceleration=args.filter_linear_acceleration,
        linear_acceleration_filter_window_size=args.linear_acceleration_filter_window_size,
        low_pass_linear_acceleration=args.low_pass_linear_acceleration,
        linear_acceleration_low_pass_cutoff_hz=args.linear_acceleration_low_pass_cutoff_hz,
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    simulated_df.to_csv(output_path, index=False)

    print(f"Wrote reconstructed odometry to {output_path}")
    print(f"Rows: {len(simulated_df)}")
    print(
        "Final position: "
        f"({simulated_df['imu_position_x'].iloc[-1]:.3f}, "
        f"{simulated_df['imu_position_y'].iloc[-1]:.3f}, "
        f"{simulated_df['imu_position_z'].iloc[-1]:.3f})"
    )


if __name__ == "__main__":
    main()
