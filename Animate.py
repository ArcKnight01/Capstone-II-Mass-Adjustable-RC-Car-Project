"""Animate recorded RC car telemetry from CSV logs in the data folder."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


DATA_DIR = Path(__file__).resolve().parent / "data"
DEFAULT_CSV_NAME = (
    "simulated_data.csv"
    if (DATA_DIR / "simulated_data.csv").exists()
    else "data.csv"
)
DEFAULT_INTERVAL_MS = 80
DEFAULT_TRAIL_LENGTH = 250
DEFAULT_STEERING_SIGN_CONVENTION = "negative-left"


def steering_sign_description(steering_sign_convention: str) -> str:
    """Return a human-readable summary of the steering sign convention."""
    if steering_sign_convention == "negative-left":
        return "negative=left, positive=right"
    if steering_sign_convention == "positive-left":
        return "positive=left, negative=right"
    raise ValueError(f"Unsupported steering sign convention: {steering_sign_convention}")


def steering_math_sign_multiplier(steering_sign_convention: str) -> float:
    """
    Map the logged steering sign convention into standard XY-plane math rotation.

    Standard math rotation for the body XY plane used here is:
    - positive angle rotates from +X toward +Y
    - with +X forward and +Y left, positive math rotation means left
    """
    if steering_sign_convention == "negative-left":
        return -1.0
    if steering_sign_convention == "positive-left":
        return 1.0
    raise ValueError(f"Unsupported steering sign convention: {steering_sign_convention}")


def parse_args() -> argparse.Namespace:
    """Parse command line options for the animation viewer."""
    parser = argparse.ArgumentParser(
        description="Animate RC car telemetry from a CSV file in the data folder."
    )
    parser.add_argument(
        "csv_file",
        nargs="?",
        type=Path,
        default=Path(DEFAULT_CSV_NAME),
        help=(
            "CSV file to animate. Bare filenames are resolved inside data/. "
            f"Default: {DEFAULT_CSV_NAME}"
        ),
    )
    parser.add_argument(
        "--interval",
        type=int,
        default=DEFAULT_INTERVAL_MS,
        help=f"Delay between animation frames in milliseconds. Default: {DEFAULT_INTERVAL_MS}.",
    )
    parser.add_argument(
        "--step",
        type=int,
        default=1,
        help="Number of samples to skip between frames. Default: 1.",
    )
    parser.add_argument(
        "--trail",
        type=int,
        default=DEFAULT_TRAIL_LENGTH,
        help=f"Number of prior samples to keep visible in the trail. Default: {DEFAULT_TRAIL_LENGTH}.",
    )
    parser.add_argument(
        "--save",
        type=Path,
        help="Optional output path for a saved animation, for example output.gif or output.mp4.",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=15,
        help="Frames per second when saving an animation. Default: 15.",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Build the figure without opening an interactive window.",
    )
    parser.add_argument(
        "--steering-sign",
        choices=["negative-left", "positive-left"],
        default=DEFAULT_STEERING_SIGN_CONVENTION,
        help=(
            "Interpret logged steering angles using this sign convention. "
            f"Default: {DEFAULT_STEERING_SIGN_CONVENTION}."
        ),
    )
    return parser.parse_args()


def resolve_csv_path(csv_arg: Path) -> Path:
    """Resolve a CSV argument, defaulting bare filenames into the data directory."""
    if csv_arg.is_absolute():
        return csv_arg
    if csv_arg.parent == Path("."):
        return (DATA_DIR / csv_arg).resolve()
    return csv_arg.resolve()


def load_dataframe(csv_path: Path) -> pd.DataFrame:
    """Load a CSV file and validate that it contains at least one data row."""
    df = pd.read_csv(csv_path)
    if df.empty:
        raise ValueError(f"{csv_path} contains only headers and no data rows.")
    return df


def pick_numeric_series(
    df: pd.DataFrame, candidates: list[str]
) -> tuple[pd.Series, str | None]:
    """Return the first matching numeric series from a list of candidate columns."""
    for column in candidates:
        if column in df.columns:
            return pd.to_numeric(df[column], errors="coerce"), column
    return pd.Series(np.nan, index=df.index, dtype=float), None


def has_valid_data(series: pd.Series | np.ndarray) -> bool:
    """Check whether a numeric series contains at least one finite value."""
    values = np.asarray(series, dtype=float)
    return bool(np.isfinite(values).any())


def fill_numeric_series(series: pd.Series, fallback: float = 0.0) -> np.ndarray:
    """Interpolate missing numeric values and return a dense NumPy array."""
    if not has_valid_data(series):
        return np.full(len(series), fallback, dtype=float)

    filled = series.interpolate(limit_direction="both")
    filled = filled.bfill().ffill()
    return filled.to_numpy(dtype=float)


def fill_angle_series(
    series: pd.Series,
    fallback: float = 0.0,
    unwrap: bool = True,
) -> np.ndarray:
    """Interpolate angles in degrees and optionally unwrap them for continuity."""
    values = fill_numeric_series(series, fallback=fallback)
    if not np.isfinite(values).any():
        return values
    if not unwrap:
        return values
    return np.rad2deg(np.unwrap(np.deg2rad(values)))


def normalize_time(df: pd.DataFrame) -> tuple[np.ndarray, str]:
    """Build a monotonic time axis in seconds."""
    time_series, column_name = pick_numeric_series(df, ["runtime", "time", "Time"])
    if not has_valid_data(time_series):
        return np.arange(len(df), dtype=float), "sample"

    time_values = fill_numeric_series(time_series)
    time_values = time_values - time_values[0]

    if len(time_values) > 1:
        dt = np.diff(time_values)
        positive_dt = dt[dt > 0]
        replacement_dt = float(np.median(positive_dt)) if len(positive_dt) else 1.0
        for index in range(1, len(time_values)):
            if time_values[index] <= time_values[index - 1]:
                time_values[index] = time_values[index - 1] + replacement_dt

    return time_values, column_name or "sample"


def latlon_to_local_xy(lat: np.ndarray, lon: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Convert latitude and longitude arrays to local meter offsets."""
    lat0 = np.deg2rad(lat[0])
    meters_per_degree_lat = 111_320.0
    meters_per_degree_lon = 111_320.0 * np.cos(lat0)

    x = (lon - lon[0]) * meters_per_degree_lon
    y = (lat - lat[0]) * meters_per_degree_lat
    return x, y


def derive_trajectory(
    df: pd.DataFrame, time_s: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray, str]:
    """Create 3D trajectory coordinates using the best available telemetry columns."""
    x_series, x_name = pick_numeric_series(df, ["imu_position_x", "PositionX", "X", "x"])
    y_series, y_name = pick_numeric_series(df, ["imu_position_y", "PositionY", "Y", "y"])
    z_series, z_name = pick_numeric_series(df, ["imu_position_z", "PositionZ", "Z", "z"])
    if has_valid_data(x_series) and has_valid_data(y_series):
        z_values = fill_numeric_series(z_series) if has_valid_data(z_series) else np.zeros(len(df), dtype=float)
        return (
            fill_numeric_series(x_series),
            fill_numeric_series(y_series),
            z_values,
            f"{x_name}/{y_name}/{z_name or 'zero z'}",
        )

    lat_series, lat_name = pick_numeric_series(df, ["lat", "latitude", "Latitude"])
    lon_series, lon_name = pick_numeric_series(df, ["lon", "longitude", "Longitude"])
    if has_valid_data(lat_series) and has_valid_data(lon_series):
        x, y = latlon_to_local_xy(
            fill_numeric_series(lat_series),
            fill_numeric_series(lon_series),
        )
        z_values = fill_numeric_series(z_series) if has_valid_data(z_series) else np.zeros(len(df), dtype=float)
        return x, y, z_values, f"{lat_name}/{lon_name}/{z_name or 'zero z'}"

    vx_series, vx_name = pick_numeric_series(df, ["imu_velocity_x", "VelocityX", "vx"])
    vy_series, vy_name = pick_numeric_series(df, ["imu_velocity_y", "VelocityY", "vy"])
    vz_series, vz_name = pick_numeric_series(df, ["imu_velocity_z", "VelocityZ", "vz"])
    if has_valid_data(vx_series) or has_valid_data(vy_series) or has_valid_data(vz_series):
        vx = fill_numeric_series(vx_series)
        vy = fill_numeric_series(vy_series)
        vz = fill_numeric_series(vz_series)
        dt = np.diff(time_s, prepend=time_s[0])
        positive_dt = dt[dt > 0]
        replacement_dt = float(np.median(positive_dt)) if len(positive_dt) else 0.1
        dt = np.where(dt > 0, dt, replacement_dt)
        dt[0] = 0.0
        x = np.cumsum(vx * dt)
        y = np.cumsum(vy * dt)
        z = np.cumsum(vz * dt)
        source_names = [name for name in [vx_name, vy_name, vz_name] if name]
        return x, y, z, f"integrated {'/'.join(source_names)}"

    x = np.zeros(len(df), dtype=float)
    y = np.zeros(len(df), dtype=float)
    z = np.zeros(len(df), dtype=float)
    return x, y, z, "no trajectory logged"


def compute_heading_from_path(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Estimate heading angles from successive path samples."""
    if len(x) < 2:
        return np.zeros(len(x), dtype=float)

    dx = np.gradient(x)
    dy = np.gradient(y)
    raw_heading = np.arctan2(dy, dx)
    return np.degrees(np.unwrap(raw_heading))


def wrap_angle(angle_deg: float) -> float:
    """Wrap an angle to the range [0, 360)."""
    return float(angle_deg % 360.0)


def derive_heading(df: pd.DataFrame, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, str]:
    """Choose the best heading source for display."""
    heading_series, heading_name = pick_numeric_series(
        df,
        [
            "imu_absolute_yaw",
            "yaw",
            "Yaw",
            "imu_relative_yaw",
            "EulerZ",
            "heading",
        ],
    )
    if has_valid_data(heading_series):
        return fill_angle_series(heading_series), heading_name or "heading"
    return compute_heading_from_path(x, y), "path heading"


def derive_speed(
    df: pd.DataFrame,
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    time_s: np.ndarray,
) -> tuple[np.ndarray, str]:
    """Compute speed magnitude from velocity columns or path deltas."""
    vx_series, vx_name = pick_numeric_series(df, ["imu_velocity_x", "VelocityX", "vx"])
    vy_series, vy_name = pick_numeric_series(df, ["imu_velocity_y", "VelocityY", "vy"])
    vz_series, vz_name = pick_numeric_series(df, ["imu_velocity_z", "VelocityZ", "vz"])
    if has_valid_data(vx_series) or has_valid_data(vy_series) or has_valid_data(vz_series):
        vx = fill_numeric_series(vx_series)
        vy = fill_numeric_series(vy_series)
        vz = fill_numeric_series(vz_series)
        speed = np.sqrt(vx**2 + vy**2 + vz**2)
        source_names = [name for name in [vx_name, vy_name, vz_name] if name]
        return speed, " / ".join(source_names)

    dx = np.diff(x, prepend=x[0])
    dy = np.diff(y, prepend=y[0])
    dz = np.diff(z, prepend=z[0])
    dt = np.diff(time_s, prepend=time_s[0])
    positive_dt = dt[dt > 0]
    replacement_dt = float(np.median(positive_dt)) if len(positive_dt) else 1.0
    dt = np.where(dt > 0, dt, replacement_dt)
    dt[0] = 1.0
    speed = np.sqrt(dx**2 + dy**2 + dz**2) / dt
    speed[0] = 0.0
    return speed, "trajectory delta"


def derive_acceleration(df: pd.DataFrame) -> tuple[np.ndarray, str]:
    """Compute acceleration magnitude from the best available acceleration columns."""
    ax_series, ax_name = pick_numeric_series(
        df,
        [
            "imu_acceleration_x",
            "acceleration_x",
            "imu_body_acceleration_x",
            "AccelerationX",
            "ax",
        ],
    )
    ay_series, ay_name = pick_numeric_series(
        df,
        [
            "imu_acceleration_y",
            "acceleration_y",
            "imu_body_acceleration_y",
            "AccelerationY",
            "ay",
        ],
    )
    az_series, az_name = pick_numeric_series(
        df,
        [
            "imu_acceleration_z",
            "acceleration_z",
            "imu_body_acceleration_z",
            "AccelerationZ",
            "az",
        ],
    )
    if has_valid_data(ax_series) or has_valid_data(ay_series) or has_valid_data(az_series):
        ax = fill_numeric_series(ax_series)
        ay = fill_numeric_series(ay_series)
        az = fill_numeric_series(az_series)
        accel = np.sqrt(ax**2 + ay**2 + az**2)
        source_names = [name for name in [ax_name, ay_name, az_name] if name]
        return accel, " / ".join(source_names)

    return np.zeros(len(df), dtype=float), "not available"


def derive_acceleration_components(
    df: pd.DataFrame,
) -> tuple[dict[str, np.ndarray], dict[str, str | None]]:
    """Return acceleration components for plotting detailed linear-acceleration signals."""
    raw_ax_series, raw_ax_name = pick_numeric_series(
        df,
        ["imu_body_acceleration_raw_x"],
    )
    raw_ay_series, raw_ay_name = pick_numeric_series(
        df,
        ["imu_body_acceleration_raw_y"],
    )
    raw_az_series, raw_az_name = pick_numeric_series(
        df,
        ["imu_body_acceleration_raw_z"],
    )
    filtered_ax_series, filtered_ax_name = pick_numeric_series(
        df,
        ["imu_body_acceleration_filtered_x", "imu_body_acceleration_x"],
    )
    filtered_ay_series, filtered_ay_name = pick_numeric_series(
        df,
        ["imu_body_acceleration_filtered_y", "imu_body_acceleration_y"],
    )
    filtered_az_series, filtered_az_name = pick_numeric_series(
        df,
        ["imu_body_acceleration_filtered_z", "imu_body_acceleration_z"],
    )

    if (
        has_valid_data(raw_ax_series)
        or has_valid_data(raw_ay_series)
        or has_valid_data(raw_az_series)
    ) and (
        has_valid_data(filtered_ax_series)
        or has_valid_data(filtered_ay_series)
        or has_valid_data(filtered_az_series)
    ):
        raw_ax = fill_numeric_series(raw_ax_series, fallback=np.nan)
        raw_ay = fill_numeric_series(raw_ay_series, fallback=np.nan)
        raw_az = fill_numeric_series(raw_az_series, fallback=np.nan)
        filtered_ax = fill_numeric_series(filtered_ax_series, fallback=np.nan)
        filtered_ay = fill_numeric_series(filtered_ay_series, fallback=np.nan)
        filtered_az = fill_numeric_series(filtered_az_series, fallback=np.nan)

        return (
            {
                "raw_ax": raw_ax,
                "raw_ay": raw_ay,
                "raw_az": raw_az,
                "filtered_ax": filtered_ax,
                "filtered_ay": filtered_ay,
                "filtered_az": filtered_az,
            },
            {
                "raw_ax": raw_ax_name,
                "raw_ay": raw_ay_name,
                "raw_az": raw_az_name,
                "filtered_ax": filtered_ax_name,
                "filtered_ay": filtered_ay_name,
                "filtered_az": filtered_az_name,
            },
        )

    ax_series, ax_name = pick_numeric_series(
        df,
        [
            "imu_acceleration_x",
            "acceleration_x",
            "imu_body_acceleration_x",
            "AccelerationX",
            "ax",
        ],
    )
    ay_series, ay_name = pick_numeric_series(
        df,
        [
            "imu_acceleration_y",
            "acceleration_y",
            "imu_body_acceleration_y",
            "AccelerationY",
            "ay",
        ],
    )
    az_series, az_name = pick_numeric_series(
        df,
        [
            "imu_acceleration_z",
            "acceleration_z",
            "imu_body_acceleration_z",
            "AccelerationZ",
            "az",
        ],
    )

    ax_values = fill_numeric_series(ax_series, fallback=np.nan)
    ay_values = fill_numeric_series(ay_series, fallback=np.nan)
    az_values = fill_numeric_series(az_series, fallback=np.nan)
    magnitude = np.sqrt(
        np.nan_to_num(ax_values, nan=0.0) ** 2
        + np.nan_to_num(ay_values, nan=0.0) ** 2
        + np.nan_to_num(az_values, nan=0.0) ** 2
    )

    return (
        {
            "ax": ax_values,
            "ay": ay_values,
            "az": az_values,
            "|a|": magnitude,
        },
        {
            "ax": ax_name,
            "ay": ay_name,
            "az": az_name,
            "|a|": "magnitude",
        },
    )


def derive_absolute_orientation_components(
    df: pd.DataFrame,
    fallback_heading: np.ndarray,
) -> tuple[dict[str, np.ndarray], dict[str, str | None]]:
    """Return absolute roll, pitch, and yaw signals for the orientation panel."""
    roll_series, roll_name = pick_numeric_series(
        df,
        ["imu_absolute_roll", "roll", "Roll", "imu_relative_roll"],
    )
    pitch_series, pitch_name = pick_numeric_series(
        df,
        ["imu_absolute_pitch", "pitch", "Pitch", "imu_relative_pitch"],
    )
    yaw_series, yaw_name = pick_numeric_series(
        df,
        ["imu_absolute_yaw", "yaw", "Yaw", "EulerZ", "imu_relative_yaw", "heading"],
    )

    roll_values = fill_angle_series(roll_series, fallback=np.nan)
    pitch_values = fill_angle_series(pitch_series, fallback=np.nan)
    if has_valid_data(yaw_series):
        yaw_values = fill_angle_series(yaw_series, fallback=np.nan)
        yaw_source = yaw_name
    else:
        yaw_values = np.asarray(fallback_heading, dtype=float)
        yaw_source = "path heading"

    return (
        {
            "roll": roll_values,
            "pitch": pitch_values,
            "yaw": yaw_values,
        },
        {
            "roll": roll_name,
            "pitch": pitch_name,
            "yaw": yaw_source,
        },
    )


def build_signal_series(df: pd.DataFrame) -> dict[str, tuple[np.ndarray, str | None]]:
    """Collect optional one-dimensional telemetry series used by the info panel."""
    receiver_angle, receiver_name = pick_numeric_series(
        df, ["receiver_angle", "steering angle", "steering_angle", "angle"]
    )
    battery_percent, battery_name = pick_numeric_series(df, ["battery_percent"])
    roll, roll_name = pick_numeric_series(
        df,
        ["imu_absolute_roll", "roll", "Roll", "imu_relative_roll"],
    )
    pitch, pitch_name = pick_numeric_series(
        df,
        ["imu_absolute_pitch", "pitch", "Pitch", "imu_relative_pitch"],
    )
    lat, lat_name = pick_numeric_series(df, ["lat", "latitude", "Latitude"])
    lon, lon_name = pick_numeric_series(df, ["lon", "longitude", "Longitude"])

    return {
        "receiver_angle": (
            fill_numeric_series(receiver_angle, fallback=np.nan),
            receiver_name,
        ),
        "battery_percent": (
            fill_numeric_series(battery_percent, fallback=np.nan),
            battery_name,
        ),
        "roll": (fill_angle_series(roll, fallback=np.nan), roll_name),
        "pitch": (fill_angle_series(pitch, fallback=np.nan), pitch_name),
        "lat": (fill_numeric_series(lat, fallback=np.nan), lat_name),
        "lon": (fill_numeric_series(lon, fallback=np.nan), lon_name),
    }


def compute_axis_limits(values: np.ndarray) -> tuple[float, float]:
    """Build readable axis limits even when the signal is flat."""
    min_value = float(np.nanmin(values))
    max_value = float(np.nanmax(values))
    span = max_value - min_value
    if span <= 1e-9:
        padding = max(1.0, abs(max_value) * 0.1 + 0.5)
        return min_value - padding, max_value + padding
    padding = span * 0.1
    return min_value - padding, max_value + padding


def is_effectively_constant(values: np.ndarray, tolerance: float = 1e-9) -> bool:
    """Return True when a signal has no meaningful variation."""
    finite_values = np.asarray(values, dtype=float)
    finite_values = finite_values[np.isfinite(finite_values)]
    if len(finite_values) == 0:
        return True
    return bool(np.ptp(finite_values) <= tolerance)


def setup_track_axis(
    ax: plt.Axes,
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    trajectory_source: str,
    stationary_trajectory: bool,
) -> None:
    """Configure the 3D track plot."""
    ax.plot(x, y, z, color="#c9d2d8", linewidth=1.5, linestyle="--", label="Full path")
    ax.set_title("RC Car Track (3D)")
    ax.set_xlabel("Track X (m)")
    ax.set_ylabel("Track Y (m)")
    ax.set_zlabel("Track Z (m)")
    ax.grid(True, alpha=0.3)

    x_min, x_max = compute_axis_limits(x)
    y_min, y_max = compute_axis_limits(y)
    z_min, z_max = compute_axis_limits(z)

    x_mid = float(np.mean([x_min, x_max]))
    y_mid = float(np.mean([y_min, y_max]))
    z_mid = float(np.mean([z_min, z_max]))
    radius = max(x_max - x_min, y_max - y_min, z_max - z_min) / 2.0
    radius = max(radius, 1.0)

    ax.set_xlim(x_mid - radius, x_mid + radius)
    ax.set_ylim(y_mid - radius, y_mid + radius)
    ax.set_zlim(z_mid - radius, z_mid + radius)
    ax.set_box_aspect((1.0, 1.0, 0.75))
    ax.view_init(elev=24, azim=-58)
    ax.text2D(
        0.02,
        0.02,
        f"Trajectory source: {trajectory_source}",
        transform=ax.transAxes,
        fontsize=9,
        color="#495057",
        bbox={"facecolor": "white", "alpha": 0.9, "edgecolor": "#d0d7de"},
    )
    if stationary_trajectory:
        ax.text2D(
            0.02,
            0.10,
            "No changing position was logged.\nShowing in-place orientation only.",
            transform=ax.transAxes,
            fontsize=9,
            color="#7c2d12",
            bbox={"facecolor": "#fff7ed", "alpha": 0.95, "edgecolor": "#fdba74"},
        )


def setup_hud_axis(ax: plt.Axes, csv_path: Path, time_s: np.ndarray, frame_count: int) -> plt.Text:
    """Create the static HUD container and return the dynamic text artist."""
    ax.set_axis_off()
    ax.set_title("Telemetry", fontsize=12.5, pad=2)
    summary = [
        f"File: {csv_path.name}",
        f"Samples: {frame_count} | Duration: {time_s[-1]:.2f} s",
    ]
    ax.text(
        0.02,
        0.98,
        "\n".join(summary),
        transform=ax.transAxes,
        va="top",
        fontsize=10.0,
        color="#1f2933",
        linespacing=1.15,
        clip_on=True,
        bbox={"facecolor": "#f8fafc", "edgecolor": "#d0d7de", "boxstyle": "round,pad=0.5"},
    )
    return ax.text(
        0.02,
        0.64,
        "",
        transform=ax.transAxes,
        va="top",
        fontsize=8.9,
        family="monospace",
        color="#111827",
        linespacing=1.18,
        clip_on=True,
        bbox={"facecolor": "white", "edgecolor": "#d0d7de", "boxstyle": "round,pad=0.5"},
    )


def render_unavailable_axis(ax: plt.Axes, title: str, message: str = "Not available") -> None:
    """Render a readable placeholder instead of collapsing a missing telemetry panel."""
    ax.clear()
    ax.set_title(title)
    ax.set_facecolor("#f8fafc")
    ax.grid(False)
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_color("#d0d7de")
    ax.text(
        0.5,
        0.5,
        message,
        ha="center",
        va="center",
        fontsize=10,
        color="#6b7280",
        transform=ax.transAxes,
    )


def maybe_hide_axis(ax: plt.Axes, values: np.ndarray, label: str, color: str) -> tuple[plt.Line2D | None, plt.Line2D | None]:
    """Plot a signal if it contains useful variation and return its artists."""
    if len(values) == 0 or not np.isfinite(values).any():
        render_unavailable_axis(ax, label, "No logged samples")
        return None, None

    line, = ax.plot([], [], color=color, linewidth=2.4)
    cursor = ax.axvline(0.0, color="#111827", linewidth=1.25, alpha=0.7)
    ax.set_title(label)
    ax.set_facecolor("#fbfdff")
    ax.grid(True, alpha=0.3)
    return line, cursor


def maybe_hide_multiline_axis(
    ax: plt.Axes,
    series_map: dict[str, np.ndarray],
    styles: dict[str, tuple[str, str, float]],
    title: str,
    ylabel: str,
) -> tuple[dict[str, plt.Line2D], plt.Line2D | None]:
    """Create a multi-line telemetry panel when at least one series is available."""
    valid_series = {
        name: values
        for name, values in series_map.items()
        if len(values) > 0 and np.isfinite(values).any()
    }
    if not valid_series:
        render_unavailable_axis(ax, title, "No logged samples")
        return {}, None

    lines: dict[str, plt.Line2D] = {}
    for name, values in valid_series.items():
        color, linestyle, linewidth = styles.get(name, ("#111827", "-", 2.0))
        line, = ax.plot([], [], color=color, linestyle=linestyle, linewidth=linewidth, label=name)
        lines[name] = line

    cursor = ax.axvline(0.0, color="#111827", linewidth=1.1, alpha=0.6)
    stacked_values = np.concatenate([values[np.isfinite(values)] for values in valid_series.values()])
    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.set_facecolor("#fbfdff")
    ax.set_ylim(*compute_axis_limits(stacked_values))
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8.5, ncol=min(2, len(lines)))
    return lines, cursor


def build_car_outline(
    current_x: float,
    current_y: float,
    current_z: float,
    heading_deg: float,
    car_scale: float,
) -> np.ndarray:
    """Build a simple yaw-oriented 3D car outline centered on the current sample."""
    template = np.array(
        [
            [1.0, 0.0, 0.0],
            [-0.7, 0.55, 0.0],
            [-0.35, 0.0, 0.0],
            [-0.7, -0.55, 0.0],
            [1.0, 0.0, 0.0],
        ],
        dtype=float,
    ) * car_scale

    heading_rad = np.deg2rad(heading_deg)
    rotation = np.array(
        [
            [np.cos(heading_rad), -np.sin(heading_rad), 0.0],
            [np.sin(heading_rad), np.cos(heading_rad), 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    rotated = template @ rotation.T
    rotated[:, 0] += current_x
    rotated[:, 1] += current_y
    rotated[:, 2] += current_z
    return rotated


def rotation_matrix_from_euler(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """Build a world-frame rotation matrix from roll, pitch, and yaw in degrees."""
    roll = np.deg2rad(roll_deg)
    pitch = np.deg2rad(pitch_deg)
    yaw = np.deg2rad(yaw_deg)

    rotation_x = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, np.cos(roll), -np.sin(roll)],
            [0.0, np.sin(roll), np.cos(roll)],
        ],
        dtype=float,
    )
    rotation_y = np.array(
        [
            [np.cos(pitch), 0.0, np.sin(pitch)],
            [0.0, 1.0, 0.0],
            [-np.sin(pitch), 0.0, np.cos(pitch)],
        ],
        dtype=float,
    )
    rotation_z = np.array(
        [
            [np.cos(yaw), -np.sin(yaw), 0.0],
            [np.sin(yaw), np.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    return rotation_z @ rotation_y @ rotation_x


def build_orientation_gizmo(
    current_x: float,
    current_y: float,
    current_z: float,
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
    gizmo_scale: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return RGB body-axis line segments for the current orientation."""
    origin = np.array([current_x, current_y, current_z], dtype=float)
    rotation = rotation_matrix_from_euler(roll_deg, pitch_deg, yaw_deg)
    x_axis = np.column_stack((origin, origin + rotation[:, 0] * gizmo_scale))
    y_axis = np.column_stack((origin, origin + rotation[:, 1] * gizmo_scale))
    z_axis = np.column_stack((origin, origin + rotation[:, 2] * gizmo_scale))
    return x_axis, y_axis, z_axis


def build_steering_vector_xy(
    current_x: float,
    current_y: float,
    current_z: float,
    heading_deg: float,
    steering_deg: float,
    vector_scale: float,
    steering_sign_convention: str,
) -> np.ndarray:
    """
    Return a steering unit vector drawn in the global XY plane.

    Body-frame convention used here:
    - +X is forward
    - +Y is left
    - +Z is up

    The steering sign convention is configurable because the project mapping may
    still change. The logged steering angle is converted into standard math
    rotation before it is projected into the XY plane.
    """
    origin = np.array([current_x, current_y, current_z], dtype=float)
    steering_math_deg = steering_math_sign_multiplier(steering_sign_convention) * steering_deg
    steer_heading_rad = np.deg2rad(heading_deg + steering_math_deg)
    direction = np.array(
        [
            np.cos(steer_heading_rad),
            np.sin(steer_heading_rad),
            0.0,
        ],
        dtype=float,
    )
    direction_norm = np.linalg.norm(direction)
    if direction_norm > 0.0:
        direction = direction / direction_norm
    tip = origin + direction * vector_scale
    return np.column_stack((origin, tip))


def format_optional_value(value: float, suffix: str = "", digits: int = 2) -> str:
    """Format a numeric value or return n/a if it is not finite."""
    if not np.isfinite(value):
        return "n/a"
    return f"{value:.{digits}f}{suffix}"


def format_vector(
    values: tuple[float, ...] | list[float] | np.ndarray,
    suffix: str = "",
    digits: int = 2,
) -> str:
    """Format a compact vector tuple for the HUD."""
    formatted_components = [
        format_optional_value(float(value), digits=digits)
        for value in np.asarray(values, dtype=float)
    ]
    return f"({', '.join(formatted_components)}){suffix}"


def collapse_component_source(source: str) -> str:
    """Collapse common three-axis source patterns into a shorter label."""
    normalized = str(source).replace(" / ", "/")
    parts = [part.strip() for part in normalized.split("/") if part.strip()]
    if len(parts) != 3:
        return normalized

    split_parts: list[tuple[str, str]] = []
    for part in parts:
        if "_" not in part:
            return normalized
        prefix, suffix = part.rsplit("_", 1)
        split_parts.append((prefix, suffix))

    prefixes = {prefix for prefix, _ in split_parts}
    suffixes = [suffix for _, suffix in split_parts]
    if len(prefixes) != 1:
        return normalized

    prefix = split_parts[0][0]
    if suffixes == ["x", "y", "z"]:
        return f"{prefix}_{{x,y,z}}"
    if suffixes == ["roll", "pitch", "yaw"]:
        return f"{prefix}_{{roll,pitch,yaw}}"
    return normalized


def compact_source(source: str | None, max_length: int = 30) -> str:
    """Shorten verbose source labels so the HUD remains readable."""
    if not source:
        return "n/a"
    normalized = collapse_component_source(str(source))
    replacements = {
        "integrated ": "int ",
        "trajectory delta": "traj_delta",
        "path heading": "path_heading",
        "receiver_angle": "steering",
        "imu_position_": "pos_",
        "imu_velocity_": "vel_",
        "imu_acceleration_": "acc_",
        "imu_body_acceleration_": "body_acc_",
        "imu_absolute_": "abs_",
        "imu_relative_": "rel_",
        "battery_percent": "battery",
    }
    for old, new in replacements.items():
        normalized = normalized.replace(old, new)
    if len(normalized) <= max_length:
        return normalized
    return normalized[: max_length - 3] + "..."


def compact_source_group(
    sources: list[str | None] | tuple[str | None, ...],
    max_length: int = 26,
) -> str:
    """Shorten a grouped source description for HUD display."""
    combined = " / ".join(source for source in sources if source)
    return compact_source(combined, max_length=max_length)


def build_animation(
    csv_path: Path,
    interval_ms: int,
    frame_step: int,
    trail_length: int,
    steering_sign_convention: str,
) -> tuple[plt.Figure, FuncAnimation, dict[str, object]]:
    """Build the figure and animation objects."""
    df = load_dataframe(csv_path)
    time_s, time_source = normalize_time(df)
    x, y, z, trajectory_source = derive_trajectory(df, time_s)
    heading_deg, heading_source = derive_heading(df, x, y)
    speed, speed_source = derive_speed(df, x, y, z, time_s)
    acceleration_mag, acceleration_source = derive_acceleration(df)
    acceleration_components, acceleration_component_sources = derive_acceleration_components(df)
    signals = build_signal_series(df)
    roll_values, roll_source = signals["roll"]
    pitch_values, pitch_source = signals["pitch"]
    absolute_orientation, absolute_orientation_sources = derive_absolute_orientation_components(
        df,
        fallback_heading=heading_deg,
    )
    stationary_trajectory = (
        is_effectively_constant(x)
        and is_effectively_constant(y)
        and is_effectively_constant(z)
    )
    flat_speed = is_effectively_constant(speed)

    frame_indices = np.arange(0, len(df), max(1, frame_step), dtype=int)
    if frame_indices[-1] != len(df) - 1:
        frame_indices = np.append(frame_indices, len(df) - 1)

    plt.style.use("seaborn-v0_8-whitegrid")
    plt.rcParams.update(
        {
            "axes.titlesize": 11.5,
            "axes.labelsize": 10.5,
            "xtick.labelsize": 9.5,
            "ytick.labelsize": 9.5,
        }
    )
    fig = plt.figure(figsize=(16.2, 9.6), constrained_layout=False)
    grid = fig.add_gridspec(
        5,
        2,
        width_ratios=[3.1, 2.35],
        height_ratios=[1.82, 0.9, 0.95, 0.95, 1.0],
    )

    ax_track = fig.add_subplot(grid[:, 0], projection="3d")
    ax_hud = fig.add_subplot(grid[0, 1])
    ax_steer = fig.add_subplot(grid[1, 1])
    ax_motion = fig.add_subplot(grid[2, 1], sharex=ax_steer)
    ax_accel = fig.add_subplot(grid[3, 1], sharex=ax_steer)
    ax_orientation = fig.add_subplot(grid[4, 1], sharex=ax_steer)
    for axis in [ax_steer, ax_motion, ax_accel]:
        axis.tick_params(labelbottom=False)

    fig.suptitle(
        (
            f"RC Telemetry Dashboard: {csv_path.name}"
            f" ({steering_sign_description(steering_sign_convention)})"
        ),
        fontsize=16,
        fontweight="bold",
        color="#111827",
    )
    fig.subplots_adjust(
        left=0.045,
        right=0.975,
        top=0.93,
        bottom=0.065,
        wspace=0.18,
        hspace=0.28,
    )

    setup_track_axis(ax_track, x, y, z, trajectory_source, stationary_trajectory)
    hud_text = setup_hud_axis(ax_hud, csv_path, time_s, len(df))

    trail_line, = ax_track.plot([], [], [], color="#ec4899", linewidth=2.5, label="Recent trail")
    current_point, = ax_track.plot([], [], [], "o", color="#0f172a", markersize=6, zorder=6)
    car_outline, = ax_track.plot([], [], [], color="#ef4444", linewidth=2.0, zorder=8)
    gizmo_x_line, = ax_track.plot([], [], [], color="#ef4444", linewidth=2.4, zorder=9)
    gizmo_y_line, = ax_track.plot([], [], [], color="#22c55e", linewidth=2.4, zorder=9)
    gizmo_z_line, = ax_track.plot([], [], [], color="#3b82f6", linewidth=2.4, zorder=9)
    steering_vector_line, = ax_track.plot([], [], [], color="#f97316", linewidth=2.8, zorder=10)

    path_extent = max(np.ptp(x), np.ptp(y), np.ptp(z), 1.0)
    car_scale = max(path_extent * 0.03, 0.55 if stationary_trajectory else 0.35)
    gizmo_scale = car_scale * 1.5
    steering_vector_scale = gizmo_scale
    track_legend_handles = [
        Line2D([0], [0], color="#c9d2d8", linewidth=1.5, linestyle="--", label="Full path"),
        Line2D([0], [0], color="#ec4899", linewidth=2.5, linestyle="-", label="Recent trail"),
        Line2D([0], [0], color="#ef4444", linewidth=2.4, linestyle="-", label="+X axis"),
        Line2D([0], [0], color="#22c55e", linewidth=2.4, linestyle="-", label="+Y axis"),
        Line2D([0], [0], color="#3b82f6", linewidth=2.4, linestyle="-", label="+Z axis"),
        Line2D(
            [0],
            [0],
            color="#f97316",
            linewidth=2.8,
            linestyle="-",
            label=f"Steering vector ({steering_sign_description(steering_sign_convention)})",
        ),
    ]
    ax_track.legend(
        handles=track_legend_handles,
        loc="upper right",
        fontsize=9,
        frameon=True,
    )

    steering_values, steering_name = signals["receiver_angle"]
    motion_values = heading_deg if stationary_trajectory and flat_speed else speed
    motion_label = (
        "Heading (deg)"
        if stationary_trajectory and flat_speed
        else "Speed (m/s)"
    )
    steer_label = (
        "Steering Angle (deg)"
        if steering_name
        else "Steering Angle (not logged, deg)"
    )

    steer_plot, steer_cursor = maybe_hide_axis(ax_steer, steering_values, steer_label, "#2563eb")
    motion_plot, motion_cursor = maybe_hide_axis(ax_motion, motion_values, motion_label, "#ef4444")

    if steer_plot is not None:
        ax_steer.set_xlim(time_s[0], time_s[-1] if time_s[-1] > 0 else max(1.0, len(time_s) - 1))
        ax_steer.set_ylim(*compute_axis_limits(steering_values))
        ax_steer.set_ylabel("deg")

    if motion_plot is not None:
        ax_motion.set_xlim(time_s[0], time_s[-1] if time_s[-1] > 0 else max(1.0, len(time_s) - 1))
        ax_motion.set_ylim(*compute_axis_limits(motion_values))
        ax_motion.set_ylabel("deg" if stationary_trajectory and flat_speed else "m/s")

    acceleration_styles = {
        "ax": ("#dc2626", "-", 2.0),
        "ay": ("#16a34a", "-", 2.0),
        "az": ("#2563eb", "-", 2.0),
        "|a|": ("#111827", "--", 2.4),
        "raw_ax": ("#fca5a5", "--", 1.4),
        "raw_ay": ("#86efac", "--", 1.4),
        "raw_az": ("#93c5fd", "--", 1.4),
        "filtered_ax": ("#dc2626", "-", 2.2),
        "filtered_ay": ("#16a34a", "-", 2.2),
        "filtered_az": ("#2563eb", "-", 2.2),
    }
    acceleration_lines, acceleration_cursor = maybe_hide_multiline_axis(
        ax_accel,
        acceleration_components,
        acceleration_styles,
        (
            "Linear Acceleration (raw vs filtered, m/s^2)"
            if "raw_ax" in acceleration_components and "filtered_ax" in acceleration_components
            else "Linear Acceleration (m/s^2)"
        ),
        "m/s^2",
    )
    if acceleration_lines:
        ax_accel.set_xlim(time_s[0], time_s[-1] if time_s[-1] > 0 else max(1.0, len(time_s) - 1))

    orientation_styles = {
        "roll": ("#f97316", "-", 2.1),
        "pitch": ("#7c3aed", "-", 2.1),
        "yaw": ("#0f766e", "-", 2.3),
    }
    orientation_lines, orientation_cursor = maybe_hide_multiline_axis(
        ax_orientation,
        absolute_orientation,
        orientation_styles,
        "Absolute Orientation (deg)",
        "deg",
    )
    if orientation_lines:
        ax_orientation.set_xlim(time_s[0], time_s[-1] if time_s[-1] > 0 else max(1.0, len(time_s) - 1))
        ax_orientation.set_xlabel(f"Time ({time_source})")

    battery_values, battery_name = signals["battery_percent"]
    battery_axis = None
    battery_cursor = None
    battery_plot = None
    if np.isfinite(battery_values).any():
        battery_axis = ax_motion.twinx()
        battery_plot, = battery_axis.plot(time_s, battery_values, color="#16a34a", linewidth=1.2, alpha=0.7)
        battery_cursor = battery_axis.axvline(0.0, color="#16a34a", linewidth=1.0, alpha=0.35)
        battery_axis.set_ylabel(battery_name or "battery %", color="#166534")
        battery_axis.tick_params(axis="y", colors="#166534")
        battery_axis.set_ylim(*compute_axis_limits(battery_values))

    def update(frame_number: int):
        sample_index = frame_indices[frame_number]
        trail_start = max(0, sample_index - max(1, trail_length))

        trail_line.set_data_3d(
            x[trail_start : sample_index + 1],
            y[trail_start : sample_index + 1],
            z[trail_start : sample_index + 1],
        )
        current_point.set_data_3d([x[sample_index]], [y[sample_index]], [z[sample_index]])

        heading = heading_deg[sample_index]
        roll = roll_values[sample_index] if np.isfinite(roll_values[sample_index]) else 0.0
        pitch = pitch_values[sample_index] if np.isfinite(pitch_values[sample_index]) else 0.0
        car_points = build_car_outline(
            x[sample_index],
            y[sample_index],
            z[sample_index],
            heading,
            car_scale,
        )
        car_outline.set_data_3d(car_points[:, 0], car_points[:, 1], car_points[:, 2])
        gizmo_x, gizmo_y, gizmo_z = build_orientation_gizmo(
            x[sample_index],
            y[sample_index],
            z[sample_index],
            roll,
            pitch,
            heading,
            gizmo_scale,
        )
        gizmo_x_line.set_data_3d(gizmo_x[0], gizmo_x[1], gizmo_x[2])
        gizmo_y_line.set_data_3d(gizmo_y[0], gizmo_y[1], gizmo_y[2])
        gizmo_z_line.set_data_3d(gizmo_z[0], gizmo_z[1], gizmo_z[2])
        steering_vector = build_steering_vector_xy(
            x[sample_index],
            y[sample_index],
            z[sample_index],
            heading,
            steering_values[sample_index] if np.isfinite(steering_values[sample_index]) else 0.0,
            steering_vector_scale,
            steering_sign_convention,
        )
        steering_vector_line.set_data_3d(
            steering_vector[0],
            steering_vector[1],
            steering_vector[2],
        )

        if steer_plot is not None:
            steer_plot.set_data(time_s[: sample_index + 1], steering_values[: sample_index + 1])
            steer_cursor.set_xdata([time_s[sample_index], time_s[sample_index]])

        if motion_plot is not None:
            motion_plot.set_data(time_s[: sample_index + 1], motion_values[: sample_index + 1])
            motion_cursor.set_xdata([time_s[sample_index], time_s[sample_index]])

        if acceleration_lines and acceleration_cursor is not None:
            for name, line in acceleration_lines.items():
                line.set_data(
                    time_s[: sample_index + 1],
                    acceleration_components[name][: sample_index + 1],
                )
            acceleration_cursor.set_xdata([time_s[sample_index], time_s[sample_index]])

        if orientation_lines and orientation_cursor is not None:
            for name, line in orientation_lines.items():
                line.set_data(
                    time_s[: sample_index + 1],
                    absolute_orientation[name][: sample_index + 1],
                )
            orientation_cursor.set_xdata([time_s[sample_index], time_s[sample_index]])

        if battery_axis is not None and battery_cursor is not None:
            battery_cursor.set_xdata([time_s[sample_index], time_s[sample_index]])

        lat_values, lat_source = signals["lat"]
        lon_values, lon_source = signals["lon"]
        position_text = format_vector(
            (x[sample_index], y[sample_index], z[sample_index]),
            suffix=" m",
            digits=2,
        )
        linear_accel_text = format_vector(
            (
                acceleration_components["filtered_ax"][sample_index]
                if "filtered_ax" in acceleration_components
                else acceleration_components["ax"][sample_index],
                acceleration_components["filtered_ay"][sample_index]
                if "filtered_ay" in acceleration_components
                else acceleration_components["ay"][sample_index],
                acceleration_components["filtered_az"][sample_index]
                if "filtered_az" in acceleration_components
                else acceleration_components["az"][sample_index],
            ),
            suffix=" m/s^2",
            digits=2,
        )
        orientation_text = (
            f"{format_optional_value(roll_values[sample_index], ' deg'):>9} | "
            f"{format_optional_value(pitch_values[sample_index], ' deg'):>9} | "
            f"{format_optional_value(wrap_angle(heading_deg[sample_index]), ' deg'):>9}"
        )
        orientation_source_text = compact_source_group(
            [roll_source, pitch_source, heading_source],
            max_length=24,
        )
        battery_text = format_optional_value(battery_values[sample_index], ' %')
        source_line_one = (
            f"src: trj={compact_source(trajectory_source, max_length=18)}  "
            f"acc={compact_source(acceleration_source, max_length=18)}"
        )
        source_line_two = (
            f"     st={compact_source(steering_name, max_length=14)}  "
            f"ori={orientation_source_text}"
        )
        source_line_three = (
            f"     st_sign={steering_sign_description(steering_sign_convention)}"
        )

        hud_lines = [
            f"time/mode:  {time_s[sample_index]:6.2f} s | {'in-place' if stationary_trajectory else 'moving'}",
            f"position:   {position_text}",
            f"heading/st: {format_optional_value(wrap_angle(heading_deg[sample_index]), ' deg'):>9} | {format_optional_value(steering_values[sample_index], ' deg'):>9}",
            f"speed/|a|:  {format_optional_value(speed[sample_index], ' m/s'):>9} | {format_optional_value(acceleration_mag[sample_index], ' m/s^2'):>9}",
            f"lin accel:  {linear_accel_text}",
            f"roll/p/y:   {orientation_text}",
        ]

        if np.isfinite(battery_values).any():
            hud_lines.append(f"battery:    {battery_text:>9}")

        hud_lines.extend([source_line_one, source_line_two, source_line_three])

        if lat_source and lon_source and (
            np.isfinite(lat_values[sample_index]) or np.isfinite(lon_values[sample_index])
        ):
            hud_lines.extend(
                [
                    "lat/lon:    "
                    f"({format_optional_value(lat_values[sample_index], digits=6)}, "
                    f"{format_optional_value(lon_values[sample_index], digits=6)})",
                ]
            )

        hud_text.set_text("\n".join(hud_lines))

        artists = [
            trail_line,
            current_point,
            car_outline,
            gizmo_x_line,
            gizmo_y_line,
            gizmo_z_line,
            steering_vector_line,
            hud_text,
        ]
        if steer_plot is not None:
            artists.extend([steer_plot, steer_cursor])
        if motion_plot is not None:
            artists.extend([motion_plot, motion_cursor])
        if acceleration_lines and acceleration_cursor is not None:
            artists.extend(list(acceleration_lines.values()))
            artists.append(acceleration_cursor)
        if orientation_lines and orientation_cursor is not None:
            artists.extend(list(orientation_lines.values()))
            artists.append(orientation_cursor)
        if battery_plot is not None and battery_cursor is not None:
            artists.extend([battery_plot, battery_cursor])
        return artists

    animation = FuncAnimation(
        fig,
        update,
        frames=len(frame_indices),
        interval=max(1, interval_ms),
        blit=False,
        repeat=True,
    )
    return fig, animation, {
        "stationary_trajectory": stationary_trajectory,
        "trajectory_source": trajectory_source,
        "heading_source": heading_source,
        "steering_sign_convention": steering_sign_convention,
    }


def main() -> None:
    """Entry point for the telemetry animation viewer."""
    args = parse_args()
    csv_path = resolve_csv_path(args.csv_file)

    if not csv_path.exists():
        raise FileNotFoundError(f"Telemetry file not found: {csv_path}")

    figure, animation, metadata = build_animation(
        csv_path=csv_path,
        interval_ms=args.interval,
        frame_step=args.step,
        trail_length=args.trail,
        steering_sign_convention=args.steering_sign,
    )

    print(f"Animating telemetry from {csv_path}")
    print(f"Steering sign convention: {metadata['steering_sign_convention']}")
    if metadata["stationary_trajectory"]:
        print(
            "No changing position or velocity was found in this log, so the car will "
            "rotate in place instead of translating across the track."
        )

    if args.save:
        animation.save(args.save, fps=args.fps)
        print(f"Saved animation to {args.save.resolve()}")

    if args.no_show:
        figure.canvas.draw()
        plt.close(figure)
        return

    plt.show()


if __name__ == "__main__":
    main()
