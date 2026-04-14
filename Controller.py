"""
Controller.py - Main runtime loop for the RC car data logging system.

This module orchestrates data collection from IMU, RC receiver, GPS, and battery,
logs comprehensive telemetry to CSV, and optionally runs the Extended Kalman Filter
for sensor fusion alongside the existing Odometry dead-reckoning pipeline.

Navigation Modes (selectable at construction time via ``navigation_mode``):
  NavigationMode.ODOMETRY_ONLY  -- IMU dead reckoning only.  Baseline, no EKF.
  NavigationMode.EKF_IMU_ONLY   -- EKF using IMU only.  Indoor-safe, no GPS required.
  NavigationMode.EKF_GPS_IMU    -- Full EKF fusing IMU + GPS.  Outdoor use.

Odometry always runs regardless of mode so both estimates appear in every CSV for
comparison.  EKF columns are empty strings when EKF is not active.

Original design intent (preserved):
  The primary goal of sensor fusion here is to trust the BNO055 NDOF orientation
  (roll, pitch, yaw) heavily — its 9-DOF NDOF fusion is far more reliable than
  double-integrating raw accelerometers — while using the IMU linear accelerations
  and GPS velocity (NOT GPS position) as the main dynamic inputs.  GPS position is
  included but given large noise (sigma ~2.5 m) so it only prevents long-term drift
  rather than dominating the estimate.  This complements Odometry rather than
  replacing it.

GPS notes:
  GPS will not always be available (indoor testing, startup, urban canyons).  The
  EKF handles this gracefully: GPS updates are simply skipped when no fix is injected.
  EKF_IMU_ONLY mode never attempts GPS updates at all.

Data flow:
  1. Odometry.update() → IMU accel/gyro/orientation → velocity/position (always)
  2. If EKF mode active: predict() → IMU updates → optional GPS updates
  3. build_log_row() writes both Odometry and EKF columns to CSV every loop
  4. GPS data is injected externally via inject_gps_data() when available

Usage:
    python Controller.py [output_filename] [verbose]

Example:
    python Controller.py test_run True
"""

from __future__ import annotations

import argparse
import math
import sys
import os
import time
import csv
import pathlib
from typing import Mapping

import numpy as np

try:
    robotSupported = os.uname().nodename in ('terminatorpi', 'robotpi', 'carpi')
except Exception:
    import platform
    robotSupported = platform.uname().node in ('terminatorpi', 'robotpi', 'carpi')

if robotSupported:
    import board
    import busio
    import adafruit_bno055

from Odometry import Odometry
from IMU import IMU
from RCReceiverNano import RCReceiver
from RobotClock import Clock
from Power import Battery
from NavigationMode import NavigationMode

try:
    from ExtendedKalmanFilter import ExtendedKalmanFilter
    from ConfigLoader import load_config_bundle
    _EKF_AVAILABLE = True
except ImportError:
    _EKF_AVAILABLE = False

# BNO055 Euler angles are in degrees (Adafruit library convention).
# The BNO055 NDOF heading is 0–360° clockwise from magnetic North.
# ENU yaw convention used by the EKF: 0 = East, positive = CCW.
_DEG_TO_RAD = math.pi / 180.0

# How often to update the BNO sensor data (in hertz).
BNO_UPDATE_FREQUENCY_HZ = 10

# ── Steering angle conversion ──────────────────────────────────────────────
# The RC receiver reports ``angle`` as an integer (degrees of servo travel).
# The true road-wheel angle vs servo angle relationship depends on the linkage
# geometry and has not yet been measured.  Until that mapping is characterised,
# a 1:1 degrees-to-radians conversion is used as a placeholder.
# TODO: replace with the measured servo-to-road-wheel transfer function.
_STEERING_DEG_TO_RAD = _DEG_TO_RAD


def _bno055_heading_to_enu_yaw_rad(heading_deg: float) -> float:
    """
    Convert BNO055 compass heading to EKF ENU yaw convention.

    BNO055: 0 = magnetic North, increases clockwise (compass convention).
    EKF ENU: 0 = East, increases counter-clockwise.

    The conversion is:
        yaw_enu = 90° - heading_compass    (mod 360°)

    then wrapped to [-pi, pi).
    """
    yaw_deg = (90.0 - heading_deg) % 360.0
    yaw_rad = math.radians(yaw_deg)
    return math.atan2(math.sin(yaw_rad), math.cos(yaw_rad))


class Controller:
    """Main controller loop: sensor polling, odometry, optional EKF, CSV logging."""

    # ── CSV column layout ──────────────────────────────────────────────────
    # Odometry / raw sensor columns come first (unchanged from original).
    # EKF and GPS columns are appended so existing CSV consumers are not broken.

    CSV_HEADERS_CORE = [
        'time',
        'runtime',
        'loop_dt',
        'nav_mode',
        'receiver_pulse_width',
        'receiver_angle',
        'receiver_sample_age_sec',
        'receiver_is_fresh',
        'battery_percent',
        'battery_seconds_left',
        'battery_time_remaining',
        'battery_plugged_in',
        'imu_temperature',
        'imu_raw_acceleration_x',
        'imu_raw_acceleration_y',
        'imu_raw_acceleration_z',
        'imu_body_acceleration_x',
        'imu_body_acceleration_y',
        'imu_body_acceleration_z',
        'imu_acceleration_x',
        'imu_acceleration_y',
        'imu_acceleration_z',
        'imu_velocity_x',
        'imu_velocity_y',
        'imu_velocity_z',
        'imu_position_x',
        'imu_position_y',
        'imu_position_z',
        'imu_gravity_x',
        'imu_gravity_y',
        'imu_gravity_z',
        'imu_absolute_roll',
        'imu_absolute_pitch',
        'imu_absolute_yaw',
        'imu_relative_roll',
        'imu_relative_pitch',
        'imu_relative_yaw',
        'imu_initial_roll',
        'imu_initial_pitch',
        'imu_initial_yaw',
        'imu_quaternion_w',
        'imu_quaternion_x',
        'imu_quaternion_y',
        'imu_quaternion_z',
        'imu_angular_velocity_x',
        'imu_angular_velocity_y',
        'imu_angular_velocity_z',
        'imu_magnetic_x',
        'imu_magnetic_y',
        'imu_magnetic_z',
    ]

    # GPS columns: populated from injected GPS data dict each loop.
    CSV_HEADERS_GPS = [
        'gps_lat',
        'gps_lon',
        'gps_has_fix',
        'gps_speed_mps',
        'gps_track_deg',
    ]

    # EKF state columns: populated when EKF is active, empty strings otherwise.
    # All positions are in the local ENU frame (metres from first GPS fix or origin).
    CSV_HEADERS_EKF = [
        'ekf_initialized',
        'ekf_east_m',
        'ekf_north_m',
        'ekf_yaw_rad',
        'ekf_roll_rad',
        'ekf_pitch_rad',
        'ekf_vx_body_mps',
        'ekf_vy_body_mps',
        'ekf_yaw_rate_radps',
        'ekf_bias_ax_mps2',
        'ekf_bias_ay_mps2',
        'ekf_bias_gz_radps',
        'ekf_pos_1sigma_m',
        'ekf_yaw_1sigma_rad',
        'ekf_vel_1sigma_mps',
    ]

    CSV_HEADERS = CSV_HEADERS_CORE + CSV_HEADERS_GPS + CSV_HEADERS_EKF

    def __init__(
        self,
        verbose: bool = True,
        enabled: bool = True,
        log_to_csv: bool = True,
        csv_data_dir_name: str = 'data',
        csv_data_filename: str = 'data',
        receiver: RCReceiver | None = None,
        odometry: Odometry | None = None,
        loop_delay: float = 1.0 / BNO_UPDATE_FREQUENCY_HZ,
        navigation_mode: str = NavigationMode.ODOMETRY_ONLY,
        ekf: 'ExtendedKalmanFilter | None' = None,
    ) -> None:
        """
        Initialise the controller and its subsystems.

        Parameters
        ----------
        verbose : bool
            Enable console output.
        enabled : bool
            Master enable flag.  When ``False``, hardware peripherals
            (RCReceiver) are not initialised and telemetry is not printed.
        log_to_csv : bool
            Write telemetry to CSV on each update.
        csv_data_dir_name : str
            Directory for CSV output.
        csv_data_filename : str
            Base filename for CSV output.
        receiver : RCReceiver or None
            RC steering receiver.  Created with defaults if ``None``.
        odometry : Odometry or None
            Odometry dead-reckoning instance.  Created with defaults if ``None``.
        loop_delay : float
            Target time between update() calls (seconds).
        navigation_mode : str
            One of ``NavigationMode.ODOMETRY_ONLY``, ``EKF_IMU_ONLY``,
            or ``EKF_GPS_IMU``.  Controls which estimation pipeline runs.
        ekf : ExtendedKalmanFilter or None
            Pre-built EKF instance.  When ``None`` and the mode requires an EKF,
            one is constructed from the default config bundle.
        """
        if not NavigationMode.is_valid(navigation_mode):
            raise ValueError(
                f"Unknown navigation_mode '{navigation_mode}'. "
                f"Choose from: {NavigationMode.ALL}"
            )
        self._navigation_mode = navigation_mode

        log_dir = './'
        self.csv_data_filename = csv_data_filename
        self.csv_data_dir = pathlib.Path(log_dir, csv_data_dir_name)
        self.csv_data_dir.mkdir(parents=True, exist_ok=True)
        self.init_csv(self.csv_data_filename)

        self.__battery = Battery()
        self.__clock = Clock()
        self.__tickTimer = Clock()
        self.__tickTimer.reset()
        self.__tickTimer.update()
        self.__delT = self.__tickTimer.get_time("run")
        self.__clock.reset()
        self.__clock.update()
        self.__runtime = self.__clock.get_time("run")
        self.__time = self.__clock.get_time("current")
        self.__loop_delay = loop_delay
        self.__enabled = enabled
        self.__verbose = verbose
        self.__log_to_csv = log_to_csv
        # Build odometry first (IMU calibration happens inside).
        self.odometry = odometry if odometry is not None else Odometry()

        # Start receiver after calibration so stale Nano data is discarded.
        # RCReceiver opens a serial port immediately; skip on non-robot platforms.
        if self.__enabled:
            self.receiver = receiver if receiver is not None else RCReceiver()
            self.receiver.reset()
        else:
            self.receiver = None

        # Reset timers after startup / calibration.
        self.__tickTimer.reset()
        self.__clock.reset()
        self.__clock.update()
        self.__runtime = self.__clock.get_time("run")
        self.__time = self.__clock.get_time("current")

        # ── EKF setup ──────────────────────────────────────────────────────
        self._ekf: ExtendedKalmanFilter | None = None
        self._ekf_initialized: bool = False
        # Latest GPS data dict.  Populated via inject_gps_data().
        self._gps_data: dict = {
            'has_2d_fix': False,
            'latitude': None,
            'longitude': None,
            'speed_m_s': None,
            'track_deg': None,
            'epx_m': None,
            'epy_m': None,
            'eps_m_s': None,
        }

        if NavigationMode.uses_ekf(navigation_mode):
            if not _EKF_AVAILABLE:
                print(
                    "WARNING: ExtendedKalmanFilter could not be imported. "
                    "Falling back to ODOMETRY_ONLY mode."
                )
                self._navigation_mode = NavigationMode.ODOMETRY_ONLY
            else:
                if ekf is not None:
                    self._ekf = ekf
                else:
                    try:
                        bundle = load_config_bundle()
                        self._ekf = ExtendedKalmanFilter.from_config_bundle(bundle)
                    except Exception as exc:
                        print(f"WARNING: EKF construction failed ({exc}). Falling back to ODOMETRY_ONLY.")
                        self._navigation_mode = NavigationMode.ODOMETRY_ONLY

        if self.__verbose:
            print(f"Initialized Controller | navigation_mode={self._navigation_mode}")
            print(f"  EKF available: {self._ekf is not None}")

    # ── Public API ─────────────────────────────────────────────────────────

    def inject_gps_data(self, gps_data: Mapping[str, object]) -> None:
        """
        Provide the latest GPS fix data for use by the EKF.

        Call this whenever your GPS reader has a new report.  The data will be
        consumed by the next update() call.  Missing fields default to None so
        partial updates (e.g. position-only) are safe.

        Parameters
        ----------
        gps_data : Mapping[str, object]
            Dict with any subset of:
              'has_2d_fix'  (bool)
              'latitude'    (float, decimal degrees)
              'longitude'   (float, decimal degrees)
              'speed_m_s'   (float, m/s ground speed)
              'track_deg'   (float, degrees, course over ground)
              'epx_m'       (float, east position error 1-sigma)
              'epy_m'       (float, north position error 1-sigma)
              'eps_m_s'     (float, speed error 1-sigma)
        """
        self._gps_data.update(gps_data)

    def run(self) -> None:
        """Run the update loop until KeyboardInterrupt."""
        try:
            while True:
                self.update()
        except KeyboardInterrupt:
            print("Controller stopped.")

    def update(self) -> None:
        """Update all subsystems and log one CSV row."""
        self.__battery.update()

        self.__tickTimer.update()
        self.__delT = self.__tickTimer.get_time("run")
        if self.__delT <= 0:
            self.__delT = self.__loop_delay
        self.__tickTimer.reset()

        self.__clock.update()
        self.__runtime = self.__clock.get_time("run")
        self.__time = self.__clock.get_time("current")

        # Read steering.
        _stub: dict = {'pulse_width': 0, 'angle': 0, 'sample_age_sec': '', 'is_fresh': False}
        receiver_data_dict: dict = (
            (self.receiver.get_data() or _stub) if self.__enabled and self.receiver is not None
            else _stub
        )
        steering_angle_deg = float(receiver_data_dict.get('angle', 0))

        # Odometry always runs — provides IMU data for EKF and CSV baseline.
        self.odometry.update(dt=self.__delT)
        odometry_data: dict = self.odometry.get_data()

        # EKF step (only when an EKF mode is active).
        if self._ekf is not None:
            self._run_ekf_step(odometry_data, steering_angle_deg, self.__delT)

        row = self.build_log_row(receiver_data_dict, odometry_data)

        if self.__log_to_csv:
            self.add_to_csv(self.csv_data_filename, row)

        if self.__verbose:
            print(" | ".join(
                f"{h}={v}" for h, v in zip(self.CSV_HEADERS, row)
            ))

        time.sleep(self.__loop_delay)

    # ── EKF internals ──────────────────────────────────────────────────────

    def _run_ekf_step(
        self,
        odometry_data: dict,
        steering_angle_deg: float,
        dt: float,
    ) -> None:
        """
        Run one EKF predict + update cycle.

        Data sources (all from Odometry which has already read and filtered the
        IMU this loop):

          body_acceleration[0]  → longitudinal linear accel (m/s²), bias-uncorrected
          body_acceleration[1]  → lateral linear accel (m/s²)
          angular_velocity[2]   → gyro z / yaw rate (rad/s)
          absolute_orientation  → (roll, pitch, yaw) in DEGREES from BNO055 NDOF

        The BNO055 NDOF fusion is trusted heavily: the R matrices for orientation
        updates are small (σ ≈ 0.026 rad) so the Kalman gain pulls the orientation
        states close to the sensor reading every loop.  This was the original design
        intent — fuse high-quality IMU orientation with lower-quality GPS velocity.
        """
        if self._ekf is None:
            return

        # ── Extract IMU data ───────────────────────────────────────────────
        body_accel = odometry_data.get('body_acceleration', (0.0, 0.0, 0.0))
        angular_velocity = odometry_data.get('angular_velocity', (0.0, 0.0, 0.0))
        abs_orientation_deg = odometry_data.get('absolute_orientation', (0.0, 0.0, 0.0))

        accel_x = float(body_accel[0])          # longitudinal, m/s²
        accel_y = float(body_accel[1])          # lateral, m/s²
        gyro_z_radps = float(angular_velocity[2])  # yaw rate, rad/s

        # BNO055 Euler: absolute_orientation = (roll_deg, pitch_deg, yaw/heading_deg)
        # yaw/heading is 0–360° CW from magnetic North.
        roll_rad = float(abs_orientation_deg[0]) * _DEG_TO_RAD
        pitch_rad = float(abs_orientation_deg[1]) * _DEG_TO_RAD
        yaw_rad = _bno055_heading_to_enu_yaw_rad(float(abs_orientation_deg[2]))

        # Steering: logged integer degrees → radians.
        # NOTE: This assumes 1:1 servo-angle to road-wheel-angle and 1 deg = 1 unit.
        # TODO: Replace with a calibrated servo-to-road-wheel transfer function.
        steering_rad = steering_angle_deg * _STEERING_DEG_TO_RAD

        # ── Initialise EKF on first valid opportunity ──────────────────────
        if not self._ekf_initialized:
            self._try_init_ekf(yaw_rad, roll_rad, pitch_rad)
            if not self._ekf_initialized:
                return  # Still waiting (GPS_IMU mode waiting for fix)

        # ── Predict step ───────────────────────────────────────────────────
        self._ekf.predict(
            accel_body_x_mps2=accel_x,
            dt=max(dt, 1e-4),
            steering_angle_rad=steering_rad,
        )

        # ── IMU measurement updates (every loop, high trust) ───────────────
        # The BNO055 NDOF orientation is trusted heavily.  R matrices are small
        # (σ ≈ 0.026 rad) so these updates drive the orientation states close to
        # the sensor.  This is the core of the original sensor-fusion intent.
        self._ekf.update_roll(roll_rad)
        self._ekf.update_pitch(pitch_rad)
        self._ekf.update_yaw(yaw_rad)
        self._ekf.update_yaw_rate(gyro_z_radps)

        # Lateral accel update: uses tire-force model.  EKF skips it internally
        # below the low-speed threshold where the model is unreliable.
        self._ekf.update_lateral_accel(accel_y, steering_rad)

        # ── GPS updates (GPS_IMU mode only, when fix is valid) ─────────────
        if NavigationMode.uses_gps(self._navigation_mode):
            self._ekf.update_from_gps_data(self._gps_data)

    def _try_init_ekf(self, yaw_rad: float, roll_rad: float, pitch_rad: float) -> None:
        """
        Attempt EKF initialisation.  Sets ``_ekf_initialized`` on success.

        IMU_ONLY mode: initialises immediately at local origin (0, 0) with IMU
                       orientation.  No GPS required.

        GPS_IMU mode: waits for the first valid GPS fix.  Once a fix arrives the
                      EKF origin is set there and yaw is seeded from IMU (not GPS
                      track) because IMU heading is more reliable at low speed.
        """
        assert self._ekf is not None

        if self._navigation_mode == NavigationMode.EKF_IMU_ONLY:
            self._ekf.initialize(
                position_east_m=0.0,
                position_north_m=0.0,
                yaw_rad=yaw_rad,
                roll_rad=roll_rad,
                pitch_rad=pitch_rad,
            )
            self._ekf_initialized = True
            print("EKF initialised (IMU-only mode) at local origin.")

        elif self._navigation_mode == NavigationMode.EKF_GPS_IMU:
            gps = self._gps_data
            if gps.get('has_2d_fix') and gps.get('latitude') is not None:
                # Pass yaw_rad from IMU — more reliable than GPS track at startup.
                self._ekf.initialize_from_gps(
                    latitude_deg=float(gps['latitude']),
                    longitude_deg=float(gps['longitude']),
                    speed_m_s=gps.get('speed_m_s'),
                    track_deg=gps.get('track_deg'),
                    yaw_rad=yaw_rad,
                )
                self._ekf_initialized = True
                print(
                    f"EKF initialised (GPS+IMU mode) at "
                    f"{gps['latitude']:.6f}, {gps['longitude']:.6f}."
                )

    def _get_ekf_log_row(self) -> list:
        """
        Return EKF state as an ordered list matching ``CSV_HEADERS_EKF``.

        Returns empty strings when the EKF is not active or not yet initialised.
        """
        if self._ekf is None or not self._ekf_initialized:
            return [False] + [''] * (len(self.CSV_HEADERS_EKF) - 1)

        est = self._ekf.get_estimate()
        x = est.state
        p = est.covariance
        return [
            True,
            x[0],   # ekf_east_m
            x[1],   # ekf_north_m
            x[2],   # ekf_yaw_rad
            x[3],   # ekf_roll_rad
            x[4],   # ekf_pitch_rad
            x[5],   # ekf_vx_body_mps
            x[6],   # ekf_vy_body_mps
            x[7],   # ekf_yaw_rate_radps
            x[8],   # ekf_bias_ax_mps2
            x[9],   # ekf_bias_ay_mps2
            x[10],  # ekf_bias_gz_radps
            float(np.sqrt(max(p[0, 0], 0.0))),   # ekf_pos_1sigma_m
            float(np.sqrt(max(p[2, 2], 0.0))),   # ekf_yaw_1sigma_rad
            float(np.sqrt(max(p[5, 5], 0.0))),   # ekf_vel_1sigma_mps
        ]

    # ── CSV helpers ────────────────────────────────────────────────────────

    def build_log_row(
        self,
        receiver_data_dict: dict,
        odometry_data: dict,
    ) -> list:
        """Build one CSV row from all active data sources."""
        battery_percent = self.safe_battery_value(self.__battery.retrieve_percentage())
        battery_seconds_left = self.safe_battery_value(self.__battery.retrieve_seconds_left())
        battery_time_remaining = self.safe_battery_value(self.__battery.get_time_remaining())
        battery_plugged_in = self.safe_battery_value(self.__battery.get_plugged_in())

        raw_acceleration = self.ensure_vector(odometry_data.get('raw_acceleration'))
        body_acceleration = self.ensure_vector(odometry_data.get('body_acceleration'))
        acceleration = self.ensure_vector(odometry_data.get('acceleration'))
        velocity = self.ensure_vector(odometry_data.get('velocity'))
        position = self.ensure_vector(odometry_data.get('position'))
        gravity = self.ensure_vector(odometry_data.get('gravity'))
        absolute_orientation = self.ensure_vector(odometry_data.get('absolute_orientation'))
        relative_orientation = self.ensure_vector(odometry_data.get('relative_orientation'))
        initial_orientation = self.ensure_vector(odometry_data.get('initial_orientation'))
        quaternion = self.ensure_vector(odometry_data.get('quaternion'), length=4)
        angular_velocity = self.ensure_vector(odometry_data.get('angular_velocity'))
        magnetic = self.ensure_vector(odometry_data.get('magnetic'))

        gps = self._gps_data
        gps_row = [
            gps.get('latitude', ''),
            gps.get('longitude', ''),
            bool(gps.get('has_2d_fix', False)),
            gps.get('speed_m_s', ''),
            gps.get('track_deg', ''),
        ]

        core_row = [
            self.__time,
            self.__runtime,
            self.__delT,
            self._navigation_mode,
            receiver_data_dict.get('pulse_width', 0),
            receiver_data_dict.get('angle', 0),
            receiver_data_dict.get('sample_age_sec', ''),
            receiver_data_dict.get('is_fresh', False),
            battery_percent,
            battery_seconds_left,
            battery_time_remaining,
            battery_plugged_in,
            self.safe_battery_value(odometry_data.get('temperature')),
            raw_acceleration[0], raw_acceleration[1], raw_acceleration[2],
            body_acceleration[0], body_acceleration[1], body_acceleration[2],
            acceleration[0], acceleration[1], acceleration[2],
            velocity[0], velocity[1], velocity[2],
            position[0], position[1], position[2],
            gravity[0], gravity[1], gravity[2],
            absolute_orientation[0], absolute_orientation[1], absolute_orientation[2],
            relative_orientation[0], relative_orientation[1], relative_orientation[2],
            initial_orientation[0], initial_orientation[1], initial_orientation[2],
            quaternion[0], quaternion[1], quaternion[2], quaternion[3],
            angular_velocity[0], angular_velocity[1], angular_velocity[2],
            magnetic[0], magnetic[1], magnetic[2],
        ]

        return core_row + gps_row + self._get_ekf_log_row()

    def ensure_vector(self, value: object, length: int = 3) -> tuple:
        """Normalise a vector-like value to a fixed-length tuple."""
        if isinstance(value, np.ndarray):
            value = value.tolist()
        if isinstance(value, (list, tuple)):
            normalized = list(value[:length])
            if len(normalized) < length:
                normalized.extend([''] * (length - len(normalized)))
            return tuple(normalized)
        return tuple([''] * length)

    def safe_battery_value(self, value: object) -> object:
        """Convert None battery values to empty string for CSV."""
        return '' if value is None else value

    def init_csv(self, filename: str) -> None:
        """Create or overwrite the CSV log file with the header row."""
        path = pathlib.Path(self.csv_data_dir, f"{filename}.csv")
        with open(path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile, delimiter=',', quotechar='|',
                                quoting=csv.QUOTE_MINIMAL)
            writer.writerow(self.CSV_HEADERS)

    def add_to_csv(self, filename: str, row: list) -> None:
        """Append a single data row to the CSV log."""
        path = pathlib.Path(self.csv_data_dir, f"{filename}.csv")
        with open(path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile, delimiter=',', quotechar='|',
                                quoting=csv.QUOTE_MINIMAL)
            writer.writerow(row)

    # ── Geometry helpers ───────────────────────────────────────────────────

    def get_midpoint(self, p1: tuple, p2: tuple) -> tuple:
        (x1, y1) = p1
        (x2, y2) = p2
        return ((x1 + x2) / 2, (y1 + y2) / 2)

    def get_distance(self, p1: tuple, p2: tuple) -> float:
        x1, x2 = p1[0], p2[0]
        y1, y2 = p1[1], p2[1]
        return float(np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2))

    def decide(self) -> None:
        """Feedback/control logic stub (not yet implemented)."""
        pass


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Run the RC car controller telemetry loop with configurable IMU/odometry filters."
    )
    parser.add_argument(
        "filename",
        nargs="?",
        default="data",
        help="Base filename for CSV output in the data directory.",
    )
    parser.add_argument(
        "legacy_verbose",
        nargs="?",
        default=None,
        help="Legacy positional verbose flag for backward compatibility.",
    )
    parser.add_argument(
        "legacy_mode",
        nargs="?",
        default=None,
        help="Legacy positional navigation mode for backward compatibility.",
    )
    parser.add_argument(
        "--verbose",
        dest="verbose",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Enable or disable verbose console output.",
    )
    parser.add_argument(
        "--mode",
        choices=NavigationMode.ALL,
        default=None,
        help="Navigation pipeline to run.",
    )
    parser.add_argument(
        "--post-calib",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Enable the IMU stationary post-calibration pass.",
    )
    parser.add_argument(
        "--post-calib-samples",
        type=int,
        default=256,
        help="Stationary samples to capture during post-calibration.",
    )
    parser.add_argument(
        "--post-calib-period",
        type=float,
        default=0.02,
        help="Seconds between post-calibration samples.",
    )
    parser.add_argument(
        "--post-calib-gravity",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Split stationary accel/gravity mismatch across both outputs during post-calibration.",
    )
    parser.add_argument(
        "--filter-gyro",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Enable moving-average filtering on gyroscope readings.",
    )
    parser.add_argument(
        "--gyro-window",
        type=int,
        default=3,
        help="Gyroscope moving-average window size.",
    )
    parser.add_argument(
        "--filter-linear-accel",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable moving-average filtering on linear acceleration.",
    )
    parser.add_argument(
        "--linear-accel-window",
        type=int,
        default=3,
        help="Linear-acceleration moving-average window size.",
    )
    parser.add_argument(
        "--low-pass",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable first-order low-pass filtering on linear acceleration.",
    )
    parser.add_argument(
        "--low-pass-cutoff",
        type=float,
        default=2.0,
        help="Low-pass cutoff frequency in hertz for linear acceleration.",
    )
    parser.add_argument(
        "--orientation-low-pass",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable angle-aware low-pass filtering on IMU Euler orientation.",
    )
    parser.add_argument(
        "--orientation-low-pass-cutoff",
        type=float,
        default=2.0,
        help="Low-pass cutoff frequency in hertz for IMU Euler orientation.",
    )

    cli_args = parser.parse_args()

    filename = str(cli_args.filename)
    verbose = (
        str(cli_args.legacy_verbose).strip().lower() in ("true", "1", "yes", "y")
        if cli_args.legacy_verbose is not None
        else True
    )
    if cli_args.verbose is not None:
        verbose = cli_args.verbose

    mode = cli_args.legacy_mode if cli_args.legacy_mode is not None else NavigationMode.ODOMETRY_ONLY
    if cli_args.mode is not None:
        mode = cli_args.mode

    imu = IMU(
        enabled=(True if robotSupported else False),
        verbose=verbose,
        enable_post_calibration=cli_args.post_calib,
        post_calibration_sample_count=cli_args.post_calib_samples,
        post_calibration_sample_period_sec=cli_args.post_calib_period,
        post_calibrate_gravity=cli_args.post_calib_gravity,
    )
    odometry = Odometry(
        verbose=verbose,
        enabled=(True if robotSupported else False),
        imu=imu,
        calibrate_imu=True,
        filter_gyro=cli_args.filter_gyro,
        gyro_filter_window_size=cli_args.gyro_window,
        filter_linear_acceleration=cli_args.filter_linear_accel,
        linear_acceleration_filter_window_size=cli_args.linear_accel_window,
        low_pass_linear_acceleration=cli_args.low_pass,
        linear_acceleration_low_pass_cutoff_hz=cli_args.low_pass_cutoff,
        low_pass_orientation=cli_args.orientation_low_pass,
        orientation_low_pass_cutoff_hz=cli_args.orientation_low_pass_cutoff,
    )

    controller = Controller(
        verbose=verbose,
        enabled=(True if robotSupported else False),
        log_to_csv=True,
        csv_data_dir_name="data",
        csv_data_filename=filename,
        navigation_mode=mode,
        odometry=odometry,
    )
    controller.run()
