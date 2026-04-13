"""
IMU.py - Interface to Adafruit BNO055 9-DOF IMU sensor.

This module provides a wrapper around the BNO055 IMU sensor, handling initialization,
calibration, and data retrieval. It performs interactive calibration on startup and
provides access to raw sensor vectors, Euler angles, quaternions, and temperature.

Data Flow:
1. Initialize I2C connection to BNO055 (Pi only) or mock data (Windows)
2. Perform calibration sequence (gyro, magnetometer, accelerometer)
3. Provide methods to read: acceleration, gyro, magnetometer, orientation, temperature

Usage:
    python IMU.py
    - Runs calibration and prints sensor readings in a loop

Note: Requires hardware on Raspberry Pi; uses mock data on Windows.
"""

import sys
import os
import time

import numpy as np

from IMUUtil import *

try:
    robotSupported = os.uname().nodename in ('terminatorpi', 'robotpi', 'carpi')
except:
    import platform
    robotSupported = platform.uname().node in ('terminatorpi', 'robotpi', 'carpi')
    
if robotSupported:
    import board
    import busio
    import adafruit_bno055

import math

STANDARD_GRAVITY_MPS2 = 9.80665

def normalize_bno055_euler(
    euler: tuple[float | None, float | None, float | None]
    | list[float | None]
    | None
    | np.ndarray
) -> tuple[float | None, float | None, float | None]:
    """
    Reorder the BNO055 Euler tuple into ``(roll, pitch, yaw)``.

    The Adafruit BNO055 driver exposes Euler output as ``(heading, roll, pitch)``.
    Most of this project expects the more common ``(roll, pitch, yaw)`` ordering,
    so we normalize it at the IMU boundary.
    """
    if euler is None:
        return (None, None, None)

    try:
        heading, roll, pitch = euler
    except (TypeError, ValueError):
        return (None, None, None)

    return roll, pitch, heading

def quaternion_rotation_matrix(Q: tuple[float, float, float, float] | list[float] | np.ndarray) -> np.ndarray:
    """
    Convert a quaternion into a three-dimensional rotation matrix.

    Parameters
    ----------
    Q : tuple[float, float, float, float] | list[float] | numpy.ndarray
        Quaternion values ordered as ``(q0, q1, q2, q3)``.

    Returns
    -------
    numpy.ndarray
        ``3 x 3`` rotation matrix that converts points from the local reference
        frame to the global reference frame.
    """
    quaternion = np.asarray(Q, dtype=float)
    if quaternion.shape != (4,):
        raise ValueError(f"Expected a 4-element quaternion, got shape {quaternion.shape}.")

    norm = np.linalg.norm(quaternion)
    if norm == 0.0 or not np.isfinite(norm):
        raise ValueError("Quaternion must contain finite, non-zero values.")

    q0, q1, q2, q3 = quaternion / norm
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix

def quaternion_to_euler_angle(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    """
    Convert quaternion components into Euler angles in degrees.

    Parameters
    ----------
    w : float
        Scalar quaternion component.
    x : float
        X quaternion component.
    y : float
        Y quaternion component.
    z : float
        Z quaternion component.

    Returns
    -------
    tuple[float, float, float]
        Euler angles ``(roll, pitch, yaw)`` in degrees.
    """
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return X, Y, Z

# The default I2C address
DEFAULT_I2C_ADDR = 40

# The alternate I2C address 
ALT_I2C_ADDR = 41

# How often to update the BNO sensor data (in hertz).
BNO_UPDATE_FREQUENCY_HZ = 10

# The BNO055 Mode, deprecated possibly.
class Mode:
    CONFIG_MODE = 0x00
    ACCONLY_MODE = 0x01
    MAGONLY_MODE = 0x02
    GYRONLY_MODE = 0x03
    ACCMAG_MODE = 0x04
    ACCGYRO_MODE = 0x05
    MAGGYRO_MODE = 0x06
    AMG_MODE = 0x07
    IMUPLUS_MODE = 0x08
    COMPASS_MODE = 0x09
    M4G_MODE = 0x0A
    NDOF_FMC_OFF_MODE = 0x0B
    NDOF_MODE = 0x0C


class IMU(object):
    """
    A class for the BNO055 IMU 

    - Note: +z is up when the sensor is face up.

    Data values:

    temperature - The sensor temperature in degrees Celsius.

    acceleration - This is a 3-tuple of X, Y, Z axis accelerometer values in meters per
    second squared.

    magnetic - This is a 3-tuple of X, Y, Z axis magnetometer values in microteslas.

    gyro - This is a 3-tuple of X, Y, Z axis gyroscope values in degrees per second.

    euler - This is a 3-tuple of orientation Euler angle values.

    quaternion - This is a 4-tuple of orientation quaternion values.

    linear_acceleration - This is a 3-tuple of X, Y, Z linear acceleration values (i.e.
    without effect of gravity) in meters per second squared.

    gravity - This is a 3-tuple of X, Y, Z gravity acceleration values (i.e. without the
    effect of linear acceleration) in meters per second squared.

    https://docs.circuitpython.org/projects/bno055/en/latest/api.html 
    """

    @staticmethod
    def _to_vector3(
        values: tuple[float | None, float | None, float | None]
        | list[float | None]
        | np.ndarray
        | None
    ) -> np.ndarray | None:
        """Convert a candidate sensor reading into a finite ``(3,)`` vector."""
        if values is None:
            return None

        try:
            vector = np.asarray(values, dtype=float)
        except (TypeError, ValueError):
            return None

        if vector.shape != (3,) or not np.all(np.isfinite(vector)):
            return None

        return vector

    @staticmethod
    def _vector_to_tuple(vector: np.ndarray | None) -> tuple[float, float, float]:
        """Convert a vector into a plain Python tuple."""
        if vector is None:
            return (0.0, 0.0, 0.0)

        return tuple(np.asarray(vector, dtype=float).tolist())

    @staticmethod
    def _midrange_offset(samples: list[np.ndarray]) -> np.ndarray:
        """
        Estimate a vector offset using the legacy midpoint-of-range method.

        This mirrors the older ADCS/odometry calibration style where the offset is
        computed as the midpoint between the minimum and maximum observed value on
        each axis. It is provided for compatibility and experimentation only.
        """
        if not samples:
            return np.zeros(3, dtype=float)

        sample_array = np.vstack(samples)
        return 0.5 * (np.min(sample_array, axis=0) + np.max(sample_array, axis=0))

    def __init__(self,
                 enabled:bool=True, 
                 verbose : bool = True,
                 use_alternate_imu_address : bool = False,
                 mode : Mode = Mode.NDOF_MODE,
                 use_manual_calibration : bool = False,
                 enable_post_calibration : bool = True,
                 post_calibration_sample_count : int = 128,
                 post_calibration_sample_period_sec : float = 0.02,
                 post_calibrate_gravity : bool = True
                 ):
        """
        Initialize the BNO055 IMU interface.

        Parameters
        ----------
        enabled : bool, optional
            Whether the IMU should be enabled.
        verbose : bool, optional
            Whether verbose output should be enabled.
        use_alternate_imu_address : bool, optional
            Whether to use the alternate I2C address.
        mode : Mode, optional
            BNO055 operating mode to use.
        use_manual_calibration : bool, optional
            Legacy compatibility flag. Manual vector calibration is now handled by
            the post-calibration step.
        enable_post_calibration : bool, optional
            Whether to run an additional stationary vector calibration after the
            BNO055 reports itself calibrated.
        post_calibration_sample_count : int, optional
            Number of stationary samples to capture during the post-calibration pass.
        post_calibration_sample_period_sec : float, optional
            Delay between post-calibration samples in seconds.
        post_calibrate_gravity : bool, optional
            Whether to split stationary acceleration/gravity mismatch across both
            raw acceleration and gravity outputs.

        Returns
        -------
        None
            This constructor initializes the IMU sensor interface.
        """
        
        self.__verbose = verbose
        self.__enabled = enabled
        if use_alternate_imu_address:
            self.__i2c_address = ALT_I2C_ADDR
        else:
            self.__i2c_address = DEFAULT_I2C_ADDR
        self.__i2c = busio.I2C(board.SCL, board.SDA)
        self.__sensor = adafruit_bno055.BNO055_I2C(self.__i2c, address=self.__i2c_address)
        
        # set the mode:
        self.__sensor.mode = mode
        self.__mode = mode

        self.__last_val = 0xFFFF
        self.__zeroed_orientation_offset = (0,0,0)
        self.__calibrated : bool = False
        self.__enable_post_calibration = bool(enable_post_calibration or use_manual_calibration)
        self.__post_calibration_sample_count = max(1, int(post_calibration_sample_count))
        self.__post_calibration_sample_period_sec = max(0.0, float(post_calibration_sample_period_sec))
        self.__post_calibrate_gravity = bool(post_calibrate_gravity)
        self.__linear_acceleration_offset = np.zeros(3, dtype=float)
        self.__raw_acceleration_offset = np.zeros(3, dtype=float)
        self.__gravity_offset = np.zeros(3, dtype=float)
        self.__post_calibration_sample_total = 0
        self.__post_calibration_gravity_reference = STANDARD_GRAVITY_MPS2
        self.__last_temperature = 0
        self.__last_quaternion = (1.0, 0.0, 0.0, 0.0)
        self.__last_euler = (0.0, 0.0, 0.0)
        self.__last_linear_acceleration = (0.0, 0.0, 0.0)
        self.__last_gravity_vector = (0.0, 0.0, STANDARD_GRAVITY_MPS2)
        self.__last_raw_acceleration = (0.0, 0.0, 0.0)
        self.__last_raw_gyro = (0.0, 0.0, 0.0)
        self.__last_raw_magnetometer = (0.0, 0.0, 0.0)
        print(f"ACCEL RANGE: {self.__sensor.accel_mode}G")

    def _safe_sensor_read(self, read_fn, fallback, *, context: str):
        """
        Execute a sensor read and return a fallback value on transient I2C failures.

        Parameters
        ----------
        read_fn : callable
            Zero-argument callable that performs the hardware read.
        fallback : Any
            Value to return when the sensor read fails.
        context : str
            Human-readable label describing the measurement being read.

        Returns
        -------
        Any
            Fresh sensor data when available, otherwise ``fallback``.
        """
        try:
            return read_fn()
        except OSError as exc:
            print(f"IMU warning: failed to read {context} ({exc}); reusing last value.")
            return fallback

    @property
    def calibrated(self) -> bool:
        """Whether the BNO055 has completed its built-in calibration."""
        return self.__calibrated

    @calibrated.setter
    def calibrated(self, value: bool) -> None:
        self.__calibrated = bool(value)

    def get_temperature(self) -> int:
        """
        Retrieve the sensor temperature.

        Parameters
        ----------
        None

        Returns
        -------
        int
            Sensor temperature in degrees Celsius.
        """
        # global last_val  # noqa: PLW0603
        result = self._safe_sensor_read(
            lambda: self.__sensor.temperature,
            self.__last_temperature,
            context="temperature",
        )
        if abs(result - self.__last_val) == 128:
            result = self._safe_sensor_read(
                lambda: self.__sensor.temperature,
                self.__last_temperature,
                context="temperature retry",
            )
            if abs(result - self.__last_val) == 128:
                result = 0b00111111 & result
        self.__last_val = result
        self.__last_temperature = result
        return result
    
    def set_zeroed_orientation(self) -> None:
        """
        Store the current Euler orientation as the zero reference.

        Parameters
        ----------
        None

        Returns
        -------
        None
            This method updates the stored orientation offset.
        """
        self.__zeroed_orientation_offset = normalize_bno055_euler(self.__sensor.euler)
    
    def get_zeroed_orientation(self) -> tuple[float, float, float]:
        """
        Get the stored zero-reference orientation.

        Parameters
        ----------
        None

        Returns
        -------
        tuple[float, float, float]
            Stored zeroed orientation as ``(roll, pitch, yaw)`` in degrees.
        """
        return self.__zeroed_orientation_offset 

    def calibrate(self) -> None:
        """
        Run the full IMU calibration sequence.

        Parameters
        ----------
        None

        Returns
        -------
        None
            This method calibrates the magnetometer, accelerometer, and gyroscope.
        """
        self.calibrate_magnetometer()
        time.sleep(1)
        self.calibrate_accelerometer()
        time.sleep(1)
        self.calibrate_gyro()
        self.calibrated = self.__sensor.calibrated 
        while (self.__sensor.calibration_status[0] == 0) and (self.__mode == Mode.NDOF_MODE):
            print("Waiting on SYS to calibrate -- The BNO055 has NOT found the North Pole. \nWhen in NDOF mode,  any data where the system calibration value is '0'should generally be ignored")
            time.sleep(1)
        if self.__enable_post_calibration:
            self.calibrate_vector_outputs()
        self.set_zeroed_orientation()
        print(f"BNO055 IMU has completed calibration, calibration status is {self.__calibrated}")

    def calibrate_vector_outputs(self) -> None:
        """
        Estimate stationary offsets for derived acceleration channels.

        The BNO055 handles its own internal calibration, but its fused
        ``linear_acceleration`` output can still retain a small bias that causes
        large drift after integration. This extra pass assumes the sensor is held
        still and captures a short stationary window to remove:
        - linear acceleration bias relative to the expected zero vector
        - raw/gravity disagreement while stationary
        """
        calibration_pause = 3.0
        print("Post-Calibration: Hold the IMU still to capture fused sensor bias.")
        time.sleep(calibration_pause)
        print("Post-Calibration: Sampling linear acceleration, acceleration, and gravity...")

        linear_samples: list[np.ndarray] = []
        raw_samples: list[np.ndarray] = []
        gravity_samples: list[np.ndarray] = []

        max_attempts = max(self.__post_calibration_sample_count * 3, self.__post_calibration_sample_count)
        attempts = 0
        while len(linear_samples) < self.__post_calibration_sample_count and attempts < max_attempts:
            attempts += 1

            linear_vector = self._to_vector3(self.__sensor.linear_acceleration)
            raw_vector = self._to_vector3(self.__sensor.acceleration)
            gravity_vector = self._to_vector3(self.__sensor.gravity)
            if linear_vector is None or raw_vector is None or gravity_vector is None:
                time.sleep(self.__post_calibration_sample_period_sec)
                continue

            linear_samples.append(linear_vector)
            raw_samples.append(raw_vector)
            gravity_samples.append(gravity_vector)

            if self.__verbose and len(linear_samples) in (1, self.__post_calibration_sample_count):
                print(
                    "Post-Calibration Sample "
                    f"{len(linear_samples)}/{self.__post_calibration_sample_count}: "
                    f"lin={self._vector_to_tuple(linear_vector)}, "
                    f"acc={self._vector_to_tuple(raw_vector)}, "
                    f"grav={self._vector_to_tuple(gravity_vector)}"
                )

            time.sleep(self.__post_calibration_sample_period_sec)

        if not linear_samples:
            print("Post-Calibration: No valid stationary samples captured; keeping zero offsets.")
            return

        linear_mean = np.mean(np.vstack(linear_samples), axis=0)
        raw_mean = np.mean(np.vstack(raw_samples), axis=0)
        gravity_mean = np.mean(np.vstack(gravity_samples), axis=0)

        stationary_accel_gravity_delta = raw_mean - gravity_mean
        if self.__post_calibrate_gravity:
            self.__raw_acceleration_offset = 0.5 * stationary_accel_gravity_delta
            self.__gravity_offset = -0.5 * stationary_accel_gravity_delta
        else:
            self.__raw_acceleration_offset = stationary_accel_gravity_delta
            self.__gravity_offset = np.zeros(3, dtype=float)

        self.__linear_acceleration_offset = linear_mean
        self.__post_calibration_sample_total = len(linear_samples)

        gravity_norm = float(np.linalg.norm(gravity_mean))
        if np.isfinite(gravity_norm) and gravity_norm > 0.0:
            self.__post_calibration_gravity_reference = gravity_norm

        print(
            "Post-Calibration complete:"
            f" linear_offset={self._vector_to_tuple(self.__linear_acceleration_offset)},"
            f" acceleration_offset={self._vector_to_tuple(self.__raw_acceleration_offset)},"
            f" gravity_offset={self._vector_to_tuple(self.__gravity_offset)},"
            f" gravity_norm={self.__post_calibration_gravity_reference:.3f} m/s^2,"
            f" samples={self.__post_calibration_sample_total}"
        )
    
    def calibrate_magnetometer(self) -> None:
        """
        Calibrate the IMU magnetometer.

        Parameters
        ----------
        None

        Returns
        -------
        None
            This method blocks until the magnetometer calibration completes.
        """
        print("Magnetometer: Move sensor away from magnetic interference or shields. Perform the figure-eight until calibrated.")
        while not self.__sensor.calibration_status[3] == 3:
            # Calibration Dance Step One: Magnetometer
            #   Move sensor away from magnetic interference or shields
            #   Perform the figure-eight until calibrated
            print(f"Mag Calib Status: {100 / 3 * self.__sensor.calibration_status[3]:3.0f}%")
            time.sleep(1)
        print("... CALIBRATED")
        time.sleep(1)
        print(f"  Offsets_Magnetometer:  {self.__sensor.offsets_magnetometer}")

    def calibrate_accelerometer(self) -> None:
        """
        Calibrate the IMU accelerometer.

        Parameters
        ----------
        None

        Returns
        -------
        None
            This method blocks until the accelerometer calibration completes.
        """
        print("Accelerometer: Perform the six-step calibration dance.")
        print("Place sensor board into six stable positions for a few seconds each:")
        print("1) x-axis right, y-axis up,    z-axis away")
        print("2) x-axis up,    y-axis left,  z-axis away")
        print("3) x-axis left,  y-axis down,  z-axis away")
        print("4) x-axis down,  y-axis right, z-axis away")
        print("5) x-axis left,  y-axis right, z-axis up")
        print("6) x-axis right, y-axis left,  z-axis down")
        print("Repeat the steps until calibrated")
        while not self.__sensor.calibration_status[2] == 3:
            # Calibration Dance Step Two: Accelerometer
            #   Place sensor board into six stable positions for a few seconds each:
            #    1) x-axis right, y-axis up,    z-axis away
            #    2) x-axis up,    y-axis left,  z-axis away
            #    3) x-axis left,  y-axis down,  z-axis away
            #    4) x-axis down,  y-axis right, z-axis away
            #    5) x-axis left,  y-axis right, z-axis up
            #    6) x-axis right, y-axis left,  z-axis down
            #   Repeat the steps until calibrated
            print(f"Accel Calib Status: {100 / 3 * self.__sensor.calibration_status[2]:3.0f}%")
            time.sleep(1)
        print("... CALIBRATED")
        time.sleep(1)
        print(f"  Offsets_Accelerometer: {self.__sensor.offsets_accelerometer}")
        

    def calibrate_gyro(self) -> None:
        """
        Calibrate the IMU gyroscope.

        Parameters
        ----------
        None

        Returns
        -------
        None
            This method blocks until the gyroscope calibration completes.
        """
        print("Gyroscope: Perform the hold-in-place calibration dance.")
        print("Place sensor in any stable position for a few seconds\n(Accelerometer calibration may also calibrate the gyro)")
        while not self.__sensor.calibration_status[1] == 3:
            # Calibration Dance Step Three: Gyroscope
            #  Place sensor in any stable position for a few seconds
            #  (Accelerometer calibration may also calibrate the gyro)
            print(f"Gyro Calib Status: {100 / 3 * self.__sensor.calibration_status[1]:3.0f}%")
            time.sleep(1)
        print("... CALIBRATED")
        time.sleep(1)
        print(f"  Offsets_Gyroscope:     {self.__sensor.offsets_gyroscope}")


    def get_quaternion(self) -> tuple[float, float, float, float]:
        """
        Retrieve the current orientation quaternion.

        Parameters
        ----------
        None

        Returns
        -------
        tuple[float, float, float, float]
            Current quaternion from the IMU.
        """
        quaternion = self._safe_sensor_read(
            lambda: self.__sensor.quaternion,
            self.__last_quaternion,
            context="quaternion",
        )
        if quaternion is None:
            return self.__last_quaternion

        self.__last_quaternion = quaternion
        return quaternion
    
    def get_euler_angles(self) -> tuple[float, float, float]:
        """
        Retrieve the current Euler orientation angles.

        Parameters
        ----------
        None

        Returns
        -------
        tuple[float, float, float]
            Current Euler angles as ``(roll, pitch, yaw)`` in degrees.
        """
        euler = self._safe_sensor_read(
            lambda: normalize_bno055_euler(self.__sensor.euler),
            self.__last_euler,
            context="Euler angles",
        )
        if euler is None:
            return self.__last_euler

        self.__last_euler = euler
        return euler
    
    def get_rotation_matrix(self) -> np.ndarray:
        """
        Compute a rotation matrix from the current quaternion.

        Parameters
        ----------
        None

        Returns
        -------
        numpy.ndarray
            ``3 x 3`` rotation matrix derived from the current quaternion.
        """
        # TODO calculate 3x3 rotation matrix from quaternion
        # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        return quaternion_rotation_matrix(self.get_quaternion())
    
    def get_linear_acceleration(self) -> tuple[float, float, float]:
        """
        Retrieve the current linear acceleration.

        Parameters
        ----------
        None

        Returns
        -------
        tuple[float, float, float]
            Linear acceleration in meters per second squared, excluding gravity.
        """
        linear_acceleration = self._safe_sensor_read(
            lambda: self._to_vector3(self.__sensor.linear_acceleration),
            self._to_vector3(self.__last_linear_acceleration),
            context="linear acceleration",
        )
        if linear_acceleration is None:
            return self.__last_linear_acceleration

        corrected_linear_acceleration = linear_acceleration - self.__linear_acceleration_offset
        corrected_linear_acceleration_tuple = self._vector_to_tuple(corrected_linear_acceleration)
        self.__last_linear_acceleration = corrected_linear_acceleration_tuple
        return corrected_linear_acceleration_tuple
    
    def get_gravity_vector(self) -> tuple[float, float, float]:
        """
        Retrieve the current gravity vector.

        Parameters
        ----------
        None

        Returns
        -------
        tuple[float, float, float]
            Gravity vector in meters per second squared.
        """
        gravity_vector = self._safe_sensor_read(
            lambda: self._to_vector3(self.__sensor.gravity),
            self._to_vector3(self.__last_gravity_vector),
            context="gravity vector",
        )
        if gravity_vector is None:
            return self.__last_gravity_vector

        corrected_gravity_vector = gravity_vector - self.__gravity_offset
        corrected_gravity_vector_tuple = self._vector_to_tuple(corrected_gravity_vector)
        self.__last_gravity_vector = corrected_gravity_vector_tuple
        return corrected_gravity_vector_tuple

    def get_raw_acceleration(self) -> tuple[float, float, float]:
        """
        Retrieve the raw accelerometer reading.

        Parameters
        ----------
        None

        Returns
        -------
        tuple[float, float, float]
            Raw acceleration values from the sensor.
        """
        raw_acceleration = self._safe_sensor_read(
            lambda: self._to_vector3(self.__sensor.acceleration),
            self._to_vector3(self.__last_raw_acceleration),
            context="raw acceleration",
        )
        if raw_acceleration is None:
            return self.__last_raw_acceleration

        corrected_raw_acceleration = raw_acceleration - self.__raw_acceleration_offset
        corrected_raw_acceleration_tuple = self._vector_to_tuple(corrected_raw_acceleration)
        self.__last_raw_acceleration = corrected_raw_acceleration_tuple
        return corrected_raw_acceleration_tuple
    
    def get_raw_gyro(self) -> tuple[float, float, float]:
        """
        Retrieve the raw gyroscope reading.

        Parameters
        ----------
        None

        Returns
        -------
        tuple[float, float, float]
            Raw angular velocity values from the sensor.
        """
        gyro = self._safe_sensor_read(
            lambda: self.__sensor.gyro,
            self.__last_raw_gyro,
            context="gyroscope",
        )
        if gyro is None:
            return self.__last_raw_gyro

        self.__last_raw_gyro = gyro
        return gyro
    
    def get_raw_magnetometer(self) -> tuple[float, float, float]:
        """
        Retrieve the raw magnetometer reading.

        Parameters
        ----------
        None

        Returns
        -------
        tuple[float, float, float]
            Raw magnetic field values from the sensor.
        """
        magnetic = self._safe_sensor_read(
            lambda: self.__sensor.magnetic,
            self.__last_raw_magnetometer,
            context="magnetometer",
        )
        if magnetic is None:
            return self.__last_raw_magnetometer

        self.__last_raw_magnetometer = magnetic
        return magnetic

    # Legacy/manual ADCS-style helpers kept for experimentation only.
    # The active pipeline continues to use the BNO055 fused outputs directly.

    def manual_estimate_orientation_from_accel_mag(
        self,
        acceleration: tuple[float, float, float] | list[float] | np.ndarray | None = None,
        magnetic: tuple[float, float, float] | list[float] | np.ndarray | None = None,
        mag_offset: tuple[float, float, float] | list[float] | np.ndarray = (0.0, 0.0, 0.0),
    ) -> tuple[float, float, float]:
        """
        Estimate orientation using the legacy accelerometer/magnetometer equations.

        This wraps the ADCS-style helper functions in ``IMUUtil.py`` and is not
        used by the current odometry path.
        """
        accel_vector = self._to_vector3(self.__sensor.acceleration if acceleration is None else acceleration)
        magnetic_vector = self._to_vector3(self.__sensor.magnetic if magnetic is None else magnetic)
        magnetic_offset = self._to_vector3(mag_offset)
        if accel_vector is None or magnetic_vector is None:
            return (0.0, 0.0, 0.0)
        if magnetic_offset is None:
            magnetic_offset = np.zeros(3, dtype=float)

        corrected_magnetic = magnetic_vector - magnetic_offset
        roll = roll_am(accel_vector[0], accel_vector[1], accel_vector[2])
        pitch = pitch_am(accel_vector[0], accel_vector[1], accel_vector[2])
        yaw = yaw_am(
            accel_vector[0],
            accel_vector[1],
            accel_vector[2],
            corrected_magnetic[0],
            corrected_magnetic[1],
            corrected_magnetic[2],
        )
        return (float(roll), float(pitch), float(yaw))

    def manual_estimate_orientation_from_gyro(
        self,
        previous_orientation: tuple[float, float, float] | list[float] | np.ndarray,
        dt: float,
        gyro: tuple[float, float, float] | list[float] | np.ndarray | None = None,
        gyro_is_radians: bool = True,
    ) -> tuple[float, float, float]:
        """
        Integrate gyroscope data with the legacy ADCS equations.

        Parameters are intentionally explicit so this helper can be used for
        offline experiments without touching the main BNO055 fusion path.
        """
        previous_vector = self._to_vector3(previous_orientation)
        gyro_vector = self._to_vector3(self.__sensor.gyro if gyro is None else gyro)
        if previous_vector is None or gyro_vector is None:
            return (0.0, 0.0, 0.0)

        gyro_rates = np.rad2deg(gyro_vector) if gyro_is_radians else gyro_vector
        roll = roll_gy(previous_vector[0], dt, gyro_rates[0])
        pitch = pitch_gy(previous_vector[1], dt, gyro_rates[1])
        yaw = yaw_gy(previous_vector[2], dt, gyro_rates[2])
        return (float(roll), float(pitch), float(yaw))

    def manual_estimate_orientation_complementary(
        self,
        previous_orientation: tuple[float, float, float] | list[float] | np.ndarray,
        dt: float,
        weight: float = 0.5,
        acceleration: tuple[float, float, float] | list[float] | np.ndarray | None = None,
        magnetic: tuple[float, float, float] | list[float] | np.ndarray | None = None,
        gyro: tuple[float, float, float] | list[float] | np.ndarray | None = None,
        mag_offset: tuple[float, float, float] | list[float] | np.ndarray = (0.0, 0.0, 0.0),
        gyro_is_radians: bool = True,
    ) -> tuple[float, float, float]:
        """
        Estimate orientation with the legacy complementary filter helpers.

        This is a compatibility wrapper around ``roll_F``, ``pitch_F``, and
        ``yaw_F`` from ``IMUUtil.py`` and is not used by the active odometry flow.
        """
        previous_vector = self._to_vector3(previous_orientation)
        accel_vector = self._to_vector3(self.__sensor.acceleration if acceleration is None else acceleration)
        magnetic_vector = self._to_vector3(self.__sensor.magnetic if magnetic is None else magnetic)
        gyro_vector = self._to_vector3(self.__sensor.gyro if gyro is None else gyro)
        magnetic_offset = self._to_vector3(mag_offset)
        if previous_vector is None or accel_vector is None or magnetic_vector is None or gyro_vector is None:
            return (0.0, 0.0, 0.0)
        if magnetic_offset is None:
            magnetic_offset = np.zeros(3, dtype=float)

        corrected_magnetic = magnetic_vector - magnetic_offset
        gyro_rates = np.rad2deg(gyro_vector) if gyro_is_radians else gyro_vector
        roll = roll_F(
            previous_vector[0],
            dt,
            gyro_rates[0],
            accel_vector[0],
            accel_vector[1],
            accel_vector[2],
            weight,
        )
        pitch = pitch_F(
            previous_vector[1],
            dt,
            gyro_rates[1],
            accel_vector[0],
            accel_vector[1],
            accel_vector[2],
            weight,
        )
        yaw = yaw_F(
            previous_vector[2],
            dt,
            gyro_rates[2],
            accel_vector[0],
            accel_vector[1],
            accel_vector[2],
            corrected_magnetic[0],
            corrected_magnetic[1],
            corrected_magnetic[2],
            weight,
        )
        return (float(roll), float(pitch), float(yaw))

    def manual_set_initial_orientation(
        self,
        mag_offset: tuple[float, float, float] | list[float] | np.ndarray = (0.0, 0.0, 0.0),
        calibration_pause_sec: float = 0.001,
    ) -> tuple[float, float, float]:
        """
        Capture an initial ADCS-style orientation from acceleration and magnetics.

        This mirrors the older ``set_initial`` helper but does not affect the
        current BNO055-driven zeroed orientation unless explicitly called.
        """
        print("[CALIBRATION] Preparing to set initial orientation. Please hold the IMU still.")
        time.sleep(max(0.0, calibration_pause_sec))
        print("[CALIBRATION] Setting orientation...")
        orientation = self.manual_estimate_orientation_from_accel_mag(mag_offset=mag_offset)
        print("[CALIBRATION] Initial orientation set.")
        print(f"[CALIBRATION] Initial Orientation (Roll,Pitch,Yaw): {list(orientation)}")
        return orientation

    def manual_zero_orientation(
        self,
        orientation: tuple[float, float, float] | list[float] | np.ndarray,
        zero_reference: tuple[float, float, float] | list[float] | np.ndarray | None = None,
    ) -> tuple[float, float, float]:
        """
        Zero an orientation tuple against a chosen reference orientation.

        This mirrors the intent of the older ``zero_orientation`` helper without
        mutating the active BNO055 orientation state.
        """
        orientation_vector = self._to_vector3(orientation)
        reference_vector = self._to_vector3(self.get_zeroed_orientation() if zero_reference is None else zero_reference)
        if orientation_vector is None or reference_vector is None:
            return (0.0, 0.0, 0.0)

        zeroed_orientation = orientation_vector - reference_vector
        return self._vector_to_tuple(zeroed_orientation)

    def manual_calibrate_linear_acceleration_offset(
        self,
        sample_count: int = 10,
        sample_period_sec: float = 1.0,
    ) -> tuple[float, float, float]:
        """
        Estimate a linear-acceleration offset using the old odometry midrange rule.

        This helper samples the BNO055 ``linear_acceleration`` output but does not
        apply the result anywhere automatically.
        """
        print("[CALIBRATION] Preparing to calibrate linear acceleration. Please hold still.")
        time.sleep(max(0.0, sample_period_sec))
        print("[CALIBRATION] Calibrating...")

        samples: list[np.ndarray] = []
        for sample_index in range(max(1, int(sample_count))):
            linear_vector = self._to_vector3(self.__sensor.linear_acceleration)
            if linear_vector is None:
                time.sleep(max(0.0, sample_period_sec))
                continue
            if self.__verbose:
                print(f"Linear Accel (x,y,z) @ n={sample_index}: {self._vector_to_tuple(linear_vector)}")
            samples.append(linear_vector)
            time.sleep(max(0.0, sample_period_sec))

        offset = self._midrange_offset(samples)
        print("[CALIBRATION] Calibration complete.")
        print(f"[CALIBRATION] Linear acceleration offsets: {self._vector_to_tuple(offset)}.")
        return self._vector_to_tuple(offset)

    def manual_calibrate_magnetometer_offset(
        self,
        sample_count: int = 10,
        sample_period_sec: float = 0.001,
    ) -> tuple[float, float, float]:
        """
        Estimate a magnetometer offset using the legacy midpoint-of-range method.

        This mirrors the older ``calibrate_mag`` helper and is provided for
        experimentation only.
        """
        print("[CALIBRATION] Preparing to calibrate magnetometer. Please wave around.")
        time.sleep(max(0.0, sample_period_sec))
        print("[CALIBRATION] Calibrating...")

        samples: list[np.ndarray] = []
        for sample_index in range(max(1, int(sample_count))):
            magnetic_vector = self._to_vector3(self.__sensor.magnetic)
            if magnetic_vector is None:
                time.sleep(max(0.0, sample_period_sec))
                continue
            if self.__verbose:
                print(f"Mag(x,y,z)@{sample_index}: {self._vector_to_tuple(magnetic_vector)}")
            samples.append(magnetic_vector)
            time.sleep(max(0.0, sample_period_sec))

        offset = self._midrange_offset(samples)
        print("[CALIBRATION] Calibration complete.")
        print(f"[CALIBRATION] Magnetometer offsets: {self._vector_to_tuple(offset)}.")
        return self._vector_to_tuple(offset)

    def manual_calibrate_gyro_offset(
        self,
        sample_count: int = 10,
        sample_period_sec: float = 0.001,
        gyro_is_radians: bool = True,
    ) -> tuple[float, float, float]:
        """
        Estimate a gyroscope offset using the legacy midpoint-of-range method.

        Set ``gyro_is_radians=False`` if the supplied gyro samples are already in
        degrees per second.
        """
        print("[CALIBRATION] Preparing to calibrate gyroscope. Please hold still.")
        time.sleep(max(0.0, sample_period_sec))
        print("[CALIBRATION] Calibrating...")

        samples: list[np.ndarray] = []
        for sample_index in range(max(1, int(sample_count))):
            gyro_vector = self._to_vector3(self.__sensor.gyro)
            if gyro_vector is None:
                time.sleep(max(0.0, sample_period_sec))
                continue

            gyro_sample = np.rad2deg(gyro_vector) if gyro_is_radians else gyro_vector
            if self.__verbose:
                print(f"Gyro(x,y,z)@{sample_index}: {self._vector_to_tuple(gyro_sample)}")
            samples.append(gyro_sample)
            time.sleep(max(0.0, sample_period_sec))

        offset = self._midrange_offset(samples)
        print("[CALIBRATION] Calibration complete.")
        print(f"[CALIBRATION] Gyroscope offsets: {self._vector_to_tuple(offset)}.")
        return self._vector_to_tuple(offset)

    def manual_find_north(
        self,
        gravity_vector: tuple[float, float, float] | list[float] | np.ndarray | None = None,
        magnetic_vector: tuple[float, float, float] | list[float] | np.ndarray | None = None,
    ) -> tuple[float, float, float]:
        """
        Estimate a north-referenced orientation from gravity and magnetic vectors.

        This mirrors the older ``find_north`` helper. By default it uses the
        BNO055 gravity estimate and raw magnetic field measurement.
        """
        gravity_candidate = self._to_vector3(self.__sensor.gravity if gravity_vector is None else gravity_vector)
        magnetic_candidate = self._to_vector3(self.__sensor.magnetic if magnetic_vector is None else magnetic_vector)
        if gravity_candidate is None or magnetic_candidate is None:
            return (0.0, 0.0, 0.0)

        gravity_norm = np.linalg.norm(gravity_candidate)
        magnetic_norm = np.linalg.norm(magnetic_candidate)
        if gravity_norm == 0.0 or magnetic_norm == 0.0:
            return (0.0, 0.0, 0.0)

        gravity_unit = gravity_candidate / gravity_norm
        magnetic_unit = magnetic_candidate / magnetic_norm
        east = np.cross(gravity_unit, magnetic_unit)
        east_norm = np.linalg.norm(east)
        if east_norm == 0.0:
            return (0.0, 0.0, 0.0)

        east = east / east_norm
        north = np.cross(east, gravity_unit)
        roll_north = np.degrees(np.arctan2(north[2], north[1]))
        pitch_north = np.degrees(np.arctan2(north[2], north[0]))
        yaw_north = np.degrees(np.arctan2(north[0], north[1]))
        return (float(roll_north), float(pitch_north), float(yaw_north))

    def manual_get_gravity_corrected_acceleration(
        self,
        acceleration: tuple[float, float, float] | list[float] | np.ndarray | None = None,
        roll_deg: float | None = None,
        pitch_deg: float | None = None,
        gravity_mps2: float = STANDARD_GRAVITY_MPS2,
    ) -> tuple[float, float, float]:
        """
        Remove gravity from raw acceleration using the older roll/pitch projection.

        This is intentionally separate from the main pipeline because the BNO055
        already provides fused ``linear_acceleration`` and ``gravity`` outputs.
        """
        accel_vector = self._to_vector3(self.__sensor.acceleration if acceleration is None else acceleration)
        if accel_vector is None:
            return (0.0, 0.0, 0.0)

        if roll_deg is None:
            roll_deg = roll_am(accel_vector[0], accel_vector[1], accel_vector[2])
        if pitch_deg is None:
            pitch_deg = pitch_am(accel_vector[0], accel_vector[1], accel_vector[2])

        g_x = gravity_mps2 * math.sin(math.radians(pitch_deg))
        g_y = gravity_mps2 * math.sin(math.radians(roll_deg))
        g_z_theta = math.atan(
            math.sqrt(
                (math.tan(math.radians(roll_deg)) ** 2)
                + (math.tan(math.radians(pitch_deg)) ** 2)
            )
        )
        g_z = gravity_mps2 * math.cos(g_z_theta)

        corrected_x = accel_vector[0] - g_x
        corrected_y = accel_vector[1] - g_y
        corrected_z = accel_vector[2] - g_z if accel_vector[2] > 0 else accel_vector[2] + g_z
        return (float(corrected_x), float(corrected_y), float(corrected_z))

    def get_calibration_statuses(self) -> None:
        """
        Print the current IMU calibration status.

        Parameters
        ----------
        None

        Returns
        -------
        None
            This method prints the current calibration state.
        """
        status = self.__sensor.calibration_status
        calibrated = "CALIBRATED" if self.__sensor.calibrated else "NOT CALIBRATED"

        print(f"The IMU is {calibrated}. (sys,gyro,accel,mag = {status})")

    def get_post_calibration_offsets(self) -> dict[str, tuple[float, float, float] | int | float]:
        """
        Retrieve the current post-calibration offsets.

        Returns
        -------
        dict[str, tuple[float, float, float] | int | float]
            Stationary offsets applied to linear acceleration, raw acceleration,
            and gravity-derived outputs.
        """
        return {
            "linear_acceleration_offset": self._vector_to_tuple(self.__linear_acceleration_offset),
            "raw_acceleration_offset": self._vector_to_tuple(self.__raw_acceleration_offset),
            "gravity_offset": self._vector_to_tuple(self.__gravity_offset),
            "sample_count": self.__post_calibration_sample_total,
            "gravity_reference_mps2": self.__post_calibration_gravity_reference,
        }

    def get_calibration_data(self) -> tuple[tuple[int, int, int, int], bool]:
        """
        Retrieve the IMU calibration tuple and calibrated flag.

        Parameters
        ----------
        None

        Returns
        -------
        tuple[tuple[int, int, int, int], bool]
            Calibration status tuple and overall calibrated state.
        """
        
        return self.__sensor.calibration_status, self.__sensor.calibrated

if __name__ == "__main__":
    if not robotSupported:
        print("IMU test loop can only run on supported robot hardware.")
        sys.exit(1)

    imu = IMU()
    update_period_sec = 1 / BNO_UPDATE_FREQUENCY_HZ

    print("Starting IMU test loop. Press Ctrl+C to stop.")
    try:
        while True:
            temperature = imu.get_temperature()
            quaternion = imu.get_quaternion()
            euler = imu.get_euler_angles()
            rotation_matrix = imu.get_rotation_matrix()
            linear_acceleration = imu.get_linear_acceleration()
            gravity = imu.get_gravity_vector()
            raw_acceleration = imu.get_raw_acceleration()
            raw_gyro = imu.get_raw_gyro()
            raw_magnetometer = imu.get_raw_magnetometer()
            calibration_status, calibrated = imu.get_calibration_data()

            print("=" * 60)
            print(f"Temperature (C):      {temperature}")
            print(f"Quaternion:           {quaternion}")
            print(f"Euler Angles:         {euler}")
            print(f"Rotation Matrix:\n{rotation_matrix}")
            print(f"Linear Accel (m/s^2): {linear_acceleration}")
            print(f"Gravity (m/s^2):      {gravity}")
            print(f"Raw Accel (m/s^2):    {raw_acceleration}")
            print(f"Raw Gyro (deg/s):     {raw_gyro}")
            print(f"Raw Magnet (uT):      {raw_magnetometer}")
            print(f"Calibration Status:   {calibration_status}")
            print(f"Is Calibrated:        {calibrated}")

            time.sleep(update_period_sec)
    except KeyboardInterrupt:
        print("\nStopped IMU test loop.")
