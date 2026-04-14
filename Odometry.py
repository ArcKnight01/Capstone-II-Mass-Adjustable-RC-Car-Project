"""
Odometry.py - IMU-based dead reckoning and motion estimation.

This module integrates IMU acceleration data to estimate vehicle position, velocity,
and orientation. It applies optional filtering (moving average, low-pass) to smooth
sensor readings and performs trapezoidal integration for velocity/position estimation.

Data Flow:
1. Read IMU: raw acceleration, gyro, magnetometer, quaternion
2. Apply filters: moving average on gyro/accel, low-pass on linear accel
3. Rotate body-frame accel to world frame using quaternion
4. Integrate: accel → velocity → position (trapezoidal rule)
5. Output: position, velocity, orientation in world frame

Usage:
    python Odometry.py
    - Initializes IMU, runs calibration, prints odometry data in loop

Configuration: Filters and integration parameters via constructor arguments.
"""

import sys
import os
import time
import csv
import numpy as np
import pandas as pd

try:
    robotSupported = os.uname().nodename in ('terminatorpi', 'robotpi', 'carpi')
except:
    import platform
    robotSupported = platform.uname().node in ('terminatorpi', 'robotpi', 'carpi')
if robotSupported:
    import board
    import busio
    import adafruit_bno055


from IMUUtil import *
from RobotClock import Clock
from IMU import *
from MovingAverageFilter import MovingAverageFilter
from LowPassFilter import LowPassFilter
from HighPassFilter import HighPassFilter

class Odometry(object):
    """ Calculates position, velocity and angular velocity from acceleration and orientation"""
    def __init__(self,
                 verbose:bool=True, 
                 enabled:bool=True,
                 imu = None,
                 calibrate_imu: bool = True,
                 initial_position : tuple = (0,0,0),
                 filter_gyro: bool = False,
                 gyro_filter_window_size: int = 3,
                 filter_linear_acceleration: bool = True,
                 linear_acceleration_filter_window_size: int = 3,
                 low_pass_linear_acceleration: bool = True,
                 linear_acceleration_low_pass_cutoff_hz: float = 2.0,

                 ):
        """
        Initialize the odometry subsystem.

        Parameters
        ----------
        verbose : bool, optional
            Whether verbose output should be enabled.
        enabled : bool, optional
            Whether odometry processing should be enabled.
        imu : object, optional
            IMU instance to use. If ``None``, a default IMU is created.
        calibrate_imu : bool, optional
            Whether to run the IMU calibration sequence during construction.
            Set this to ``False`` when passing in a preconfigured IMU that has
            already been calibrated.
        initial_position : tuple, optional
            Initial position of the robot as an ``(x, y, z)`` tuple.
        filter_gyro : bool, optional
            Whether to smooth gyroscope readings with a moving-average filter.
        gyro_filter_window_size : int, optional
            Window size for the gyroscope moving-average filter.
        filter_linear_acceleration : bool, optional
            Whether to smooth linear acceleration readings with a moving-average filter.
        linear_acceleration_filter_window_size : int, optional
            Window size for the linear-acceleration moving-average filter.
        low_pass_linear_acceleration : bool, optional
            Whether to apply a first-order low-pass filter to linear acceleration.
        linear_acceleration_low_pass_cutoff_hz : float, optional
            Cutoff frequency in hertz for the linear-acceleration low-pass filter.

        Returns
        -------
        None
            This constructor initializes odometry state and calibrates the IMU.
        """
        self.__clock = Clock()
        self.__imu = imu if imu is not None else IMU()
        if calibrate_imu:
            try:
                self.__imu.calibrate()
            except Exception:
                print("IMU failed to calibrate!")

        print(f"IMU Calibration Status: {self.__imu.calibrated}")

        #Determine whether to print data to terminal
        self.__verbose : bool = verbose
        #Set whether odometry is enabled
        self.__enabled : bool = enabled
        self.__filter_gyro: bool = bool(filter_gyro)
        self.__filter_linear_acceleration: bool = bool(filter_linear_acceleration)
        self.__low_pass_linear_acceleration: bool = bool(low_pass_linear_acceleration)
        
        self.__body_acceleration : tuple = (0,0,0)
        self.__raw_body_acceleration : tuple = (0,0,0)
        self.__acceleration : tuple = (0,0,0)
        self.__velocity : tuple = (0,0,0)
        self.__position : tuple = initial_position

        self.__previous_acceleration : tuple = (0,0,0)
        self.__previous_velocity : tuple = (0,0,0)
        self.__previous_position : tuple = initial_position
        self.__raw_gyro : tuple = (0,0,0)
        self.__quaternion : tuple = (1,0,0,0)
        self.__rotation_matrix : np.ndarray = np.eye(3, dtype=float)
        self.__gyro_filters = self._build_vector_filters(
            enabled=self.__filter_gyro,
            window_size=gyro_filter_window_size,
        )
        self.__linear_acceleration_filters = self._build_vector_filters(
            enabled=self.__filter_linear_acceleration,
            window_size=linear_acceleration_filter_window_size,
        )
        self.__linear_acceleration_low_pass_filters = self._build_vector_low_pass_filters(
            enabled=self.__low_pass_linear_acceleration,
            cutoff_hz=linear_acceleration_low_pass_cutoff_hz,
        )

        # orientation (roll, pitch, yaw)
        self.__initial_orientation : tuple = self.__imu.get_zeroed_orientation()
        self.__absolute_orientation : tuple = self.__initial_orientation
        self.__relative_orientation : tuple = (0,0,0)
        time.sleep(1)

    @staticmethod
    def _to_vector3(values: tuple | list | np.ndarray | None) -> np.ndarray:
        """
        Convert a sensor reading into a finite ``(3,)`` NumPy array.

        Parameters
        ----------
        values : tuple, list, np.ndarray, or None
            Raw sensor vector with exactly three elements.  If ``None``, not
            finite, or not shape ``(3,)``, a zero vector is returned instead.

        Returns
        -------
        np.ndarray
            Shape ``(3,)`` array of ``float64``.  All elements are finite.
            Returns ``np.zeros(3)`` when the input is invalid or ``None``.
        """
        if values is None:
            return np.zeros(3, dtype=float)

        try:
            vector = np.asarray(values, dtype=float)
        except (TypeError, ValueError):
            return np.zeros(3, dtype=float)

        if vector.shape != (3,) or not np.all(np.isfinite(vector)):
            return np.zeros(3, dtype=float)

        return vector

    @staticmethod
    def _build_vector_filters(enabled: bool, window_size: int) -> tuple[MovingAverageFilter, MovingAverageFilter, MovingAverageFilter] | None:
        """
        Create one moving-average filter per vector axis when filtering is enabled.

        Parameters
        ----------
        enabled : bool
            When ``False`` the function returns ``None`` and no filters are created.
        window_size : int
            Number of samples in the moving-average window for each axis.

        Returns
        -------
        tuple of MovingAverageFilter or None
            A 3-tuple ``(filter_x, filter_y, filter_z)`` when ``enabled`` is
            ``True``, or ``None`` when ``enabled`` is ``False``.
        """
        if not enabled:
            return None

        return tuple(MovingAverageFilter(window_size) for _ in range(3))

    @staticmethod
    def _build_vector_low_pass_filters(enabled: bool, cutoff_hz: float) -> tuple[LowPassFilter, LowPassFilter, LowPassFilter] | None:
        """
        Create one first-order low-pass filter per vector axis when filtering is enabled.

        Parameters
        ----------
        enabled : bool
            When ``False`` the function returns ``None`` and no filters are created.
        cutoff_hz : float
            Cutoff frequency in hertz applied to each axis filter.

        Returns
        -------
        tuple of LowPassFilter or None
            A 3-tuple ``(filter_x, filter_y, filter_z)`` when ``enabled`` is
            ``True``, or ``None`` when ``enabled`` is ``False``.
        """
        if not enabled:
            return None

        return tuple(LowPassFilter(cutoff_hz) for _ in range(3))

    @staticmethod
    def _build_vector_high_pass_filters(enabled: bool, cutoff_hz: float) -> tuple[HighPassFilter, HighPassFilter, HighPassFilter] | None:
        """
        Create one first-order high-pass filter per vector axis when filtering is enabled.

        Parameters
        ----------
        enabled : bool
            When ``False`` the function returns ``None`` and no filters are created.
        cutoff_hz : float
            Cutoff frequency in hertz applied to each axis filter.

        Returns
        -------
        tuple of HighPassFilter or None
            A 3-tuple ``(filter_x, filter_y, filter_z)`` when ``enabled`` is
            ``True``, or ``None`` when ``enabled`` is ``False``.
        """
        if not enabled:
            return None

        return tuple(HighPassFilter(cutoff_hz) for _ in range(3))

    @staticmethod
    def _apply_vector_moving_average_filters(
        values: tuple | list | np.ndarray | None,
        filters: tuple[MovingAverageFilter, MovingAverageFilter, MovingAverageFilter] | None,
    ) -> tuple[float, float, float]:
        """
        Apply per-axis moving-average filters to a 3-element sensor vector.

        Parameters
        ----------
        values : tuple, list, np.ndarray, or None
            Raw 3-axis sensor reading.  Invalid or ``None`` inputs are treated
            as ``(0, 0, 0)`` via :meth:`_to_vector3`.
        filters : tuple of MovingAverageFilter or None
            A 3-tuple of filters, one per axis, as returned by
            :meth:`_build_vector_filters`.  When ``None``, the input vector is
            returned unchanged.

        Returns
        -------
        tuple of float
            3-element tuple ``(x, y, z)`` of filtered values.
        """
        vector = Odometry._to_vector3(values)
        if filters is None:
            return tuple(vector.tolist())

        return tuple(
            filter_axis.update(component)
            for filter_axis, component in zip(filters, vector)
        )

    @staticmethod
    def _apply_vector_low_pass_filters(
        values: tuple | list | np.ndarray | None,
        filters: tuple[LowPassFilter, LowPassFilter, LowPassFilter] | None,
        dt: float,
    ) -> tuple[float, float, float]:
        """
        Apply per-axis first-order low-pass filters to a 3-element sensor vector.

        Parameters
        ----------
        values : tuple, list, np.ndarray, or None
            Raw 3-axis sensor reading.  Invalid or ``None`` inputs are treated
            as ``(0, 0, 0)`` via :meth:`_to_vector3`.
        filters : tuple of LowPassFilter or None
            A 3-tuple of filters, one per axis, as returned by
            :meth:`_build_vector_low_pass_filters`.  When ``None``, the input
            vector is returned unchanged.
        dt : float
            Elapsed time in seconds since the previous call.  Passed to each
            filter's ``update`` method.

        Returns
        -------
        tuple of float
            3-element tuple ``(x, y, z)`` of filtered values.
        """
        vector = Odometry._to_vector3(values)
        if filters is None:
            return tuple(vector.tolist())

        return tuple(
            filter_axis.update(component, dt)
            for filter_axis, component in zip(filters, vector)
        )

    @staticmethod
    def _apply_vector_high_pass_filters(
        values: tuple | list | np.ndarray | None,
        filters: tuple[HighPassFilter, HighPassFilter, HighPassFilter] | None,
        dt: float,
    ) -> tuple[float, float, float]:
        """
        Apply per-axis first-order high-pass filters to a 3-element sensor vector.

        Parameters
        ----------
        values : tuple, list, np.ndarray, or None
            Raw 3-axis sensor reading.  Invalid or ``None`` inputs are treated
            as ``(0, 0, 0)`` via :meth:`_to_vector3`.
        filters : tuple of HighPassFilter or None
            A 3-tuple of filters, one per axis, as returned by
            :meth:`_build_vector_high_pass_filters`.  When ``None``, the input
            vector is returned unchanged.
        dt : float
            Elapsed time in seconds since the previous call.  Passed to each
            filter's ``update`` method.

        Returns
        -------
        tuple of float
            3-element tuple ``(x, y, z)`` of filtered values.
        """
        vector = Odometry._to_vector3(values)
        if filters is None:
            return tuple(vector.tolist())

        return tuple(
            filter_axis.update(component, dt)
            for filter_axis, component in zip(filters, vector)
        )

    def _update_rotation_matrix(self, quaternion: tuple | list | np.ndarray | None) -> None:
        """
        Refresh the cached body-to-world rotation matrix from a quaternion.

        The internal matrix is only replaced when ``quaternion`` produces a
        valid, finite 3×3 result from :func:`quaternion_rotation_matrix`.
        Invalid or ``None`` inputs leave the existing matrix unchanged.

        Parameters
        ----------
        quaternion : tuple, list, np.ndarray, or None
            Quaternion in ``(w, x, y, z)`` order as returned by the BNO055.

        Returns
        -------
        None
        """
        if quaternion is None:
            return

        try:
            candidate = quaternion_rotation_matrix(quaternion)
        except (TypeError, ValueError):
            return

        if candidate.shape == (3, 3) and np.all(np.isfinite(candidate)):
            self.__rotation_matrix = candidate

    def calibrate(self):
        """
        Calibrate the odometry subsystem.

        Parameters
        ----------
        None

        Returns
        -------
        None
            This method is currently a stub.
        """
        pass

    def get_data(self) -> dict[str,tuple]: 
        """ 
        Retrieve the most recently computed odometry values.

        Parameters
        ----------
        None

        Returns
        -------
        dict[str, tuple]
            Dictionary containing temperature, acceleration, velocity, position,
            gravity, orientation, angular velocity, and magnetic field values.
        """
        return {
            'temperature' : self.__temperature,
            'raw_acceleration' : self.__raw_acceleration,
            'raw_body_acceleration' : self.__raw_body_acceleration,
            'body_acceleration' : self.__body_acceleration,
            'acceleration' : self.__acceleration,
            'velocity' : self.__velocity,
            'position' : self.__position,
            'gravity' : self.__gravity,
            'absolute_orientation' : self.__absolute_orientation,
            'relative_orientation' : self.__relative_orientation,
            'initial_orientation' : self.__initial_orientation,
            'raw_angular_velocity' : self.__raw_gyro,
            'angular_velocity' : self.__gyro,
            'magnetic' : self.__magnetometer,
            'quaternion' : self.__quaternion,
        }
    

    def update(self, dt : float = 1.000):
        """
        Update odometry values using the latest IMU sample.

        Parameters
        ----------
        dt : float, optional
            Elapsed time in seconds since the previous update.

        Returns
        -------
        None
            This method updates temperature, acceleration, orientation, velocity, and position.
        """
        self.__temperature = self.__imu.get_temperature()
        
        
        self.__raw_gyro = self.__imu.get_raw_gyro()
        self.__gyro = self._apply_vector_moving_average_filters(
            self.__raw_gyro,
            self.__gyro_filters,
        )
        self.__raw_acceleration = self.__imu.get_raw_acceleration()
        self.__magnetometer = self.__imu.get_raw_magnetometer()

        self.__gravity = self.__imu.get_gravity_vector()
        self.__raw_body_acceleration = self.__imu.get_linear_acceleration()
        body_acceleration = self._apply_vector_moving_average_filters(
            self.__raw_body_acceleration,
            self.__linear_acceleration_filters,
        )
        self.__body_acceleration = self._apply_vector_low_pass_filters(
            body_acceleration,
            self.__linear_acceleration_low_pass_filters,
            dt,
        )
        self.__quaternion = self.__imu.get_quaternion()
        self._update_rotation_matrix(self.__quaternion)

        body_acceleration_vector = self._to_vector3(self.__body_acceleration)
        world_acceleration_vector = self.__rotation_matrix @ body_acceleration_vector
        self.__acceleration = tuple(world_acceleration_vector.tolist())

        absolute_orientation = self._to_vector3(self.__imu.get_euler_angles())
        zeroed_orientation = self._to_vector3(self.__imu.get_zeroed_orientation())
        self.__absolute_orientation = tuple(absolute_orientation.tolist())
        
        self.__relative_orientation = tuple((absolute_orientation - zeroed_orientation).tolist())
        self.__velocity = tuple(np.array(self.__previous_velocity) + 0.5 * (np.array(self.__acceleration) + np.array(self.__previous_acceleration)) * dt)

        self.__position = tuple(np.array(self.__previous_position) + 0.5 * (np.array(self.__velocity) + np.array(self.__previous_velocity)) * dt)
        self.__previous_position = self.__position
        self.__previous_velocity = self.__velocity
        self.__previous_acceleration = self.__acceleration
        self.__previous_orientation = self.__relative_orientation


if __name__ == "__main__":
    
    if not robotSupported:
        print("odometry test loop can only run on supported robot hardware.")
        sys.exit(1)
    

    odometry = Odometry() 
    try: 
        while True: 
            dt = 1
            odometry.update(dt=dt)
            data = odometry.get_data()
        
            time.sleep(dt)

    except KeyboardInterrupt:
        print("\nStopped odometry test loop.")
