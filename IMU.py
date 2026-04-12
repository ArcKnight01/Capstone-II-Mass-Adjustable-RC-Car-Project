import sys
import os
import time

import numpy as np

from IMUUtil import *

try:
    robotSupported = os.uname().nodename == ('terminatorpi' or 'robotpi' or 'carpi')
except:
    import platform
    robotSupported = platform.uname().node == ('terminatorpi' or 'robotpi' or 'carpi')
    
if robotSupported:
    import board
    import busio
    import adafruit_bno055

import math

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
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
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

    def __init__(self,
                 enabled:bool=True, 
                 verbose : bool = True,
                 use_alternate_imu_address : bool = False,
                 mode : Mode = Mode.NDOF_MODE,
                 use_manual_calibration : bool = False
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
            Whether manual calibration should be used.

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

        self.__last_val = 0xFFFF
        self.__zeroed_orientation_offset = (0,0,0)
        self.__calibrated : bool = False
        print(f"ACCEL RANGE: {self.__sensor.accel_mode}G")

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
        result = self.__sensor.temperature
        if abs(result - self.__last_val) == 128:
            result = self.__sensor.temperature
            if abs(result - self.__last_val) == 128:
                return 0b00111111 & result
        self.__last_val = result
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
        self.__zeroed_orientation_offset = self.__sensor.euler
    
    def get_zeroed_orientation(self) -> tuple[float, float, float]:
        """
        Get the stored zero-reference orientation.

        Parameters
        ----------
        None

        Returns
        -------
        tuple[float, float, float]
            Stored zeroed Euler orientation.
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
        self.set_zeroed_orientation()
        print(f"BNO055 IMU has completed calibration, calibration status is {self.__calibrated}")
    
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
        return self.__sensor.quaternion
    
    def get_euler_angles(self) -> tuple[float, float, float]:
        """
        Retrieve the current Euler orientation angles.

        Parameters
        ----------
        None

        Returns
        -------
        tuple[float, float, float]
            Current Euler angles from the IMU.
        """
        return self.__sensor.euler
    
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
        return self.__sensor.linear_acceleration
    
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
        return self.__sensor.gravity

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
        return self.__sensor.acceleration
    
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
        return self.__sensor.gyro
    
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
        return self.__sensor.magnetic

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
