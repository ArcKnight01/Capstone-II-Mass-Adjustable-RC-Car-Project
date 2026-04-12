import sys
import os
import time

import numpy as np

from IMUUtil import *

try:
    robotSupported = os.uname().nodename == 'terminatorpi' or 'robotpi' or 'carpi'
except:
    import platform
    robotSupported = platform.uname().node == 'terminatorpi' or 'robotpi' or 'carpi'
if robotSupported:
    import board
    import busio
    import adafruit_bno055

import math

def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
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

def quaternion_to_euler_angle(w, x, y, z):
    """
    https://stackoverflow.com/questions/56207448/efficient-quaternions-to-euler-transformation
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

    def get_temperature(self):
        # global last_val  # noqa: PLW0603
        result = self.__sensor.temperature
        if abs(result - self.__last_val) == 128:
            result = self.__sensor.temperature
            if abs(result - self.__last_val) == 128:
                return 0b00111111 & result
        self.__last_val = result
        return result
    
    def set_zeroed_orientation(self):
        self.__zeroed_orientation_offset = self.__sensor.euler
    
    def get_zeroed_orientation(self):
        return self.__zeroed_orientation_offset 

    def calibrate(self):
        self.calibrate_magnetometer()
        time.sleep(1)
        self.calibrate_accelerometer()
        time.sleep(1)
        self.calibrate_gyro()
        self.calibrated = self.__sensor.calibrated
        self.set_zeroed_orientation()
        print(f"BNO055 IMU has completed calibration, calibration status is {self.__calibrated}")
    
    def calibrate_magnetometer(self):
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

    def calibrate_accelerometer(self):
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
        

    def calibrate_gyro(self):
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


    def get_quaternion(self):
        return self.__sensor.quaternion
    
    def get_euler_angles(self):
        return self.__sensor.euler
    
    def get_rotation_matrix(self):
        # TODO calculate 3x3 rotation matrix from quaternion
        # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        return quaternion_rotation_matrix(self.get_quaternion())
    
    def get_linear_acceleration(self):
        """
        Returns the linear acceleration, without gravity, in m/s. 
        Returns an empty tuple of length 3 when this property has been disabled by the current mode.
        """
        return self.__sensor.linear_acceleration
    
    def get_gravity_vector(self):
        return self.__sensor.gravity

    def get_raw_acceleration(self):
        return self.__sensor.acceleration
    
    def get_raw_gyro(self):
        return self.__sensor.gyro
    
    def get_raw_magnetometer(self):
        return self.__sensor.magnetic

    def get_calibration_statuses(self):
        status = self.__sensor.calibration_status
        calibrated = "CALIBRATED" if self.__sensor.calibrated else "NOT CALIBRATED"

        print(f"The IMU is {calibrated}. (sys,gyro,accel,mag = {status})")

if __name__ == "__main__":
    pass