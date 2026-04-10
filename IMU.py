import sys
import os
try:
    robotSupported = os.uname().nodename == 'terminatorpi' or 'robotpi' or 'carpi'
except:
    import platform
    robotSupported = platform.uname().node == 'terminatorpi' or 'robotpi' or 'carpi'
if robotSupported:
    import board
    import busio
    import adafruit_bno055

DEFAULT_I2C_ADDR = 40
ALT_I2C_ADDR = 41


# How often to update the BNO sensor data (in hertz).
BNO_UPDATE_FREQUENCY_HZ = 10

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

    """

    def __init__(self,
                 enabled:bool=True, 
                 verbose : bool = True,
                 use_alternate_imu_address : bool = False,
                 ):
        
        self.__verbose = verbose
        self.__enabled = enabled
        if use_alternate_imu_address:
            self.__i2c_address = ALT_I2C_ADDR
        else:
            self.__i2c_address = DEFAULT_I2C_ADDR
        self.__i2c = busio.I2C(board.SCL, board.SDA)
        self.__sensor = adafruit_bno055.BNO055_I2C(self.__i2c, address=self.__i2c_address)
    
    def verbose_print()
