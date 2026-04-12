import sys
import os
import time
import csv
import numpy as np 
import pandas as pd

try:
    robotSupported = os.uname().nodename == ('terminatorpi' or 'robotpi' or 'carpi')
except:
    import platform
    robotSupported = platform.uname().node == ('terminatorpi' or 'robotpi' or 'carpi')
if robotSupported:
    import board
    import busio
    import adafruit_bno055


from IMUUtil import *
from RobotClock import Clock
from IMU import *

class Odometry(object):
    """ Calculates position, velocity and angular velocity from acceleration and orientation"""
    def __init__(self,
                 verbose:bool=True, 
                 enabled:bool=True,
                 imu = IMU(),
                 initial_position : tuple = (0,0,0)

                 ):
        self.__clock = Clock()
        self.__imu = imu
        try: 
            imu.calibrate()
        except: 
            print("IMU failed to calibrate!")
        pass

        print(f"IMU Calibration Status: {imu.calibrated}")

        #Determine whether to print data to terminal
        self.__verbose : bool = verbose
        #Set whether odometry is enabled
        self.__enabled : bool = enabled
        
        self.__acceleration : tuple = (0,0,0)
        self.__velocity : tuple = (0,0,0)
        self.__position : tuple = initial_position

        self.__previous_acceleration : tuple = (0,0,0)
        self.__previous_velocity : tuple = (0,0,0)
        self.__previous_position : tuple = initial_position

        # orientation (roll, pitch, yaw)
        self.__initial_orientation : tuple = self.__imu.get_zeroed_orientation()
        self.__absolute_orientation : tuple = self.__initial_orientation
        self.__relative_orientation : tuple = (0,0,0)
        time.sleep(1)

    def calibrate(self):
        pass

    def get_data(self) -> dict[str,tuple]: 
        """ 
        You must run this after update to avoid errors 
        
        """
        return {
            'temperature' : self.__temperature,
            'raw_acceleration' : self.__raw_acceleration,
            'acceleration' : self.__acceleration,
            'velocity' : self.__velocity,
            'position' : self.__position,
            'gravity' : self.__gravity,
            'absolute_orientation' : self.__absolute_orientation,
            'relative_orientation' : self.__relative_orientation,
            'initial_orientation' : self.__initial_orientation,
            'angular_velocity' : self.__gyro,
            'magnetic' : self.__magnetometer
        }
    
    def find_north(self):
        # get gravity direction (down)
        gX, gY, gZ = self.__gravity #m/s^2 #previously self.__acceleration, which works if still
        gravityVec = [gX, gY, gZ]/np.sqrt(gX**2+gY**2+gZ**2) # unit vector direction
        # get magnetic field direction
        magX, magY, magZ = self.__magnetometer #gauss
        magVec = [magX,magY,magZ]/np.sqrt(magX**2+magY**2+magZ**2) # unit vector direction
        # get East (cross gravity and mag field directions)
        east = np.cross(gravityVec,magVec)
        # get North (cross East and gravity)
        north = np.cross(east,gravityVec)
        rollN = np.arctan2(north[2],north[1])
        pitchN = np.arctan2(north[2],north[0])
        # assuming roll and pitch are zero - calculate the yaw from x and y values of North direction
        yawN = np.arctan2(north[0],north[1])
        return ([(180/np.pi)*rollN,(180/np.pi)*pitchN,(180/np.pi)*yawN])

    def update(self, dt : float = 1.000):
        self.__temperature = self.__imu.get_temperature()
        
        
        self.__gyro = self.__imu.get_raw_gyro()
        self.__raw_acceleration = self.__imu.get_raw_acceleration()
        self.__magnetometer = self.__imu.get_raw_magnetometer()

        self.__gravity = self.__imu.get_gravity_vector()
        self.__acceleration = self.__imu.get_linear_acceleration()

        self.__absolute_orientation = self.__imu.get_euler_angles()
        
        self.__relative_orientation = tuple(np.array(self.__absolute_orientation) - np.array(self.__imu.get_zeroed_orientation()))
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

