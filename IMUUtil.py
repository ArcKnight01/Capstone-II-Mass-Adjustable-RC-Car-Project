"""
IMUUtil.py - Utility functions for IMU sensor fusion.

This module contains helper functions for converting accelerometer and
magnetometer readings into roll, pitch, and yaw estimates, as well as
simple gyro-based integration helpers.
"""

import numpy as np

### IMU Util

def roll_am(accelX: float, accelY: float, accelZ: float) -> float:
    """
    Estimate roll from accelerometer measurements.

    Parameters
    ----------
    accelX : float
        X-axis acceleration.
    accelY : float
        Y-axis acceleration.
    accelZ : float
        Z-axis acceleration.

    Returns
    -------
    float
        Estimated roll angle in degrees.
    """
    if accelZ > 0:
        sign = 1
    else:
        sign = -1
    roll = (180/np.pi)*np.arctan2(accelY,sign*(accelX**2+accelZ**2)**0.5)
    

    roll_corrected = (180/np.pi)*np.arctan2(accelY,(accelX**2+accelZ**2)**0.5)
     # roll function using arctan2 - four quadrant answer
    if accelX >= 0 and accelZ >= 0 : 
        pass
    elif accelZ <= 0 and accelX >= 0 :
        roll_corrected = 180 - roll
    elif accelZ <= 0 and accelX < 0 :
        roll_corrected = 180 - roll
    elif accelZ >= 0 and accelX <= 0 :
        roll_corrected = roll + 360
    
    return roll # move to be 0 to 2pi from -pi to pi


def pitch_am(accelX: float, accelY: float, accelZ: float) -> float:
    """
    Estimate pitch from accelerometer measurements.

    Parameters
    ----------
    accelX : float
        X-axis acceleration.
    accelY : float
        Y-axis acceleration.
    accelZ : float
        Z-axis acceleration.

    Returns
    -------
    float
        Estimated pitch angle in degrees.
    """
    if accelZ > 0:
        sign = 1
    else:
        sign = -1
    pitch = (180/np.pi)*np.arctan2(accelX,sign*(accelY**2+accelZ**2)**0.5) 
    # pitch function using arctan2 - four quadrant answer
    return pitch # 


def yaw_am(accelX: float, accelY: float, accelZ: float, magX: float, magY: float, magZ: float) -> float:
    """
    Estimate yaw from accelerometer and magnetometer measurements.

    Parameters
    ----------
    accelX : float
        X-axis acceleration.
    accelY : float
        Y-axis acceleration.
    accelZ : float
        Z-axis acceleration.
    magX : float
        X-axis magnetic field value.
    magY : float
        Y-axis magnetic field value.
    magZ : float
        Z-axis magnetic field value.

    Returns
    -------
    float
        Estimated yaw angle in degrees.
    """
    #print(magX,magY,magZ)
    pitchR = (np.pi/180)*pitch_am(accelX,accelY,accelZ)
    rollR = (np.pi/180)*roll_am(accelX,accelY,accelZ) # change to radians for np functions to work
    magx = magX*np.cos(pitchR) + magY*np.sin(rollR)*np.sin(pitchR) + magZ*np.cos(rollR)*np.sin(pitchR)
    magy = magY*np.cos(rollR) - magZ*np.sin(rollR)
    yawR = np.arctan2(-magy,magx) # returning from - pi to pi for some reason (shift to 0-pi)
    return ((180/np.pi)*yawR*2 + 360) % 360  # changing to degrees at the end and moving to 0 to pi

###Gyro Sensor outputs in rad/s, so to calculate the RPY, you want to take the previous value (rads) and add the gyro (rad/s) * the time in number of seconds (s)
def roll_gy(prev_angle: float, delT: float, gyro: float) -> float:
    """
    Integrate gyroscope data to update roll.

    Parameters
    ----------
    prev_angle : float
        Previous roll angle in degrees.
    delT : float
        Elapsed time in seconds.
    gyro : float
        Gyroscope roll rate.

    Returns
    -------
    float
        Updated roll angle in degrees.
    """
    roll = prev_angle + gyro*delT
    return np.mod(roll,360)

def pitch_gy(prev_angle: float, delT: float, gyro: float) -> float:
    """
    Integrate gyroscope data to update pitch.

    Parameters
    ----------
    prev_angle : float
        Previous pitch angle in degrees.
    delT : float
        Elapsed time in seconds.
    gyro : float
        Gyroscope pitch rate.

    Returns
    -------
    float
        Updated pitch angle in degrees.
    """
    pitch = prev_angle + gyro*delT
    return np.mod(pitch,360)
    
def yaw_gy(prev_angle: float, delT: float, gyro: float) -> float:
    """
    Integrate gyroscope data to update yaw.

    Parameters
    ----------
    prev_angle : float
        Previous yaw angle in degrees.
    delT : float
        Elapsed time in seconds.
    gyro : float
        Gyroscope yaw rate.

    Returns
    -------
    float
        Updated yaw angle in degrees.
    """
    yaw = prev_angle + gyro*delT
    return np.mod(yaw,360)

# SENSOR FUSION (complementary filter (gyro + accelmag))
def roll_F(prev_angle: float, delT: float, gyro: float, accelX: float, accelY: float, accelZ: float, weight: float) -> float:
    """
    Estimate roll using a complementary filter.

    Parameters
    ----------
    prev_angle : float
        Previous roll angle in degrees.
    delT : float
        Elapsed time in seconds.
    gyro : float
        Gyroscope roll rate.
    accelX : float
        X-axis acceleration.
    accelY : float
        Y-axis acceleration.
    accelZ : float
        Z-axis acceleration.
    weight : float
        Weight applied to the gyroscope estimate.

    Returns
    -------
    float
        Filtered roll angle in degrees.
    """

    roll = weight*(roll_gy(prev_angle,delT,gyro)) + (1-weight)*(roll_am(accelX, accelY, accelZ))
    return np.mod(roll,360)

def pitch_F(prev_angle: float, delT: float, gyro: float, accelX: float, accelY: float, accelZ: float, weight: float) -> float:
    """
    Estimate pitch using a complementary filter.

    Parameters
    ----------
    prev_angle : float
        Previous pitch angle in degrees.
    delT : float
        Elapsed time in seconds.
    gyro : float
        Gyroscope pitch rate.
    accelX : float
        X-axis acceleration.
    accelY : float
        Y-axis acceleration.
    accelZ : float
        Z-axis acceleration.
    weight : float
        Weight applied to the gyroscope estimate.

    Returns
    -------
    float
        Filtered pitch angle in degrees.
    """
    
    pitch = weight*(pitch_gy(prev_angle,delT,gyro)) + (1-weight)*(pitch_am(accelX,accelY,accelZ))
    return np.mod(pitch,360)

def yaw_F(prev_angle: float, delT: float, gyro: float, accelX: float, accelY: float, accelZ: float, magX: float, magY: float, magZ: float, weight: float) -> float:
    """
    Estimate yaw using a complementary filter.

    Parameters
    ----------
    prev_angle : float
        Previous yaw angle in degrees.
    delT : float
        Elapsed time in seconds.
    gyro : float
        Gyroscope yaw rate.
    accelX : float
        X-axis acceleration.
    accelY : float
        Y-axis acceleration.
    accelZ : float
        Z-axis acceleration.
    magX : float
        X-axis magnetic field value.
    magY : float
        Y-axis magnetic field value.
    magZ : float
        Z-axis magnetic field value.
    weight : float
        Weight applied to the gyroscope estimate.

    Returns
    -------
    float
        Filtered yaw angle in degrees.
    """

    yaw = weight*(yaw_gy(prev_angle,delT,gyro)) + (1-weight)*(yaw_am(accelX,accelY,accelZ,magX,magY,magZ))
    return np.mod(yaw,360)
