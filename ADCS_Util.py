import numpy as np

def roll_am(accelX,accelY,accelZ):
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


def pitch_am(accelX,accelY,accelZ):
    if accelZ > 0:
        sign = 1
    else:
        sign = -1
    pitch = (180/np.pi)*np.arctan2(accelX,sign*(accelY**2+accelZ**2)**0.5) 
    # pitch function using arctan2 - four quadrant answer
    return pitch # 


def yaw_am(accelX,accelY,accelZ,magX,magY,magZ):
    #print(magX,magY,magZ)
    pitchR = (np.pi/180)*pitch_am(accelX,accelY,accelZ)
    rollR = (np.pi/180)*roll_am(accelX,accelY,accelZ) # change to radians for np functions to work
    magx = magX*np.cos(pitchR) + magY*np.sin(rollR)*np.sin(pitchR) + magZ*np.cos(rollR)*np.sin(pitchR)
    magy = magY*np.cos(rollR) - magZ*np.sin(rollR)
    yawR = np.arctan2(-magy,magx) # returning from - pi to pi for some reason (shift to 0-pi)
    return ((180/np.pi)*yawR*2 + 360) % 360  # changing to degrees at the end and moving to 0 to pi

###Gyro Sensor outputs in rad/s, so to calculate the RPY, you want to take the previous value (rads) and add the gyro (rad/s) * the time in number of seconds (s)
def roll_gy(prev_angle, delT, gyro):
    roll = prev_angle + gyro*delT
    return np.mod(roll,360)

def pitch_gy(prev_angle, delT, gyro):
    pitch = prev_angle + gyro*delT
    return np.mod(pitch,360)
    
def yaw_gy(prev_angle, delT, gyro):
    yaw = prev_angle + gyro*delT
    return np.mod(yaw,360)

# SENSOR FUSION
def roll_F(prev_angle, delT, gyro,accelX,accelY,accelZ,weight):
    """
    Param: prev_angle, delT, gyro,accelX,accelY,accelZ,weight
    """

    roll = weight*(roll_gy(prev_angle,delT,gyro)) + (1-weight)*(roll_am(accelX, accelY, accelZ))
    return np.mod(roll,360)

def pitch_F(prev_angle, delT, gyro,accelX,accelY,accelZ,weight):
    """
    Param: prev_angle, delT, gyro,accelX,accelY,accelZ,weight
    """
    
    pitch = weight*(pitch_gy(prev_angle,delT,gyro)) + (1-weight)*(pitch_am(accelX,accelY,accelZ))
    return np.mod(pitch,360)

def yaw_F(prev_angle, delT, gyro,accelX,accelY,accelZ,magX,magY,magZ,weight):
    """
    Param: prev_angle, delT, gyro,accelX,accelY,accelZ,magX,magY,magZ,weight
    """

    yaw = weight*(yaw_gy(prev_angle,delT,gyro)) + (1-weight)*(yaw_am(accelX,accelY,accelZ,magX,magY,magZ))
    return np.mod(yaw,360)
