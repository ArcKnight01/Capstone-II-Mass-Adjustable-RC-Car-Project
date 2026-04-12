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

import time
from IMUUtil import *
import csv
from RobotClock import Clock
import numpy as np 
# How often to update the BNO sensor data (in hertz).
BNO_UPDATE_FREQUENCY_HZ = 10


class OdometryOLD(object):
    def __init__(self, test_points:int=10, 
                 verbose:bool=False, 
                 enabled:bool=True, 
                 manual:bool=False,
                 uses_alternate_i2c_addr:bool=False,
                 ):
        #Set number of test points for calibration
        self.__test_points = test_points
        #Determine whether the ADCS System will print testing data to terminal
        self.__verbose = verbose
        #Set whether robot is enabled
        self.__enabled = enabled
        self.__manual = manual
        if uses_alternate_i2c_addr:
            self.__i2c_addr = 0x29
        else:
            self.__i2c_addr = 0x28
        #Declare the sensor device
        self.__i2c = busio.I2C(board.SCL, board.SDA)
        self.__sensor = adafruit_bno055.BNO055_I2C(self.__i2c, address=self.__i2c_addr)
        
        #initialize the clock.
        self.__clock = Clock()
        
        #Set initial values for acceleration, velocity, and position to be (0,0,0)
        self.__acceleration = (0,0,0)
        self.__previous_acceleration = (0,0,0)

        self.__velocity = (0,0,0)
        self.__previous_velocity = (0,0,0)

        self.__position = (0,0,0)
        self.__previous_position = (0,0,0)
        
        #calibrate accelerometer and get offset values
        self.__accelerometer_offset = self.calibrate_accelerometer()

        #calibrate magnetometer and get offset values
        self.__mag_offset = self.calibrate_mag()

        #calibrate gyroscope and get offset values
        self.__gyro_offset = self.calibrate_gyro()
        
        self.__previous_orientation = self.__orientation = self.__initial_orientation = self.set_initial(self.__mag_offset)
        self.__orientation_zeroed = self.zero_orientation()

        #initialize the csv-data file
        self.init_csv()
        self.__tickTimer = Clock()
        self.__tickTimer.reset()
        self.__tickTimer.update()
        self.__delT = self.__tickTimer.get_time("run")
        self.__clock.reset()
        self.__clock.update()
        self.__runtime = self.__clock.get_time("run")
        self.__time = self.__clock.get_time("current")
        self.__last_val = 0xFFFF

    def temperature(self):
        global last_val  # noqa: PLW0603
        result = self.__sensor.temperature
        if abs(result - self.__last_val) == 128:
            result =self.__sensor.temperature
            if abs(result - self.__last_val) == 128:
                return 0b00111111 & result
        self.__last_val = result
        return result
    
    def calibrate(self):
        #calibrate accelerometer and get offset values
        self.__accelerometer_offset = self.calibrate_accelerometer()

        #calibrate magnetometer and get offset values
        self.__mag_offset = self.calibrate_mag()

        #calibrate gyroscope and get offset values
        self.__gyro_offset = self.calibrate_gyro()

        #set initial angle
        self.__previous_orientation = self.__orientation = self.__initial_orientation = self.set_initial(self.__mag_offset)

        self.__orientation_zeroed = self.zero_orientation()
        #zero the orientation to intitial orientation of robot
        
    
    def update(self):
        
        delT = 1/BNO_UPDATE_FREQUENCY_HZ
        
        self.__tickTimer.update()
        self.__delT = self.__tickTimer.get_time("run")
        
        self.__previous_acceleration = self.__acceleration
        self.__previous_velocity = self.__velocity
        self.__previous_position = self.__position
        self.__previous_orientation = self.__orientation

        self.__clock.update()
        self.__runtime = self.__clock.get_time("run")
        self.__time = self.__clock.get_time("current")


        ### DATA OUTPUT FROM BNO055###

        #ABSOLUTE ORIENTATION (EULER VECTOR 100 Hz) - Three axis orientation data based on a 360° sphere
        self.__euler = self.__sensor.euler

        #ABSOLUTE ORIENTATION (QUATERNION VECTOR, 100 HZ) - Four point quaternion output for more accurate data manipulation
        self.__quaternion = self.__sensor.quaternion

        ###LINEAR ACCELERATION VECTOR 100Hz Three axis of linear acceleration data (acceleration minus gravity) in m/s^2###
        self.__linear_acceleration = self.__sensor.linear_acceleration

        ###Three axis of gravitational acceleration (minus any movement) in m/s^2###
        self.__gravity = self.__sensor.gravity

        ###RAW###
        ###Acceleration Vector (100Hz) - Three axis of acceleration (gravity + linear motion) in m/s^2###
        self.__raw_acceleration = self.__sensor.acceleration #TODO: Check if this is actually raw accel

        ###Magnetic Field Strength Vector - Three axis of magnetic field sensing in micro Tesla (uT)
        self.__magnetometer = self.__sensor.magnetic

        ###Angular Velocity Vector / Gyroscope (100Hz) - Three axis of 'rotation speed' in rad/s###
        self.__gyro = self.__sensor.gyro

        self.__acceleration = self.__sensor.linear_acceleration

        #correct for offsets
        self.__magnetometer = (self.__magnetometer[0] - self.__mag_offset[0], 
                               self.__magnetometer[1] - self.__mag_offset[1], 
                               self.__magnetometer[2] - self.__mag_offset[2])
        
        self.__gyro = (self.__gyro[0] *180/np.pi - self.__gyro_offset[0],
                       self.__gyro[1] *180/np.pi - self.__gyro_offset[1],
                       self.__gyro[2] *180/np.pi - self.__gyro_offset[2])
        dt = delT
        self.__tickTimer.update()
        self.__delT = self.__tickTimer.get_time("run")
        self.__tickTimer.reset()
        dt = 1
        
        # print(f"accels: {self.__acceleration[0]}")
        #When the robot is still, the accel values are near 0. In this case, set accel values to zero.
        
        # self.__acceleration = (self.__acceleration[0] - self.__accelerometer_offset[0],
        #                        self.__acceleration[1] - self.__accelerometer_offset[1],
        #                        self.__acceleration[2] - self.__accelerometer_offset[2])
        
        # self.__delta_acceleration = (self.__acceleration[0] - self.__previous_acceleration[0],
        #                              self.__acceleration[1] - self.__previous_acceleration[1],
        #                              self.__acceleration[2] - self.__previous_acceleration[2])

        # self.__velocity = (self.__velocity[0] + self.__delta_acceleration[0]*dt, 
        #                    self.__velocity[1] + self.__delta_acceleration[1]*dt, 
        #                    self.__velocity[2] + self.__delta_acceleration[2]*dt)
        
        # self.__delta_velocity = (self.__velocity[0] - self.__previous_velocity[0],
        #                          self.__velocity[1] - self.__previous_velocity[1],
        #                          self.__velocity[2] - self.__previous_velocity[2])
        # #update position
        # self.__position = (self.__position[0] + self.__delta_velocity[0]*dt,
        #                    self.__position[1] + self.__delta_velocity[1]*dt,
        #                    self.__position[2] + self.__delta_velocity[2]*dt)
        
        #get roll,pitch,yaw with acceleration-magnetic data
        # self.__roll_am = roll_am(self.__acceleration[0], self.__acceleration[1], self.__acceleration[2]) - self.__orientation_zeroed[0]
        # self.__pitch_am = pitch_am(self.__acceleration[0], self.__acceleration[1], self.__acceleration[2]) - self.__orientation_zeroed[1]
        # self.__yaw_am = yaw_am(self.__acceleration[0], self.__acceleration[1], self.__acceleration[2], self.__magnetometer[0], self.__magnetometer[1], self.__magnetometer[2]) - self.__orientation_zeroed[2]
    
        # #get roll,pitch,yaw with respect to gyro data
        # self.__roll_gy = roll_gy(self.__previous_orientation[0], dt, self.__gyro[0]) - self.__orientation_zeroed[0]
        # self.__pitch_gy = pitch_gy(self.__previous_orientation[1], dt, self.__gyro[1]) - self.__orientation_zeroed[1]
        # self.__yaw_gy = yaw_gy(self.__previous_orientation[2], dt, self.__gyro[2]) - self.__orientation_zeroed[2]

        # #get roll,pitch,yaw using fusion of am and gyro
        # self.__roll = roll_F(self.__previous_orientation[0], dt, self.__gyro[0], self.__acceleration[0],self.__acceleration[1], self.__acceleration[2], 0.5) - self.__orientation_zeroed[0]
        # self.__pitch = pitch_F(self.__previous_orientation[1], dt, self.__gyro[1], self.__acceleration[0],self.__acceleration[1], self.__acceleration[2], 0.5) - self.__orientation_zeroed[1]
        # self.__yaw = yaw_F(self.__previous_orientation[0], dt, self.__gyro[0], self.__acceleration[0],self.__acceleration[1], self.__acceleration[2], self.__magnetometer[0], self.__magnetometer[1], self.__magnetometer[2],0.5) - self.__orientation_zeroed[2]

        #set the orientation based on roll pitch and yaw values
        # self.__orientation = (self.__roll, self.__pitch, self.__yaw)
        
        
        if(self.__verbose):
            print(f"[INFO] Raw Acceleration {self.__raw_acceleration}")
            print(f"[INFO] Linear Acceleration {self.__linear_acceleration}")
            # print(f"[INFO] Acceleration {self.__acceleration}")
            # print(f"[INFO] Velocity {self.__velocity}")
            # print(f"[INFO] Position {self.__position}")
            print(f"[INFO] Magnetometer {self.__magnetometer}")
            print(f"[INFO] Gyroscope {self.__gyro}")
            print(f"[INFO] Euler orientation {self.__euler}")
            # print(f"[INFO] Quaternion {self.__quaternion}")
            print(f"[INFO] Gravity {self.__gravity}")
            # print(f"[INFO] RPY_GY {(round(self.__roll_gy,2), round(self.__pitch_gy,2), round(self.__yaw_gy,2))} (degrees)")
            # print(f"[INFO] RPY_AM {(round(self.__roll_am,2), round(self.__pitch_am,2), round(self.__yaw_am,2))} (degrees)")
            print(f"[INFO] RPY_F {self.__orientation}")

    def calibrate_accelerometer(self):
        calibration_pause = 1

        accelXList = [];
        accelYList = [];
        accelZList = [];
        
        print("[CALIBRATION] Preparing to calibrate accelerometer. Please hold still.")
        time.sleep(calibration_pause) # pause before calibrating (1s)
        print("[CALIBRATION] Calibrating...")
        numTestPoints = 0;
        while numTestPoints < self.__test_points:
            accelX, accelY, accelZ = self.__sensor.linear_acceleration
            if(self.__verbose):
                print(f"Acceleration (x,y,z) @ n={numTestPoints}: {(accelX, accelY, accelZ)}")
            accelXList.append(accelX)
            accelYList.append(accelY)
            accelZList.append(accelZ)
            numTestPoints += 1
            time.sleep(1)
        print("[CALIBRATION] Calibration complete.")
        # print(rollList)
        time.sleep(calibration_pause) # break after calibrating

        avgX = np.mean((np.min(accelXList),np.max(accelXList)))
        avgY = np.mean((np.min(accelYList), np.max(accelYList)))
        avgZ = np.mean((np.min(accelZList), np.max(accelZList)))
        
        accel_offsets = [avgX,avgY,avgZ]

        print(f"[CALIBRATION] Acceleration offsets: {accel_offsets}.")
        return accel_offsets
    
    def zero_orientation(self):
        self.__orientation_zeroed = self.__orientation
        self.__roll, self.__pitch, self.__yaw = self.__orientation 
        self.__roll = self.__roll - self.__orientation_zeroed[0]
        self.__pitch = self.__pitch - self.__orientation_zeroed[1]
        self.__yaw = self.__yaw - self.__orientation_zeroed[2]
        self.__orientation = (self.__roll, self.__pitch, self.__yaw)
        return(self.__orientation_zeroed)

    def set_initial(self, mag_offset = [0,0,0]):
        calibration_pause=.001
        accelX, accelY, accelZ =  self.__sensor.acceleration #m/s^2
        magX, magY, magZ = self.__sensor.magnetic #gauss

        #Sets the initial position for plotting and gyro calculations.
        print("[CALIBRATION] Preparing to set initial orientation. Please hold the IMU still.")
        time.sleep(calibration_pause)
        print("[CALIBRATION] Setting orientation...")
        
        #Calibrate magnetometer readings. Defaults to zero until you
        magX = magX - mag_offset[0]
        magY = magY - mag_offset[1]
        magZ = magZ - mag_offset[2]

        roll = roll_am(accelX, accelY,accelZ)
        pitch = pitch_am(accelX,accelY,accelZ)
        yaw = yaw_am(accelX,accelY,accelZ,magX,magY,magZ)

        print("[CALIBRATION] Initial orientation set.")
        print(f"[CALIBRATION] Initial Orientation (Roll,Pitch,Yaw): {[roll,pitch,yaw]}") # display the initial position
        return [roll,pitch,yaw]
    
    
    def calibrate_mag(self):
        calibration_pause = .001
        rollList = [];
        pitchList = [];
        yawList = [];
        print("Preparing to calibrate magnetometer. Please wave around.")
        time.sleep(calibration_pause) # pause before calibrating
        print("Calibrating...")
        numTestPoints = 0;
        while numTestPoints < self.__test_points:
            magX, magY, magZ = self.__sensor.magnetic
            if(self.__verbose):
                print(f"Mag(x,y,z)@{numTestPoints}: {(magX, magY, magZ)}")
            rollList.append(magX)
            pitchList.append(magY)
            yawList.append(magZ)
            numTestPoints += 1
            time.sleep(calibration_pause)
        print("Calibration complete.")
        # print(rollList)
        time.sleep(calibration_pause) # break after calibrating
        avgX = np.mean((np.min(rollList),np.max(rollList)))
        avgY = np.mean((np.min(pitchList), np.max(pitchList)))
        avgZ = np.mean((np.min(yawList), np.max(yawList)))
        calConstants = [avgX,avgY,avgZ]
        print(calConstants)
        return calConstants
        

    def calibrate_gyro(self):
        calibration_pause=.001
        rollList = [];
        pitchList = [];
        yawList = [];
        print("Preparing to calibrate gyroscope. Please hold still.")
        time.sleep(calibration_pause) # pause before calibrating
        print("Calibrating...")

        numTestPoints = 0;
        while numTestPoints < self.__test_points:
            gyroX, gyroY, gyroZ = self.__sensor.gyro
            if(self.__verbose):
                print(f"Gyro(x,y,z)@{numTestPoints}: {(gyroX, gyroY, gyroZ)}")
            rollList = rollList + [gyroX]
            pitchList = pitchList + [gyroY] 
            yawList = yawList + [gyroZ]
            numTestPoints += 1
            time.sleep(calibration_pause)
        print("Calibration complete.")
        
        time.sleep(calibration_pause) # break after calibrating
        avgX = np.mean((np.min(rollList),np.max(rollList)))
        avgY = np.mean((np.min(pitchList), np.max(pitchList)))
        avgZ = np.mean((np.min(yawList), np.max(yawList)))
        calConstants = [avgX,avgY,avgZ]
        print(calConstants)
        return calConstants
        #return [0, 0, 0]

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

    def get_data(self):
        return(self.__time, self.__raw_acceleration, self.__acceleration, self.__velocity, self.__position, self.__orientation)

    def init_csv(self):
        with open('./imu_data.csv', 'w') as csvfile:
            data = csv.writer(csvfile, delimiter =',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            data.writerow(['Time',
                           'AccelerationX','AccelerationY','AccelerationZ',
                           'RawAccelerationX', 'RawAccelerationY', 'RawAccelerationZ',
                           'VelocityX', 'VelocityY', 'VelocityZ',
                           'PositionX', 'PositionY', 'PositionZ',
                           'Roll', 'Pitch', 'Yaw',
                           'MagnetometerX', 'MagnetometerY', 'MagnetometerZ',
                           'GyroX', 'GyroY', 'GyroZ',
                           'GravityX', 'GravityY', 'GravityZ',
                           'EulerX', 'EulerY', 'EulerZ'

                           ])

    def add_to_csv(self):
        self.__data = [
            self.__time,
              self.__acceleration[0], self.__acceleration[1], self.__acceleration[2],
              self.__raw_acceleration[0], self.__raw_acceleration[1], self.__raw_acceleration[2],
              self.__velocity[0], self.__velocity[1], self.__velocity[2],
              self.__position[0], self.__position[1], self.__position[2],
              self.__orientation[0], self.__orientation[1], self.__orientation[2],
              self.__magnetometer[0], self.__magnetometer[1], self.__magnetometer[2],
              self.__gyro[0], self.__gyro[1], self.__gyro[2],
              self.__gravity[0], self.__gravity[1], self.__gravity[2],
              self.__euler[0], self.__euler[1], self.__euler[2]
        ]
        with open('./imu_data.csv', 'a') as csvfile:
            data = csv.writer(csvfile, delimiter =',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            data.writerow(self.__data)
        self.__data = []
        pass

if __name__ == '__main__':
    # print("Args passed: ", end='')
    # print([sys.argv[0]], end='|')
    # print(sys.argv[1:], end = '||')
    
    # print(len([*sys.argv[1:]] ))
    if(sys.argv[1:] != list()):
        print(f"test_points={sys.argv[1:][0]}, verbose={sys.argv[1:][1]}")
        # print("args passed!")
        # print(sys.argv[1:])
        imu = Odometry(test_points=int(sys.argv[1]), verbose=(True if str(sys.argv[2])=='True' else False))
    else:
        # print("no args passed!")
        imu = Odometry(test_points=10, verbose=False)
    while(True):
        imu.update()
        imu.add_to_csv()
        t, raw, accel, vel, pos, rpy = imu.get_data()
        # print(imu.get_data())
        # print(f"Raw:{(round(raw[1:][0],2), round(raw[1:][1],2))}|Accel:{(round(accel[1:][0],2),}|Vel:{vel}|Pos:{pos}|Rpy:{rpy}")

        pass

