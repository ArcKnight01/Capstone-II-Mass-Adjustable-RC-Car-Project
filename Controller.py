import sys
import os
try:
    robotSupported = os.uname().nodename == ('terminatorpi' or 'robotpi' or 'carpi')
except:
    import platform
    robotSupported = platform.uname().node == ('terminatorpi' or 'robotpi' or 'carpi')

if robotSupported:
    import board
    import busio
    import adafruit_bno055

import pandas as pd
import numpy as np
import csv
import pathlib
import time
from Odometry import *
from RCReceiverNano import RCReceiver
from RobotClock import Clock
from Power import Battery

class Controller(object):
    def __init__(self,
                 verbose:bool=True, 
                 enabled:bool=True, 
                 log_to_csv:bool=True,
                 csv_data_dir_name : str = 'data',
                 csv_data_filename : str = 'data',
                 receiver : RCReceiver | None = None,
                 odometry : Odometry | None = None,
                 loop_delay : float = 1.000,
                 ):
        
        log_dir = './'
        self.csv_data_filename = csv_data_filename

        # set the csv data directory. Make and create the directory if it doesnt exist
        self.csv_data_dir = pathlib.Path(log_dir, csv_data_dir_name)
        self.csv_data_dir.mkdir(parents=True, exist_ok=True)
        self.init_csv(self.csv_data_filename)

        self.__battery = Battery()

        self.__clock = Clock()

        self.__tickTimer = Clock()

        self.__tickTimer.reset()
        self.__tickTimer.update()

        # This is used for odometry -- we need the time between the current odometry info at t and the previous odometry info at t-1)
        self.__delT = self.__tickTimer.get_time("run")

        self.__clock.reset()
        self.__clock.update()

        # This is used for time based control logic (not implemented yet), such as doing something after 5 seconds since reset. (could possibly be used for logging later too)
        self.__runtime = self.__clock.get_time("run")

        # This is used for logging.
        self.__time = self.__clock.get_time("current")

        self.__loop_delay = loop_delay

        # Build odometry first because IMU calibration can take a while.
        self.odometry = odometry if odometry is not None else Odometry()

        # Start or flush the receiver only after calibration so old Nano data
        # does not survive into the main control loop.
        self.receiver = receiver if receiver is not None else RCReceiver()
        self.receiver.reset()

        # Start loop timing only after startup work such as calibration has finished.
        self.__tickTimer.reset()
        self.__clock.reset()
        self.__clock.update()
        self.__runtime = self.__clock.get_time("run")
        self.__time = self.__clock.get_time("current")
       
        print("Initialized Controller")
    
    # calibrate the odometry system
    def calibrate():
        # TODO this is a stub, should we keep the calibration in the init function of this class?
        pass

    def run(self):
        try: 
            while True: 
                self.update()
        except KeyboardInterrupt:
            print("controller stopped due to keyboard input.")


    def update(self):

        # update the battery
        self.__battery.update()
        
        # Measure elapsed time since the previous loop for odometry integration.
        self.__tickTimer.update()

        # This is the elapsed time between the previous and current odometry samples.
        self.__delT = self.__tickTimer.get_time("run")
        if self.__delT <= 0:
            self.__delT = self.__loop_delay

        # reset the tick timer to 0, so that we can get the elapsed time in the next loop
        self.__tickTimer.reset()

        # Update controller timestamps used for logging and future control logic.
        self.__clock.update()

        # get the run time 
        self.__runtime = self.__clock.get_time("run")

        # get the time for logging to the csv
        self.__time = self.__clock.get_time("current")

        # read from rc receiver
        receiver_data_dict : dict[str, int] = self.receiver.get_data()  or { 'pulse_width': 0, 'angle': 0 }

        # return {
        # 'pulse_width': int(fields[1]),
        # 'angle': int(fields[2])
        # }

        # update odometry

        self.odometry.update(dt=self.__delT)

        #  return {
        #     'temperature' : self.__temperature,
        #     'raw_acceleration' : self.__raw_acceleration,
        #     'acceleration' : self.__acceleration,
        #     'velocity' : self.__velocity,
        #     'position' : self.__position,
        #     'gravity' : self.__gravity,
        #     'absolute_orientation' : self.__absolute_orientation,
        #     'relative_orientation' : self.__relative_orientation,
        #     'initial_orientation' : self.__initial_orientation,
        #     'angular_velocity' : self.__gyro,
        #     'magnetic' : self.__magnetometer
        # }

        # read odometry data 
        odometry_data : dict[str, int] = self.odometry.get_data()
        row = [self.__time, receiver_data_dict['angle'], odometry_data['acceleration'][0], odometry_data['acceleration'][1],odometry_data['acceleration'][2],odometry_data['absolute_orientation'][0],odometry_data['absolute_orientation'][1],odometry_data['absolute_orientation'][2],odometry_data['angular_velocity'][0],odometry_data['angular_velocity'][1],odometry_data['angular_velocity'][2],"",""]
        
        # log data
        self.add_to_csv(self.csv_data_filename, row)

        # print data
        print(
            f"t={self.__time:.3f} | "
            f"steer={receiver_data_dict['angle']} deg | "
            f"a=({odometry_data['acceleration'][0]:.3f}, "
            f"{odometry_data['acceleration'][1]:.3f}, "
            f"{odometry_data['acceleration'][2]:.3f}) m/s^2 | "
            f"rpy=({odometry_data['absolute_orientation'][0]:.3f}, "
            f"{odometry_data['absolute_orientation'][1]:.3f}, "
            f"{odometry_data['absolute_orientation'][2]:.3f}) deg | "
            f"w=({odometry_data['angular_velocity'][0]:.3f}, "
            f"{odometry_data['angular_velocity'][1]:.3f}, "
            f"{odometry_data['angular_velocity'][2]:.3f}) rad/s"
        )
        
        time.sleep(self.__loop_delay)
        


    def init_csv(self, filename:str):
        path = pathlib.Path(self.csv_data_dir, f"{filename}.csv")
        with open(path, 'w') as csvfile:
            data = csv.writer(csvfile, delimiter =',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            headers = ['time','steering angle', 'acceleration_x', 'acceleration_y', 'acceleration_z', 'roll', 'pitch', 'yaw', 'omega_x', 'omega_y', 'omega_z','lat','lon']
            data.writerow(headers)
    
    def add_to_csv(self, filename:str, row:list):
        path = pathlib.Path(self.csv_data_dir, f"{filename}.csv")
        self.__data = row
        with open(path, 'a') as csvfile:
            data = csv.writer(csvfile, delimiter =',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            data.writerow(self.__data)
        self.__data = []
        pass

    # def save(self, filename:str):
    #     """ Save the data to a file. """
    #     assert filename[-4:] == '.csv' or '.log', f"Custom Error: Please input a filename ending with \'.csv\'."
    #     assert self.csv_data_dir.exists() & self.csv_data_dir.is_dir(), f"Custom Error: Directory /{self.csv_data_dir} does not exist!"
    #     #pd.DataFrame(dict(zip(runs[0], runs[1])),colums=['times','accelerations']).to_csv(pathlib.Path(self.csv_data, filename), sep = ',')
    #     pass



if __name__ == '__main__':

    args = sys.argv[1:]
    num_args = len(args)
    if(args != list()):
        if num_args > 1:
            filename = str(args[0])
            verbose=(True if str(args[1])=='True' else False)
        else:
            filename = str(args[0])
    else:
        verbose = True
        filename = "data"
        pass

    controller : Controller = Controller(verbose=verbose, enabled=True, log_to_csv=True, csv_data_dir_name="data", csv_data_filename = filename)

    controller.run()
    

   
    
