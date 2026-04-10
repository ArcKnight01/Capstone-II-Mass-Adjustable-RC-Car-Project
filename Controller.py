import pandas as pd
import numpy as np
import csv
import pathlib
import Odometry
import RCReceiver

class Controller(object):
    def __init__(self,
                 imu : Odometry,
                 receiver : RCReceiver,
                 csv_data_dir_name : str = 'data',
                 ):
        log_dir = './'
        # set the csv data directory. Make and create the directory if it doesnt exist
        self.csv_data_dir = pathlib.Path(log_dir, 'data')
        self.csv_data_dir.mkdir(parents=True, exist_ok=True)
        self.data = {"time":list(), "steering_angle":list(), "imu_data":list()}
        self.imu = Odometry()
        print("Initialized Controller")
        pass

    def test(self):

        pass
    def setup(self):

        pass

    def calibrate(self):

        pass
    
    def run(self):
        self.calibrate()
        while True: 
            self.update()

    def update(self):
        # get the time

        # read from rc receiver

        # read imu data (odometry)

        # log data
        
        pass

    def save(self, filename:str):
        """ OPTIONAL FOR A FIVE: Save the data to a file. """
        assert filename[-4:] == '.csv' or '.log', f"Custom Error: Please input a filename ending with \'.csv\'."
        assert self.csv_data_dir.exists() & self.csv_data_dir.is_dir(), f"Custom Error: Directory /{self.csv_data_dir} does not exist!"
        #pd.DataFrame(dict(zip(runs[0], runs[1])),colums=['times','accelerations']).to_csv(pathlib.Path(self.csv_data, filename), sep = ',')
        pass

    def log_data(self):
        self.data["time"].append()
        self.data["imu_data"].append()
        pass