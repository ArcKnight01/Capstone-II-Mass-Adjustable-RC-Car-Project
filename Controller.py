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

# How often to update the BNO sensor data (in hertz).
BNO_UPDATE_FREQUENCY_HZ = 10

class Controller(object):
    CSV_HEADERS = [
        'time',
        'runtime',
        'loop_dt',
        'receiver_pulse_width',
        'receiver_angle',
        'battery_percent',
        'battery_seconds_left',
        'battery_time_remaining',
        'battery_plugged_in',
        'imu_temperature',
        'imu_raw_acceleration_x',
        'imu_raw_acceleration_y',
        'imu_raw_acceleration_z',
        'imu_acceleration_x',
        'imu_acceleration_y',
        'imu_acceleration_z',
        'imu_velocity_x',
        'imu_velocity_y',
        'imu_velocity_z',
        'imu_position_x',
        'imu_position_y',
        'imu_position_z',
        'imu_gravity_x',
        'imu_gravity_y',
        'imu_gravity_z',
        'imu_absolute_roll',
        'imu_absolute_pitch',
        'imu_absolute_yaw',
        'imu_relative_roll',
        'imu_relative_pitch',
        'imu_relative_yaw',
        'imu_initial_roll',
        'imu_initial_pitch',
        'imu_initial_yaw',
        'imu_angular_velocity_x',
        'imu_angular_velocity_y',
        'imu_angular_velocity_z',
        'imu_magnetic_x',
        'imu_magnetic_y',
        'imu_magnetic_z',
        'lat',
        'lon',
    ]

    def __init__(self,
                 verbose:bool=True, 
                 enabled:bool=True, 
                 log_to_csv:bool=True,
                 csv_data_dir_name : str = 'data',
                 csv_data_filename : str = 'data',
                 receiver : RCReceiver | None = None,
                 odometry : Odometry | None = None,
                 loop_delay : float = 1.000/BNO_UPDATE_FREQUENCY_HZ,
                 ):
        """
        Initialize the controller and connected subsystems.

        Parameters
        ----------
        verbose : bool, optional
            Whether verbose output should be enabled.
        enabled : bool, optional
            Whether the controller should be enabled.
        log_to_csv : bool, optional
            Whether controller data should be logged to CSV.
        csv_data_dir_name : str, optional
            Directory name used for CSV log output.
        csv_data_filename : str, optional
            Base filename used for CSV log output.
        receiver : RCReceiver | None, optional
            Receiver instance to use. If ``None``, a default receiver is created.
        odometry : Odometry | None, optional
            Odometry instance to use. If ``None``, a default odometry instance is created.
        loop_delay : float, optional
            Delay between controller loop iterations in seconds.

        Returns
        -------
        None
            This constructor initializes the controller state.
        """
        
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
        """
        Calibrate the controller odometry system.

        Parameters
        ----------
        None

        Returns
        -------
        None
            This method is currently a stub.
        """
        # TODO this is a stub, should we keep the calibration in the init function of this class?
        pass

    def run(self):
        """
        Run the controller update loop until interrupted.

        Parameters
        ----------
        None

        Returns
        -------
        None
            This method continuously calls :meth:`update`.
        """
        try: 
            while True: 
                self.update()
        except KeyboardInterrupt:
            print("controller stopped due to keyboard input.")


    def update(self):
        """
        Update all controller subsystems and log the current sample.

        Parameters
        ----------
        None

        Returns
        -------
        None
            This method updates battery, timing, receiver, odometry, CSV logging, and console output.
        """

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
        receiver_data_dict: dict[str, int] = self.receiver.get_data() or {'pulse_width': 0, 'angle': 0}

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
        odometry_data: dict[str, object] = self.odometry.get_data()
        row = self.build_log_row(receiver_data_dict, odometry_data)
        
        # log data
        self.add_to_csv(self.csv_data_filename, row)

        # print data
        print(" | ".join(
            f"{header}={value}" for header, value in zip(self.CSV_HEADERS, row)
        ))
        
        time.sleep(self.__loop_delay)
        
    def build_log_row(self, receiver_data_dict: dict[str, int], odometry_data: dict[str, object]) -> list[object]:
        """
        Build a CSV log row from receiver, battery, and odometry data.

        Parameters
        ----------
        receiver_data_dict : dict[str, int]
            Parsed receiver data containing the latest steering values.
        odometry_data : dict[str, object]
            Dictionary containing the latest odometry and IMU-derived values.

        Returns
        -------
        list[object]
            Ordered row values matching :attr:`CSV_HEADERS`.
        """
        battery_percent = self.safe_battery_value(self.__battery.retrieve_percentage())
        battery_seconds_left = self.safe_battery_value(self.__battery.retrieve_seconds_left())
        battery_time_remaining = self.safe_battery_value(self.__battery.get_time_remaining())
        battery_plugged_in = self.safe_battery_value(self.__battery.get_plugged_in())

        raw_acceleration = self.ensure_vector(odometry_data.get('raw_acceleration'))
        acceleration = self.ensure_vector(odometry_data.get('acceleration'))
        velocity = self.ensure_vector(odometry_data.get('velocity'))
        position = self.ensure_vector(odometry_data.get('position'))
        gravity = self.ensure_vector(odometry_data.get('gravity'))
        absolute_orientation = self.ensure_vector(odometry_data.get('absolute_orientation'))
        relative_orientation = self.ensure_vector(odometry_data.get('relative_orientation'))
        initial_orientation = self.ensure_vector(odometry_data.get('initial_orientation'))
        angular_velocity = self.ensure_vector(odometry_data.get('angular_velocity'))
        magnetic = self.ensure_vector(odometry_data.get('magnetic'))

        return [
            self.__time,
            self.__runtime,
            self.__delT,
            receiver_data_dict.get('pulse_width', 0),
            receiver_data_dict.get('angle', 0),
            battery_percent,
            battery_seconds_left,
            battery_time_remaining,
            battery_plugged_in,
            self.safe_battery_value(odometry_data.get('temperature')),
            raw_acceleration[0],
            raw_acceleration[1],
            raw_acceleration[2],
            acceleration[0],
            acceleration[1],
            acceleration[2],
            velocity[0],
            velocity[1],
            velocity[2],
            position[0],
            position[1],
            position[2],
            gravity[0],
            gravity[1],
            gravity[2],
            absolute_orientation[0],
            absolute_orientation[1],
            absolute_orientation[2],
            relative_orientation[0],
            relative_orientation[1],
            relative_orientation[2],
            initial_orientation[0],
            initial_orientation[1],
            initial_orientation[2],
            angular_velocity[0],
            angular_velocity[1],
            angular_velocity[2],
            magnetic[0],
            magnetic[1],
            magnetic[2],
            '',
            '',
        ]

    def ensure_vector(self, value: object, length: int = 3) -> tuple[object, ...]:
        """
        Normalize a vector-like value to a fixed-length tuple.

        Parameters
        ----------
        value : object
            Input value that may be a NumPy array, list, tuple, or another type.
        length : int, optional
            Desired tuple length.

        Returns
        -------
        tuple[object, ...]
            Tuple of the requested length, padded with empty strings when needed.
        """
        if isinstance(value, np.ndarray):
            value = value.tolist()
        if isinstance(value, (list, tuple)):
            normalized = list(value[:length])
            if len(normalized) < length:
                normalized.extend([''] * (length - len(normalized)))
            return tuple(normalized)
        return tuple([''] * length)

    def safe_battery_value(self, value: object) -> object:
        """
        Convert ``None`` battery values to an empty string for logging.

        Parameters
        ----------
        value : object
            Battery-derived value to normalize.

        Returns
        -------
        object
            The original value, or an empty string if the value is ``None``.
        """
        return '' if value is None else value


    def init_csv(self, filename:str):
        """
        Create or overwrite the CSV log file and write the header row.

        Parameters
        ----------
        filename : str
            Base filename for the CSV log.

        Returns
        -------
        None
            This method writes the CSV header row to disk.
        """
        path = pathlib.Path(self.csv_data_dir, f"{filename}.csv")
        with open(path, 'w') as csvfile:
            data = csv.writer(csvfile, delimiter =',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            data.writerow(self.CSV_HEADERS)
    
    def add_to_csv(self, filename:str, row:list):
        """
        Append a single row of data to the CSV log file.

        Parameters
        ----------
        filename : str
            Base filename for the CSV log.
        row : list
            Row of values to append.

        Returns
        -------
        None
            This method appends the provided row to the CSV file.
        """
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
    

   
    
