
class Sensor(object):
    """
    Base class for sensors that all sensors inherit from,
    contains the variables verbose (whether the sensor prints data to the terminal,
    and enable (whether the robot runs the sensor and initializes it))
    """
    def __init__(self, verbose:bool=False, enabled:bool=True):
        self.__verbose=verbose
        self.__enabled=enabled
    def __repr__(self):
        return(f'Sensor Object Enabled[{self.__enabled}] Verbose[{self.__verbose}]')
    