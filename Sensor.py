
"""
Sensor.py - Base sensor abstraction.

This module defines a small base class for sensors and sensor-like subsystems,
carrying common flags for verbosity and enabled state.
"""

class Sensor(object):
    """
    Base class for sensors that all sensors inherit from,
    contains the variables verbose (whether the sensor prints data to the terminal,
    and enable (whether the robot runs the sensor and initializes it))
    """
    def __init__(self, verbose:bool=False, enabled:bool=True):
        """
        Initialize the base sensor state.

        Parameters
        ----------
        verbose : bool, optional
            Whether sensor output should be printed to the terminal.
        enabled : bool, optional
            Whether the sensor should be enabled.

        Returns
        -------
        None
            This constructor stores the base sensor flags.
        """
        self.__verbose=verbose
        self.__enabled=enabled

    def __repr__(self) -> str:
        """
        Return a string representation of the sensor.

        Parameters
        ----------
        None

        Returns
        -------
        str
            Human-readable sensor state string.
        """
        return(f'Sensor Object Enabled[{self.__enabled}] Verbose[{self.__verbose}]')
    
