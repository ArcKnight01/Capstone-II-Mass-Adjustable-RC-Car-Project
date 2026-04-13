"""
Power.py - Battery and power monitoring utilities.

This module wraps psutil battery information for the RC car control system.
It exposes battery percentage, remaining time, and charger status.

Usage:
    python Power.py
"""

import psutil

# https://www.geeksforgeeks.org/python/python-script-to-show-laptop-battery-percentage/


# function returning time in hh:mm:ss
def convertTime(seconds: int | float) -> str:
    """
    Convert a duration in seconds to ``hh:mm:ss`` format.

    Parameters
    ----------
    seconds : int | float
        Duration in seconds.

    Returns
    -------
    str
        Formatted time string.
    """
    minutes, seconds = divmod(seconds, 60)
    hours, minutes = divmod(minutes, 60)
    return "%d:%02d:%02d" % (hours, minutes, seconds)

class Battery(object):
    def __init__(self, enable:bool=True, verbose:bool=False):
        """
        Initialize the battery monitor.

        Parameters
        ----------
        enable : bool, optional
            Whether battery monitoring should be enabled.
        verbose : bool, optional
            Whether verbose output should be enabled.

        Returns
        -------
        None
            This constructor initializes battery state.
        """
        self.__enable = enable
        self.__battery = psutil.sensors_battery()
        if(self.__battery == None):
            self.__enable = False
        
    def update(self):
        """
        Refresh cached battery values from ``psutil``.

        Parameters
        ----------
        None

        Returns
        -------
        None
            This method updates the cached battery time and percentage.
        """
        if(self.__enable):
            self.__time_left = self.__battery.secsleft
            self.__percent = self.__battery.percent
    
    def retrieve_percentage(self):
        """
        Retrieve the current battery percentage.

        Parameters
        ----------
        None

        Returns
        -------
        float | None
            Current battery percentage, or ``None`` when unavailable.
        """
        if(self.__enable):
            return(self.__percent)
        
    def retrieve_seconds_left(self):
        """
        Retrieve the estimated battery time remaining in seconds.

        Parameters
        ----------
        None

        Returns
        -------
        int | None
            Remaining battery time in seconds, or ``None`` when unavailable.
        """
        if(self.__enable):
            return(self.__time_left)
    
    def get_time_remaining(self):
        """
        Retrieve the estimated battery time remaining as text.

        Parameters
        ----------
        None

        Returns
        -------
        str
            Remaining battery time formatted as ``hh:mm:ss``, or ``"N/A"`` when unavailable.
        """
        seconds = self.retrieve_seconds_left()
        if seconds is None or seconds < 0:
            return "N/A"
        return convertTime(seconds)
    
    def get_plugged_in(self):
        """
        Check whether external power is connected.

        Parameters
        ----------
        None

        Returns
        -------
        bool | None
            ``True`` if external power is connected, otherwise ``False``.
        """
        if not self.__enable or self.__battery is None:
            return False
        return self.__battery.power_plugged
    
        
if __name__ == "__main__":
    battery = Battery()
    
    while True:
        battery.update()
        print(f"BATTERY PLUGGED IN? {battery.get_plugged_in()} || BATTERY PERCENTAGE: {battery.retrieve_percentage()}% ({battery.get_time_remaining()} remaining)")

