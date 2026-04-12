import time

class Clock(object):
    def __init__(self):
        """
        Initialize the robot clock.

        Parameters
        ----------
        None

        Returns
        -------
        None
            This constructor initializes the raw, current, and run times.
        """
        #intialize the raw time, the time in seconds since the Epoch (Jan 1st 1970)
        self.__raw_time = time.time() 
        #initialize raw_start_time, the time in seconds since the Epoch, 
        #at which the robot was started, not including resets.
        self.__raw_start_time = self.__raw_time
        self.__start_time = self.__raw_start_time
        self.__current_time = 0
        self.__run_time = self.__current_time

        pass
    def __update_raw_time(self):
        """
        Update the raw epoch time stored by the clock.

        Parameters
        ----------
        None

        Returns
        -------
        None
            This method refreshes the internal raw time value.
        """
        self.__raw_time = time.time()

    def __update_time(self):
        """
        Update derived current and run times.

        Parameters
        ----------
        None

        Returns
        -------
        None
            This method updates the current time since startup and the run time since reset.
        """
        self.__current_time = self.__raw_time - self.__raw_start_time
        self.__run_time = self.__raw_time - self.__start_time
    
    def __get_current_time(self):
        """
        Retrieve the elapsed time since clock initialization.

        Parameters
        ----------
        None

        Returns
        -------
        float
            Current time since the raw start time.
        """
        return self.__current_time
    
    def __get_raw_time(self):
        """
        Retrieve the raw epoch time.

        Parameters
        ----------
        None

        Returns
        -------
        float
            Time since the Unix epoch in seconds.
        """
        return(self.__raw_time)
    
    def get_time_of_last_reset(self):
        """
        Retrieve the raw time of the most recent reset.

        Parameters
        ----------
        None

        Returns
        -------
        float
            Raw time at which the clock was last reset.
        """
        return(self.__start_time)
    
    def get_raw_start_time(self):
        """
        Retrieve the raw start time of the clock.

        Parameters
        ----------
        None

        Returns
        -------
        float
            Raw time when the clock was first initialized.
        """
        return self.__raw_start_time
    
    def __get_run_time(self):
        """
        Retrieve the elapsed time since the last reset.

        Parameters
        ----------
        None

        Returns
        -------
        float
            Run time in seconds since the last reset.
        """
        return self.__run_time
    
    def update(self):
        """
        Refresh all internal clock values.

        Parameters
        ----------
        None

        Returns
        -------
        None
            This method updates raw, current, and run times.
        """
        self.__update_raw_time()
        self.__update_time()

    def reset(self):
        """
        Reset the run timer.

        Parameters
        ----------
        None

        Returns
        -------
        None
            This method resets run time and updates the reset reference time.
        """
        self.__update_raw_time()
        self.__start_time = self.__raw_time
        self.__run_time = 0

    def get_time(self, timeType:str="run"):
        """
        Retrieve a requested clock time value.

        Parameters
        ----------
        timeType : str, optional
            Time type to retrieve. Supported values are ``"run"``, ``"current"``, and ``"raw"``.

        Returns
        -------
        float
            Requested time value in seconds.
        """
        assert timeType in ["run", "current", "raw"]
        if(timeType=="run"):
            return self.__get_run_time()
        elif(timeType=="current"):
            return self.__get_current_time()
        elif(timeType=="raw"):
            return self.__get_raw_time()
        else:
            return -1



