import time

class Clock(object):
    def __init__(self):
        """initialize the robot's clock"""
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
        updates the raw time of the clock,\n
        which is the time since the Epoch,\n
        or Jan 1st 1970.
        """
        self.__raw_time = time.time()

    def __update_time(self):
        """
        updates the current time and the run time,\n
        where the current time is the time since the\n
        robot\'s raw start time, and the run time is\n
        the time since the robot was last reset.
        """
        self.__current_time = self.__raw_time - self.__raw_start_time
        self.__run_time = self.__raw_time - self.__start_time
    
    def __get_current_time(self):
        """
        returns the time since the robot\'s raw start time,\n
        this is not affected by resets.
        """
        return self.__current_time
    
    def __get_raw_time(self):
        "returns the time since the Epoch (Jan 1st 1970)"
        return(self.__raw_time)
    
    def get_time_of_last_reset(self):
        """
        returns the raw time at which the robot was last reset
        """
        return(self.__start_time)
    
    def get_raw_start_time(self):
        """
        returns the raw time of when the robot first started, 
        not including resets"""
        return self.__raw_start_time
    
    def __get_run_time(self):
        """returns the time since the robot was last reset"""
        return self.__run_time
    
    def update(self):
        """updates all time variables"""
        self.__update_raw_time()
        self.__update_time()

    def reset(self):
        """
        resets the run_time to 0 seconds,\n
        and sets the current start_time to be the current raw_time
        """
        self.__update_raw_time()
        self.__start_time = self.__raw_time
        self.__run_time = 0

    def get_time(self, timeType:str="run"):
        """
        gets the time, with an optional parameter to specify \"raw\", \"current\", or \"run\" time. The default is \"run\"\n
        run time => the time since the last reset.\n
        current time => the time since the robot\'s clock was initialized.\n
        raw time => the time since the Epoch on 1/1/1970.\n
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



