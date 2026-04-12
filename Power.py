import psutil

# https://www.geeksforgeeks.org/python/python-script-to-show-laptop-battery-percentage/


# function returning time in hh:mm:ss
def convertTime(seconds):
    minutes, seconds = divmod(seconds, 60)
    hours, minutes = divmod(minutes, 60)
    return "%d:%02d:%02d" % (hours, minutes, seconds)

class Battery(object):
    def __init__(self, enable:bool=True, verbose:bool=False):
        self.__enable = enable
        self.__battery = psutil.sensors_battery()
        if(self.__battery == None):
            self.__enable = False
        
    def update(self):
        if(self.__enable):
            self.__time_left = self.__battery.secsleft
            self.__percent = self.__battery.percent
    
    def retrieve_percentage(self):
        if(self.__enable):
            return(self.__percent)
        
    def retrieve_seconds_left(self):
        if(self.__enable):
            return(self.__time_left)
    
    def get_time_remaining(self):
        return convertTime(self.retrieve_seconds_left())
    
    def get_plugged_in(self):
        return self.__battery.power_plugged
    
        
if __name__ == "__main__":
    battery = Battery()
    
    while True:
        battery.update()
        print(f"BATTERY PLUGGED IN? {battery.get_plugged_in()} || BATTERY PERCENTAGE: {battery.retrieve_percentage()}% ({battery.get_time_remaining()} remaining)")

