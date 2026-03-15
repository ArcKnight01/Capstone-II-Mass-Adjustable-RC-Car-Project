import psutil
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
        
        
