
from gpiozero import RGBLED

class RGB_Indicator(object):
    def __init__(self, enabled:bool=True, verbose:bool=False, pins:tuple=None,red_pin:int=None, green_pin:int=None, blue_pin:int=None, pwm:bool=True, active_high:bool=True, initial_color:tuple=(255,0,0)):
        self.__enabled = enabled
        self.__verbose = verbose
        if(self.__enabled):
            assert ((red_pin and green_pin and blue_pin) or pins) and not ((red_pin or green_pin or blue_pin) and pins), ValueError
            self.__rgbLED = RGBLED(red_pin,green_pin,blue_pin, active_high=True, pwm=True, initial_value=self.__unit_color)
        else: 
            self.__rgbLED = None

        self.__color = initial_color
        self.__unit_color = map(self.__rgb_to_unit, self.__color)

    def set_color(self, r:int=None, g:int=None, b:int=None, color:tuple=None, default_color:tuple=(255,0,0)):
        """
        Set the color of the RGB LED device
        """
        assert ((r and g and b) or color) and not ((r or g or b) and color), ValueError
        #depending on whether r,g,b values are passed individually or as a tuple, set the
        if(r and g and b):
            self.__color = (r,g,b)
        elif(color):
            self.__color = color
        else:
            self.__color = default_color
        
        #convert the color in rgb format to unit format 
        self.__unit_color = map(self.rgb_to_unit,self.__color)

        if(self.__enabled):
            self.__rgbLED.color = self.__unit_color
        
    def __rgb_to_unit(self, val):
                return val/255
    
    