
from gpiozero import RGBLED

class RGB_Indicator(object):
    def __init__(self, enabled:bool=True, verbose:bool=False, pins:tuple=None,red_pin:int=None, green_pin:int=None, blue_pin:int=None, pwm:bool=True, active_high:bool=True, initial_color:tuple=(255,0,0)):
        """
        Initialize the RGB indicator LED wrapper.

        Parameters
        ----------
        enabled : bool, optional
            Whether the RGB indicator should be enabled.
        verbose : bool, optional
            Whether verbose output should be enabled.
        pins : tuple, optional
            Combined tuple of pin identifiers.
        red_pin : int, optional
            Red channel pin.
        green_pin : int, optional
            Green channel pin.
        blue_pin : int, optional
            Blue channel pin.
        pwm : bool, optional
            Whether PWM should be enabled.
        active_high : bool, optional
            Whether the LED is active high.
        initial_color : tuple, optional
            Initial RGB color tuple.

        Returns
        -------
        None
            This constructor initializes the RGB indicator.
        """
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
        Set the color of the RGB LED device.

        Parameters
        ----------
        r : int, optional
            Red channel value.
        g : int, optional
            Green channel value.
        b : int, optional
            Blue channel value.
        color : tuple, optional
            RGB tuple to apply directly.
        default_color : tuple, optional
            Fallback RGB tuple used when no color is provided.

        Returns
        -------
        None
            This method updates the stored color and the LED output.
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
        
    def __rgb_to_unit(self, val: int | float) -> float:
        """
        Convert an 8-bit RGB channel value to a unit interval value.

        Parameters
        ----------
        val : int | float
            RGB channel value in the range ``0`` to ``255``.

        Returns
        -------
        float
            Normalized channel value in the range ``0.0`` to ``1.0``.
        """
        return val/255
    
    
