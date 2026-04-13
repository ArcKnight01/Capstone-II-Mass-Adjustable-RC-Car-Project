
"""
RGB_Indicator.py - Hardware wrapper for an RGB status LED.

This module exposes a small wrapper around the gpiozero RGBLED class for
Raspberry Pi-based status indication. It is safe to import on non-Pi systems,
but will disable the LED functionality if gpiozero is unavailable.
"""

try:
    from gpiozero import RGBLED
except ImportError:  # pragma: no cover
    RGBLED = None


class RGB_Indicator(object):
    def __init__(self, enabled: bool = False, verbose: bool = False, pins: tuple = None, red_pin: int = None, green_pin: int = None, blue_pin: int = None, pwm: bool = True, active_high: bool = True, initial_color: tuple = (255, 0, 0)):
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
        self.__color = initial_color
        self.__unit_color = tuple(self.__rgb_to_unit(c) for c in self.__color)

        if self.__enabled and RGBLED is not None:
            if pins is not None:
                self.__rgbLED = RGBLED(*pins, active_high=active_high, pwm=pwm, initial_value=self.__unit_color)
            else:
                assert red_pin is not None and green_pin is not None and blue_pin is not None, ValueError(
                    "Either pins or all three RGB pins must be provided"
                )
                self.__rgbLED = RGBLED(red_pin, green_pin, blue_pin, active_high=active_high, pwm=pwm, initial_value=self.__unit_color)
        else:
            self.__rgbLED = None

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
        if color is not None:
            self.__color = color
        elif r is not None and g is not None and b is not None:
            self.__color = (r, g, b)
        else:
            self.__color = default_color

        self.__unit_color = tuple(self.__rgb_to_unit(c) for c in self.__color)

        if self.__enabled and self.__rgbLED is not None:
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
    
    
