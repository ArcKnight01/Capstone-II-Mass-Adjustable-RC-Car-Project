from gpiozero import Servo

class ServoMotor(object):
    def __init__(self, enabled:bool=True, verbose:bool=False, pin:int=9, initial_degree:float=0.0):
        """
        Initialize the servo motor wrapper.

        Parameters
        ----------
        enabled : bool, optional
            Whether the servo motor should be enabled.
        verbose : bool, optional
            Whether verbose output should be enabled.
        pin : int, optional
            GPIO pin connected to the servo.
        initial_degree : float, optional
            Initial servo angle in degrees.

        Returns
        -------
        None
            This constructor initializes servo state.
        """
        self.__pin = pin
        self.__degree = self.__initial_degree = initial_degree
        self.__value = self.__initial_value = self.__initial_degree/180.0

        self.__servoMotor = Servo(self.__pin, initial_value=self.__value)
        pass

    def rotateToDegree(self, degree:float=0):
        """
        Rotate the servo to a requested angle.

        Parameters
        ----------
        degree : float, optional
            Requested servo angle in degrees.

        Returns
        -------
        None
            This method commands the servo to move to the requested angle.
        """
        #constrain degree between -180 and 180 where 0 is the midpoint, scale this to -1 to 1 where 0 is the midpoint
        self.__degree = degree
        if(degree > 180):
            self.__servoMotor.max()
        if degree < -180:
            self.__servoMotor.min()
        if degree == 0:
            self.__servoMotor.mid()
        else:
            self.__value = self.__degree/180.0 
            self.__servoMotor.value = self.__value
    
    def get_degree(self) -> float:
        """
        Retrieve the current servo angle.

        Parameters
        ----------
        None

        Returns
        -------
        float
            Current servo angle in degrees.
        """
        return(self.__degree)
    
    def resetToInitial(self) -> None:
        """
        Reset the servo to its initial angle.

        Parameters
        ----------
        None

        Returns
        -------
        None
            This method rotates the servo back to its initial degree.
        """
        self.rotateToDegree(self.__initial_degree)

    
    




