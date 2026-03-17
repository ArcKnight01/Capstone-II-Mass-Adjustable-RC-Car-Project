from gps import gps, WATCH_ENABLE, WATCH_NEWSTYLE
import gpsd
import numpy as np
import utm
import random
import time
import datetime
import pytz



from pynmea2 import pynmea2
class GPS_System:

    def __init__(self):
        self.__session = gps(mode= )
        pass

    def run_session(self):
        while 0 == self.__session.read()
            if not hasattr(self.__session, "data"):
                continue
