from gps import gps, WATCH_ENABLE, WATCH_NEWSTYLE

import numpy as np
import utm
import random
import time
import datetime
import pytz

#https://stackoverflow.com/questions/77998670/python-gpsd-client

from pynmea2 import pynmea2

def nmea_lat(lat_deg):
    val = np.abs(lat_deg)
    degval = np.floor(val)
    minflt = (val - degval)*60
    minval = np.floor(minflt)
    muval = int((minflt - minval)*1e6)
    return f"{int(degval):02d}{int(minval):02d}.{int(muval):06d}"

def nmea_lon(lon_deg):
    val = np.abs(lon_deg)
    degval = np.floor(val)
    minflt = (val - degval)*60
    minval = np.floor(minflt)
    muval = int((minflt - minval)*1e6)
    return f"{int(degval):03d}{int(minval):02d}.{int(muval):06d}"


gpsd = gps(mode=WATCH_ENABLE | WATCH_NEWSTYLE)

while True:
    report = gpsd.next()  # blocks until a new report is ready
    if report['class'] == 'TPV':
        latitude = getattr(report, 'lat', None)
        longitude = getattr(report, 'lon', None)
        speed = getattr(report, 'speed', None)      # m/s
        timestamp = getattr(report, 'time', None)   # ISO 8601
        if latitude and longitude:
            print(f"Lat={latitude:.6f}, Lon={longitude:.6f}, Speed={speed}, Time={timestamp}")


session = gps(mode=WATCH_ENABLE)
while 0 == session.read()
    if not hasattr(session, "data"):
        # no data yet
        continue;

    # Check data class for 'DEVICE' messages from gpsd.  If
    # we're expecting messages from multiple devices we should
    # inspect the message to determine which device
    # has just become available.  But if we're just listening
    # to a single device, this may do.
    if session.data['class'] == 'DEVICE':
        # Clean up our current connection.
        session.close()
        # Tell gpsd we're ready to receive messages.
        session = gps(mode=WATCH_ENABLE)
    # Do more stuff

print "GPSD has terminated"

#! /usr/bin/env python3
"""
example  Python gpsd client
run this way: python3 example1.py.txt
"""

import gps               # the gpsd interface module

session = gps.gps(mode=gps.WATCH_ENABLE)

try:
    while 0 == session.read():
        if not (gps.MODE_SET & session.valid):
            # not useful, probably not a TPV message
            continue

        print('Mode: %s(%d) Time: ' %
              (("Invalid", "NO_FIX", "2D", "3D")[session.fix.mode],
               session.fix.mode), end="")
        # print time, if we have it
        if gps.TIME_SET & session.valid:
            print(session.fix.time, end="")
        else:
            print('n/a', end="")

        if ((gps.isfinite(session.fix.latitude) and
             gps.isfinite(session.fix.longitude))):
            print(" Lat %.6f Lon %.6f" %
                  (session.fix.latitude, session.fix.longitude))
        else:
            print(" Lat n/a Lon n/a")

except KeyboardInterrupt:
    # got a ^C.  Say bye, bye
    print('')

# Got ^C, or fell out of the loop.  Cleanup, and leave.
session.close()
exit(0)