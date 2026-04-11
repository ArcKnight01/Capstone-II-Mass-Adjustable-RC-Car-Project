import serial
import sys

def checkthesum(msg :str):
    """Your exact checksum logic from the backseat driver."""
    try:
        fields = msg.strip().split('*')
        cmd = fields[0][1:]  # strip leading $
        expected = hex(nmea_checksum(cmd))[2:].upper()
        received = fields[1][0:2].upper()
        return expected == received
    except:
        return False

def nmea_checksum(cmd :str) -> int:
    """XOR all bytes -- equivalent to BluefinMessages.checksum()"""
    checksum = 0
    for c in cmd:
        checksum ^= ord(c)
    return checksum

def parse_steer(msg  : str):
    """Parse $STEER,pulseWidth,angle*XX"""
    if not checkthesum(msg):
        print(f"Checksum mismatch, skipping: {msg}")
        return None
    
    fields = msg.split('*')[0].split(',')
    if fields[0] != '$STEER' or len(fields) < 3:
        return None
    
    return {
        'pulse_width': int(fields[1]),
        'angle': int(fields[2])
    }

class RCReceiver(object):
    """ 
    This class reads RC Receiver steering angle output from the Raspberry Pi in NMEA convention.
    """
    def __init__(self,
                 baudrate : int = 115200,
                 port : str = '/dev/ttyUSB0', 
                 verbose : bool = False
                 ):
        
        self.__verbose = verbose
        self.__serial = serial.Serial(port, baudrate, timeout=1)

    def get_data(self) -> dict[str, int] | None:
        line = self.__serial.readline().decode('utf-8', errors='ignore').strip()
        if line.startswith('$STEER'):
            result = parse_steer(line)
            if result:
                # print(f"Pulse: {result['pulse_width']}us | Angle: {result['angle']}°")
                return result
        return None
    
# ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

# while True:
#     line = ser.readline().decode('utf-8', errors='ignore').strip()
#     if line.startswith('$STEER'):
#         result = parse_steer(line)
#         if result:
#             print(f"Pulse: {result['pulse_width']}us | Angle: {result['angle']}°")

if __name__ == "__main__":
    if(sys.argv[1:] != list()):
        print(f"test_points={sys.argv[1:][0]}, verbose={sys.argv[1:][1]}")
        print("args passed!")
        print(sys.argv[1:])
        # imu = Odometry(test_points=int(sys.argv[1]), verbose=(True if str(sys.argv[2])=='True' else False))
    else:
        print("no args passed!")
        # imu = Odometry(test_points=10, verbose=False)
    rc_receiver = RCReceiver()

    while(True):
        # imu.update()
        # imu.add_to_csv()
        # t, raw, accel, vel, pos, rpy = imu.get_data()
        # print(imu.get_data())
        # print(f"Raw:{(round(raw[1:][0],2), round(raw[1:][1],2))}|Accel:{(round(accel[1:][0],2),}|Vel:{vel}|Pos:{pos}|Rpy:{rpy}")
        rc_receiver.get_data()
        pass
