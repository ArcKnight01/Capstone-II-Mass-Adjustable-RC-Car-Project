import serial
import sys

def checkthesum(msg :str):
    """checksum logic"""
    try:
        fields = msg.strip().split('*')
        cmd = fields[0][1:]  # strip leading $
        expected = hex(nmea_checksum(cmd))[2:].upper()
        received = fields[1][0:2].upper()
        return expected == received
    except:
        return False

def nmea_checksum(cmd :str) -> int:
    """XOR all bytes """
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
    else:
        print("no args passed!")
       
    rc_receiver = RCReceiver()

    while(True):
        result = rc_receiver.get_data()
        print(f"Pulse: {result['pulse_width']}us | Angle: {result['angle']}°")
        pass
