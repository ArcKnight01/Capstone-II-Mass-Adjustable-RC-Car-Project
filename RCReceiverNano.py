import serial
import sys
import time

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
        self.__set_baudrate = baudrate
        self.__port = port
        self.__verbose = verbose
        self.__serial = serial.Serial(port, baudrate, timeout=1)

    def __parse_line(self, line: str) -> dict[str, int] | None:
        if line.startswith('$STEER'):
            return parse_steer(line)
        return None

    def get_data(self, latest: bool = True) -> dict[str, int] | None:
        if self.__serial.is_open:
            latest_result = None

            if latest:
                while self.__serial.in_waiting > 0:
                    line = self.__serial.readline().decode('utf-8', errors='ignore').strip()
                    result = self.__parse_line(line)
                    if result is not None:
                        latest_result = result

                if latest_result is not None:
                    return latest_result

            line = self.__serial.readline().decode('utf-8', errors='ignore').strip()
            result = self.__parse_line(line)
            if result is not None:
                return result
        else: 
            if self.__verbose:
                print(f"Serial port at {self.__port} is not open")
        return None
    
    # function to clear the serial monitor, such as for calibration time
    def reset(self):
        if not self.__serial.is_open:
            if self.__verbose:
                print(f"Serial port at {self.__port} is not open")
            return

        queued_bytes = self.__serial.in_waiting

        # Clear input buffer, discarding all that is in the buffer.
        self.__serial.reset_input_buffer()

        if self.__verbose:
            print(f"Cleared {queued_bytes} incoming bytes from the serial queue")

if __name__ == "__main__":
    if(sys.argv[1:] != list()):
        print("args passed!")
        print(sys.argv[1:])
    else:
        print("no args passed! if you want to simulate calibration time, add an int after running this file (e.g. python3 RCReceiverNano.py 4 to simulate 4 seconds calibration time to build up serial buffer)")
       
    rc_receiver = RCReceiver()
    if(sys.argv[1:] != list()):
        secs_wait : int = int(sys.argv[1:][0])
        print(f"waiting {secs_wait} seconds to simulate calibration time between initializing the RCReceiver and reading data causing buffer build up")
        time.sleep(float(secs_wait))
        do_flush = (True if str(sys.argv[1:][1])=='True' else False)
        if do_flush:
            rc_receiver.reset()

    while(True):
        result = rc_receiver.get_data()
        print(f"Pulse: {result['pulse_width']}us | Angle: {result['angle']}°")
        pass
