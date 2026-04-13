import serial
import sys
import time

DEFAULT_READ_TIMEOUT_SEC = 0.02
DEFAULT_STALE_TIMEOUT_SEC = 0.25

def checkthesum(msg: str) -> bool:
    """
    Validate the checksum of an NMEA-style message.

    Parameters
    ----------
    msg : str
        NMEA-style message containing a checksum suffix.

    Returns
    -------
    bool
        ``True`` if the checksum matches, otherwise ``False``.
    """
    try:
        fields = msg.strip().split('*')
        cmd = fields[0][1:]  # strip leading $
        expected = hex(nmea_checksum(cmd))[2:].upper()
        received = fields[1][0:2].upper()
        return expected == received
    except:
        return False

def nmea_checksum(cmd :str) -> int:
    """
    Compute the XOR checksum for an NMEA command payload.

    Parameters
    ----------
    cmd : str
        NMEA command payload without the leading ``$`` or trailing checksum.

    Returns
    -------
    int
        XOR checksum value for the payload.
    """
    checksum = 0
    for c in cmd:
        checksum ^= ord(c)
    return checksum

def parse_steer(msg: str) -> dict[str, int] | None:
    """
    Parse a ``$STEER`` NMEA-style message.

    Parameters
    ----------
    msg : str
        Message of the form ``$STEER,pulseWidth,angle*XX``.

    Returns
    -------
    dict[str, int] | None
        Parsed steering data dictionary, or ``None`` if parsing fails.
    """
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
                 verbose : bool = False,
                 read_timeout_sec : float = DEFAULT_READ_TIMEOUT_SEC,
                 stale_timeout_sec : float = DEFAULT_STALE_TIMEOUT_SEC,
                 ):
        """
        Initialize the RC receiver serial interface.

        Parameters
        ----------
        baudrate : int, optional
            Serial baud rate used by the receiver.
        port : str, optional
            Serial device path.
        verbose : bool, optional
            Whether verbose output should be enabled.
        read_timeout_sec : float, optional
            Maximum time to wait for a serial line fragment before returning.
        stale_timeout_sec : float, optional
            Age threshold after which the cached steering sample is marked stale.

        Returns
        -------
        None
            This constructor initializes the serial connection.
        """
        self.__set_baudrate = baudrate
        self.__port = port
        self.__verbose = verbose
        self.__read_timeout_sec = max(0.0, float(read_timeout_sec))
        self.__stale_timeout_sec = max(0.0, float(stale_timeout_sec))
        self.__serial = serial.Serial(port, baudrate, timeout=self.__read_timeout_sec)
        self.__last_sample: dict[str, int | float | bool] | None = None

    def __cache_result(
        self,
        result: dict[str, int],
        received_monotonic_time: float | None = None,
    ) -> dict[str, int | float | bool]:
        """
        Store the latest parsed steering sample with a monotonic timestamp.

        Parameters
        ----------
        result : dict[str, int]
            Parsed steering sample containing pulse width and angle.
        received_monotonic_time : float | None, optional
            Time at which the sample was received. If ``None``, the current
            monotonic clock value is used.

        Returns
        -------
        dict[str, int | float | bool]
            Cached steering sample enriched with age and freshness metadata.
        """
        sample_time = time.monotonic() if received_monotonic_time is None else float(received_monotonic_time)
        self.__last_sample = {
            'pulse_width': int(result['pulse_width']),
            'angle': int(result['angle']),
            'sample_time_monotonic': sample_time,
        }
        return self.__format_sample(self.__last_sample, now_monotonic=sample_time)

    def __format_sample(
        self,
        sample: dict[str, int | float | bool] | None,
        now_monotonic: float | None = None,
    ) -> dict[str, int | float | bool] | None:
        """
        Return a steering sample with derived age and freshness flags.

        Parameters
        ----------
        sample : dict[str, int | float | bool] | None
            Cached sample to expose to callers.
        now_monotonic : float | None, optional
            Time to use when computing sample age.

        Returns
        -------
        dict[str, int | float | bool] | None
            Steering sample metadata, or ``None`` if no sample is cached.
        """
        if sample is None:
            return None

        current_time = time.monotonic() if now_monotonic is None else float(now_monotonic)
        sample_time = float(sample['sample_time_monotonic'])
        age_sec = max(0.0, current_time - sample_time)

        return {
            'pulse_width': int(sample['pulse_width']),
            'angle': int(sample['angle']),
            'sample_time_monotonic': sample_time,
            'sample_age_sec': age_sec,
            'is_fresh': age_sec <= self.__stale_timeout_sec,
        }

    def __parse_line(self, line: str) -> dict[str, int] | None:
        """
        Parse a single line from the receiver.

        Parameters
        ----------
        line : str
            Raw line read from the serial device.

        Returns
        -------
        dict[str, int] | None
            Parsed receiver data if the line is recognized, otherwise ``None``.
        """
        if line.startswith('$STEER'):
            return parse_steer(line)
        return None

    def get_data(self, latest: bool = True) -> dict[str, int | float | bool] | None:
        """
        Read steering data from the receiver without stalling the control loop.

        Parameters
        ----------
        latest : bool, optional
            Whether to drain the input buffer and return the most recent valid
            sample. When ``True`` and no new line is available, the method
            returns the latest cached sample immediately.

        Returns
        -------
        dict[str, int | float | bool] | None
            Parsed steering data enriched with sample age/freshness metadata, or
            ``None`` if no valid message has ever been received.
        """
        if not self.__serial.is_open:
            if self.__verbose:
                print(f"Serial port at {self.__port} is not open")
            return self.__format_sample(self.__last_sample)

        latest_sample = None
        latest_sample_time = None

        if latest:
            while self.__serial.in_waiting > 0:
                line = self.__serial.readline().decode('utf-8', errors='ignore').strip()
                result = self.__parse_line(line)
                if result is not None:
                    latest_sample = result
                    latest_sample_time = time.monotonic()

            if latest_sample is not None:
                return self.__cache_result(latest_sample, latest_sample_time)

            return self.__format_sample(self.__last_sample)

        if self.__serial.in_waiting > 0:
            line = self.__serial.readline().decode('utf-8', errors='ignore').strip()
            result = self.__parse_line(line)
            if result is not None:
                return self.__cache_result(result)

        return None
    
    # function to clear the serial monitor, such as for calibration time
    def reset(self):
        """
        Clear queued receiver data from the serial input buffer.

        Parameters
        ----------
        None

        Returns
        -------
        None
            This method discards any queued serial input.
        """
        if not self.__serial.is_open:
            if self.__verbose:
                print(f"Serial port at {self.__port} is not open")
            return

        queued_bytes = self.__serial.in_waiting

        # Clear input buffer, discarding all that is in the buffer.
        self.__serial.reset_input_buffer()
        self.__last_sample = None

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
