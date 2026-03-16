import pigpio

class RCReceiver:
    
    def __init__(self, gpio_pin):
        self.pi = pigpio.pi()
        self.gpio = gpio_pin
        
        self.high_tick = None
        self.pulse_width = 1500
        
        self.pi.set_mode(self.gpio, pigpio.INPUT)

        self.callback = self.pi.callback(
            self.gpio,
            pigpio.EITHER_EDGE,
            self._callback
        )

    def _callback(self, gpio, level, tick):

        # rising edge
        if level == 1:
            self.high_tick = tick

        # falling edge
        elif level == 0 and self.high_tick is not None:
            self.pulse_width = pigpio.tickDiff(self.high_tick, tick)

    def read(self):
        return self.pulse_width
    
def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

if __name__ == "__main__":
    import time
    import board
    import busio
    from adafruit_pca9685 import PCA9685
    from adafruit_servokit import ServoKit

    i2c = busio.I2C(board.SCL, board.SDA)

    pca = PCA9685(i2c)

    kit = ServoKit(channels=16, i2c=i2c)    

    # Initialize PWM board
    # kit = ServoKit(channels=16)


    steeringRCInput = RCReceiver(0)
    speedRCInput = RCReceiver(1)

    kit.servo[0].set_pulse_width_range(1000, 2000)
    kit.servo[0].actuation_range = 180
    # tune ESC pulse range
    kit.continuous_servo[1].set_pulse_width_range(1000, 2000)
    while True:
        steeringRCInput_pw = steeringRCInput.read()
        speedRCInput_pw = speedRCInput.read()

        steering_angle = map_range(steeringRCInput_pw, 1000, 2000, 0, 180) # These values will need to be replaced with accurate ranges.
         # Map to ESC throttle
        throttle = map_range(speedRCInput_pw, 1000, 2000, -1, 1)

        # Send commands
        kit.servo[0].angle = steering_angle
        kit.continuous_servo[1].throttle = throttle

        print(f"steer pulse: {steeringRCInput_pw} us | speed pulse: {speedRCInput_pw} us")

        time.sleep(0.1)
  