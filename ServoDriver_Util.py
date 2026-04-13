"""
ServoDriver_Util.py - Servo driver utility notes and examples.

This module contains commented-out example code for initializing PWM servo
hardware using Adafruit PCA9685 and the Raspberry Pi I2C bus. It is provided
as a reference for future servo actuation integration.
"""

# import board
# import busio
# import adafruit_pca9685
# i2c = busio.I2C(board.SCL, board.SDA)
# pca = adafruit_pca9685.PCA9685(i2c)

# # SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# # SPDX-License-Identifier: MIT

# # Outputs a 50% duty cycle PWM single on the 0th channel.
# # Connect an LED and resistor in series to the pin
# # to visualize duty cycle changes and its impact on brightness.

# import board

# from adafruit_pca9685 import PCA9685

# # Create the I2C bus interface.
# i2c = board.I2C()  # uses board.SCL and board.SDA
# # i2c = busio.I2C(board.GP1, board.GP0)    # Pi Pico RP2040

# # Create a simple PCA9685 class instance.
# pca = PCA9685(i2c)

# # Set the PWM frequency to 60hz.
# pca.frequency = 60

# # Set the PWM duty cycle for channel zero to 50%. duty_cycle is 16 bits to match other PWM objects
# # but the PCA9685 will only actually give 12 bits of resolution.
# pca.channels[0].duty_cycle = 0x7FFF


# # SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# # SPDX-License-Identifier: MIT

# """Simple test for a standard servo on channel 0 and a continuous rotation servo on channel 1."""

# import time

# from adafruit_servokit import ServoKit

# # Set channels to the number of servo channels on your kit.
# # 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
# kit = ServoKit(channels=8)

# kit.servo[0].angle = 180
# kit.continuous_servo[1].throttle = 1
# time.sleep(1)
# kit.continuous_servo[1].throttle = -1
# time.sleep(1)
# kit.servo[0].angle = 0
# kit.continuous_servo[1].throttle = 0


# if __name__ == "__main__":
#     import time
#     import board
#     import busio
#     from adafruit_pca9685 import PCA9685
#     from adafruit_servokit import ServoKit

#     i2c = busio.I2C(board.SCL, board.SDA)

#     pca = PCA9685(i2c)

#     kit = ServoKit(channels=16, i2c=i2c)    

#     # Initialize PWM board
#     # kit = ServoKit(channels=16)


#     steeringRCInput = RCReceiver(0)
#     speedRCInput = RCReceiver(1)

#     kit.servo[0].set_pulse_width_range(1000, 2000)
#     kit.servo[0].actuation_range = 180
#     # tune ESC pulse range
#     kit.continuous_servo[1].set_pulse_width_range(1000, 2000)
#     while True:
#         steeringRCInput_pw = steeringRCInput.read()
#         speedRCInput_pw = speedRCInput.read()

#         steering_angle = map_range(steeringRCInput_pw, 1000, 2000, 0, 180) # These values will need to be replaced with accurate ranges.
#          # Map to ESC throttle
#         throttle = map_range(speedRCInput_pw, 1000, 2000, -1, 1)

#         # Send commands
#         kit.servo[0].angle = steering_angle
#         kit.continuous_servo[1].throttle = throttle

#         print(f"steer pulse: {steeringRCInput_pw} us | speed pulse: {speedRCInput_pw} us")

#         time.sleep(0.1)
  