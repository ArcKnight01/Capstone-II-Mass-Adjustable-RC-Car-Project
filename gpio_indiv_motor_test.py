from gpiozero import Motor
from gpiozero import RGBLED
import os 
import sys
import time
import curses

# actions = {
#     cuses.KEY_UP: "fwd"
#     curses.KEY_DOWN: "rev"
#     curses.KEY_LEFT: "left"
#     curses.KEY_RIGHT: "right"
# }
led = RGBLED(23,24,25, active_high=True, pwm=True, initial_value=(1,0,0))

def spin_motor(motor:Motor, motor_speed:float, motor_direction:str):
    speed = float(motor_speed)
    direction = str(motor_direction)
    if(direction.lower().strip() == 'rev'):
        led.color = (1,0,0)
        speed = speed *-1
    elif(direction.lower().strip() == 'fwd'):
        led.color = (0,0,1)
        speed = speed
    else:
        led.color = (0,0,0)
        speed = 0
    assert (speed >= -100) & (speed <= 100), "[ERROR] Speed is out of bounds, must be a percent!"
    if(speed > 0):
        motor.forward(abs(speed)/100.0)
        if(motor.is_active):
            print("[INFO] Motor Forward.")
    elif(speed <0):
        motor.backward(abs(speed)/100.0)
        if(motor.is_active):
            print("[INFO] Motor Reverse.")
    else:
        motor.stop()
        if(motor.is_active == False):
            print("[INFO] Motor Stopped.")
    
def main():
    led.on()
    led.color = (0,1,0)
    test_motor = []
    if(sys.argv[1:][0] == "test"):
        if(sys.argv[1:] != list()):
            # assert (len(sys.argv[1:]) >= 3) & (len(sys.argv[1:]) <=5), "[ERROR] Invalid Arguments passed!"
            test_motor=Motor(forward= int(sys.argv[1:][1]), backward= int(sys.argv[1:][2]), enable=int(sys.argv[1:][3]), pwm=True)
            speed = float(sys.argv[1:][4])
            motor_dir = str(sys.argv[1:][5])
            while(True):
                spin_motor(test_motor,speed,motor_dir)
    elif(sys.argv[1:][0] == "all"):
        if(sys.argv[1:] != list()):
            left_motor_front=Motor(forward= 15, backward= 14, enable=18, pwm=True)
            left_motor_back=Motor(forward= 7, backward= 8, enable=12, pwm=True)
            left_speed = float(sys.argv[1:][1])
            right_motor_front=Motor(forward= 5, backward=6, enable=13, pwm=True)
            right_motor_back=Motor(forward= 26, backward= 20, enable=19, pwm=True)
            right_speed = float(sys.argv[1:][2])
            left_motor_dir = sys.argv[1:][3]
            right_motor_dir = sys.argv[1:][4]
            while(True):
                spin_motor(left_motor_front, left_speed, left_motor_dir)
                spin_motor(left_motor_back, left_speed, left_motor_dir)
                spin_motor(right_motor_front, right_speed, right_motor_dir)
                spin_motor(right_motor_back, right_speed, right_motor_dir)

if __name__ == "__main__":
    # curses.wrapper(main)
    main()