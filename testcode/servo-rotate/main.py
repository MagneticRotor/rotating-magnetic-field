""" Continuous Servo Test Program

    This program is designed to continuously run a 
    servo connected to pin A2 at a given Velocity.

    Change the velocity variable below and re-upload
    the program to change the speed.
"""

# Velocity setting: Can be a number between -1.0 and 1.0
velocity = 1.0

# Continuous Servo Test Program for CircuitPython
import time
import board
import pulseio
from adafruit_motor import servo

print("Initialize Servo")
# create a PWMOut object on Pin A2.
pwm = pulseio.PWMOut(board.A2, frequency=50)
     
# Create a servo object, my_servo.
my_servo = servo.ContinuousServo(pwm)

# Check velocity
if velocity > 1.0: velocity = 1.0
if velocity < -1.0: velocity = -1.0

# Start movement
print("Starting Movement")
my_servo.throttle = velocity

# This should run the servo continuously. If the program aborts
# and stops the movement you might have to add the following lines.
#while True:
#    print('Still Moving . . .')
#    time.sleep(10.0)
    
