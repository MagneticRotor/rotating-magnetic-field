""" Magnetic Rotator Main Program for Circuitpython

    This program manages the rotation of the magnetic field.
    It also provides a communication interface over USB.
    
    The program is designed to run on a Express device using
    the circuitpython interpreter (orginally developped for 
    4.0.2).
    
    Connections:
    - Servo:
      - GND and 5V
      - Signal to A2
    
    - Magnetic Sensors:

    Commands: The following commands can be used.
    - servo.stop: stops the rotating servo (sets speed to servo_zero_speed)
    - servo.speed f: Sets the speed to f (currently F can be -1 . . . 1)
    
    Task List
    - Make basic outline
    - Main loop:
      - ./ periodic status info
      - look for commands and react to them
        - get new text
        - check if <CR> available -> split complete command -> execute it

"""

# Imports
import time
import sys
import board
import pulseio
import supervisor
from adafruit_motor import servo

# Settings
servo_zero_speed = -0.017
servo_pin = board.A2

# create a PWMOut object on Pin A2.
pwm = pulseio.PWMOut(servo_pin, frequency=50)
     
# Create a servo object, my_servo.
my_servo = servo.ContinuousServo(pwm)

# Loop variable
command = '' # incoming command string
servo_speed = servo_zero_speed # current speed
# Main Loop
while True:
    # Print message
    print("Status: servospeed = %f, current command = <%s>" % (servo_speed,command))
    # Look for command
    if supervisor.runtime.serial_bytes_available:
        # Get new text
        while supervisor.runtime.serial_bytes_available:
            command += sys.stdin.read(1)
        # If there's a completed command -> isolate and print it
    # Sleep
    time.sleep(2.0)
     
"""
    while True:
        print("forward")
        my_servo.throttle = 1.0
        time.sleep(2.0)
        print("stop")
        my_servo.throttle = 0.0
        time.sleep(2.0)
        print("reverse")
        my_servo.throttle = -1.0
        time.sleep(2.0)
        print("stop")
        my_servo.throttle = 0.0
        time.sleep(4.0)
"""