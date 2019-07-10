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
import busio
import pulseio
import supervisor
from adafruit_motor import servo
import adafruit_lsm9ds0

# Settings
servo_zero_speed = -0.017
servo_max_speed = 1.0
servo_min_speed = -1.0
servo_pin = board.A2
help_message = """ Magnetic Field Rotator Commands:
help - prints this message
servo.stop - stops the servo
servo.speed value - sets the speed to given value
"""

# create a PWMOut object on Pin A2.
pwm = pulseio.PWMOut(servo_pin, frequency=256)
     
# Create a servo object, my_servo.
my_servo = servo.ContinuousServo(pwm)
#my_servo = servo.Servo(pwm)

# Connect the Magnetometer
i2c = busio.I2C(board.SCL, board.SDA)
magsensor = adafruit_lsm9ds0.LSM9DS0_I2C(i2c)

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
        if '\n' in command:
            fullcomm, command = command.split('\n')
            # Help command
            if 'help' in fullcomm.strip()[:4]:
                print(help_message)
            # Stop command
            elif 'servo.stop' in fullcomm.strip()[:10]:
                servo_speed = servo_zero_speed
                my_servo.throttle = servo_zero_speed
                print('Stopped Servo')
            # Speed command
            elif 'servo.speed' in fullcomm.strip()[:11]:
                # Get the speed
                try:
                    new_speed = float(fullcomm.strip()[11:].strip())
                except:
                    print('Invalid speed in command <%s>' % fullcomm)
                    continue
                # Check the speed
                if new_speed < servo_min_speed or new_speed > servo_max_speed:
                    print('Speed %f out of range [%f . . %f]' % 
                          (new_speed, servo_min_speed, servo_max_speed))
                    continue
                # Set the speed
                servo_speed = new_speed
                my_servo.throttle = new_speed
                print('Set servo speed to %.2f' % new_speed)
            # Get Magnet Command
            elif 'mag.read' in fullcomm.strip()[:8]:
                # Get the value
                mag_x, mag_y, mag_z = magsensor.magnetic
                mag_tot = (mag_x**2+mag_y**2+mag_z**2)**0.5
                print('Magnetometer (gauss) - x, y, z, tot: %f, %f, %f, %f' % 
                      (mag_x, mag_y, mag_z, mag_tot))
            else:
                print('Warning, invalid command <%s>' % fullcomm)
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
