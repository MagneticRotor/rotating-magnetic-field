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

    Commands: See "help_message" below

    Task List
    - ./ Connect and get new sensor running -> test it
    - ./ Connect new servo and add magnet
    - Make code for getting maxima on all axis (need median code) and when going below median
    - Make code for getting time between maxima on each axis
    - make commands for setting rpm frequency (and store)
    - make code for adjusting RPM based on maxima interval

"""

# Imports
import time
import sys
import board
import busio
import pulseio
import supervisor
from adafruit_motor import servo
# import adafruit_lsm9ds0
import adafruit_mlx90393

# Settings
show_debug = True
servo_zero_speed = -0.01
servo_max_speed = 1.0
servo_min_speed = -1.0
servo_pin = board.A2
help_message = """ Magnetic Field Rotator Commands:
help - prints this message
stop - stops the servo (both speed and frequency mode)
speed value - sets the speed to given value (only works if rpm value = 0)
rpm value - sets the desired RPM value (set rpm 0 to use speed command)
read - returns magnet probe readout
"""

# create a PWMOut object on Pin A2.
pwm = pulseio.PWMOut(servo_pin, frequency=50)

# Create a servo object, my_servo.
my_servo = servo.ContinuousServo(pwm)
#my_servo = servo.Servo(pwm)

# Connect the Magnetometer
i2c = busio.I2C(board.SCL, board.SDA)
#magsensor = adafruit_lsm9ds0.LSM9DS0_I2C(i2c)
magsensor = adafruit_mlx90393.MLX90393(i2c, gain=adafruit_mlx90393.GAIN_1X)

# Statistics variable
magval = 0.0  # input value
medval = 0.0  # very long term filter time constant 10sec
fltval = 0.0  # short filter (time constant 0.5sec)
fltold = fltval # old filter value
timelast = time.monotonic()
deltime = 1.0
delist = [1.0 for i in range(5)] # list of deltas
delistind = 0 # index for list

# Loop variable
command = '' # incoming command string
servo_speed = servo_zero_speed # current speed
servo_rpm = 0.0 # current required rpm (only used if >0)
# Main Loop
while True:
    # Get magnetic field
    mag_x, mag_y, mag_z = magsensor.magnetic
    mag_tot = (mag_x**2+mag_y**2+mag_z**2)**0.5
    # Get mag statistics
    magval = mag_y
    medval = 0.99*medval + 0.01*magval
    fltold = fltval
    fltval = 0.8*fltval + 0.2*magval
    # Check if fltval has risen above medval
    if fltval > medval and fltold <= medval:
        # Check if time is reasonable
        if time.monotonic() > timelast + deltime/5.0 and time.monotonic() > timelast + 0.1:
            # update time delta and last time fltval rose above medval
            deltime = time.monotonic() - timelast
            timelast = time.monotonic()
            # update list 
            delistind += 1
            if delistind >= len(delist): delistind = 0
            delist[delistind] = deltime
            # If rpm is set, do correction
            if servo_rpm:
                now_rpm = 60*len(delist)/sum(delist)
                rpmdiff = abs(servo_rpm-now_rpm) + 0.5
                if servo_rpm > now_rpm: servo_speed += rpmdiff / 1000
                else: servo_speed -= rpmdiff / 1000
                my_servo.throttle = servo_speed
    # Print message
    print("Status: speed = %.3f, magy/tot (uT) = %.0f/%.0f, rpm_now/set = %.2f/%.2f" 
          % (servo_speed,mag_y,mag_tot, 60*len(delist)/sum(delist), servo_rpm))
    if show_debug:
        print("  valmed/flt =  %.1f/%.1f, dtime = %.2f, <delist> = %.2f" % 
              (medval, fltval, deltime, sum(delist)/len(delist)))
    if len(command):
        print("> %s" % command)
    # Look for command
    if supervisor.runtime.serial_bytes_available:
        # Get new text
        while supervisor.runtime.serial_bytes_available:
            command += sys.stdin.read(1)
            if show_debug: print("New char %s %d" % (command[-1],ord(command[-1])))
            # Treat backspace
            if ord(command[-1]) in [127,8]:
                if len(command) > 1: command = command[:-2]
                else: command = ''
        # If there's a completed command -> isolate and print it
        if '\n' in command:
            fullcomm, command = command.split('\n',1)
            # Help command
            if 'help' in fullcomm.strip()[:4]:
                print(help_message)
            # Stop command
            elif 'stop' in fullcomm.strip()[:4]:
                servo_speed = servo_zero_speed
                my_servo.throttle = servo_zero_speed
                print('Stopped Servo')
            # Speed command
            elif 'speed' in fullcomm.strip()[:5]:
                # Get the speed
                try:
                    new_speed = float(fullcomm.strip()[6:].strip())
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
            # Speed command
            elif 'rpm' in fullcomm.strip()[:3]:
                # Get the speed
                try:
                    new_rpm = float(fullcomm.strip()[4:].strip())
                except:
                    print('Invalid rpm in command <%s>' % fullcomm)
                    continue
                # Check the speed
                if new_rpm < 0 or new_rpm > 150:
                    print('RPM %f out of range [%f . . %f]' %
                          (new_rpm, 0, 150))
                    continue
                # Set the rpm
                servo_rpm = new_rpm
                print('Set servo rpm to %.2f' % new_rpm)
                # If speed is zero, set initial speed at 10%
                if servo_speed == servo_zero_speed:
                    servo_speed = 0.1
                    my_servo.throttle = servo_speed
            # Get Magnet Command
            elif 'read' in fullcomm.strip()[:4]:
                # Get the value
                mag_x, mag_y, mag_z = magsensor.magnetic
                mag_tot = (mag_x**2+mag_y**2+mag_z**2)**0.5
                print('Magnetometer (gauss) - x, y, z, tot: %f, %f, %f, %f' %
                      (mag_x, mag_y, mag_z, mag_tot))
            else:
                print('Warning, invalid command <%s>' % fullcomm)
    # Sleep
    time.sleep(0.10)

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
