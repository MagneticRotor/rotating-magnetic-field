""" Magnetic Rotator Main Program for Circuitpython

    This program manages the rotation of the magnetic field.
    It also provides a communication interface over USB.

    The program is designed to run on a Express device using
    the circuitpython interpreter (orginally developped for
    4.0.2).
    
    WARNING: This program is incomplete, as there is insufficient
             memory on the Metro M0 to run the full program.
             use the Arduino version of the program instead.

    Connections:
    - Servo:
      - GND and 5V
      - Signal to A2

    - Magnetic Sensors:
      - GND and 5V
      - SCL and SDA to corresponding pins

    Commands: See "help_message" below

    Task List
    ./ Make more frequent readings (but less frequent printouts)
      - use local time instead of delay: make zeropassdelta
    ./ cycle through longer tasks (includes write to file, write command at end of each)
    ./ Get code for setting clock
    - Get code from writing file (new and append)
      - setup setup spi, cs
      - if /sd doesn't exist: try umount, setup, mount
      - make filename
      - make string and write to file
    - Getting timestamp into message
    - Create new file when needed
    - Write to file: time, rpmset/now, speed, magxyx, stopcount

    ISSUE: Insufficient Memory
    - With sliming things down maximally unable to import storage, SDCard, PCF8523 (rtc)
      - with no code I can only import PCF8523
      - gc.collect doesn't fix the problem.
      - gc.mem_free() says we use 9kB of memory loading pcf8523 module
      - MLX90393 needs 6kB of memory
      - adafruit_sdcard needs 6kB of memory
    - Possible solutions:
      - Get M4 Express
      - Make arduino program
      - Just to logging in labview: Not an option

"""
# Imports
import gc
print(gc.mem_free())
from time import sleep, monotonic, monotonic_ns
from sys import stdin
from board import A2, D10, SCK, MOSI, MISO, SCL, SDA
from busio import I2C
from busio import SPI
from digitalio import DigitalInOut
import storage
from pulseio import PWMOut
from supervisor import runtime
from adafruit_motor import servo
#from adafruit_sdcard import SDCard
#from adafruit_pcf8523 import PCF8523
print(gc.mem_free())
from adafruit_mlx90393 import MLX90393, GAIN_1X
print(gc.mem_free())

# Settings
#show_debug = True
servo_zero_speed = 0.017
servo_max_speed = 1.0
servo_min_speed = -1.0
read_deltime = 0.02 # time interval for readings
fileout_deltime = 0.1 # time interval for writing to file (disable if == 0)
print_deltime = 0.5 # time interval for printing to serial (disable if == 0)
help_message = """ Magnetic Field Rotator Commands:
help - prints this message
stop - stops the servo (both speed and frequency mode)
speed value - sets the speed to given value (only works if rpm value = 0)
rpm value - sets the desired RPM value (set rpm 0 to use speed command)
read - returns magnet probe readout
"""

# create a PWMOut object on Pin A2 (for servo)
pwm = PWMOut(A2, frequency=50)

# Create a servo object, my_servo.
my_servo = servo.ContinuousServo(pwm)

# Connect the I2C
i2c = I2C(SCL, SDA)
# Connect to the magnetometer
# Following is for MLX90393 sensor
#     Possible addresses (depending on A0/A1 setting) are
#     0x0C, 0x0D, 0x0E and 0x0F
magsensor = MLX90393(i2c, address = 0x0C, gain=GAIN_1X)
# Connect to the rtc
#rtc = PCF8523(i2c)

# Set up SPI for talking to SD card
spi = SPI(SCK, MOSI=MOSI, MISO=MISO)
cs = DigitalInOut(D10)

# Time variables
# Comment: all is done in nanoseconds b/c time.monotonic
#          looses accuracy after a few hours
readcount = 0 # Just to keep track
read_nextime = monotonic_ns()
fileout_nextime = read_nextime
print_nextime = read_nextime

# Statistics variable: The system detects when the short filter value (fltval)
#     rises above the long filter value (medval). It then uses the time between
#     subsequent such events to determine the rotation frequency. The last 5
#     measurements are averaged to determine the speed correction.
magval = 0.0  # input value
medval = 0.0  # very long term filter time constant 10sec
fltval = 0.0  # short filter (time constant 0.5sec)
oldval = fltval # old (short) filter value
zeropasstime = read_nextime # time of last passage through zero (IN NANOSECONDS)
zeropassdelta = 1.0 # current time between last zero passages
zeropasslist = [1.0 for i in range(5)] # list of deltas of zero passages
zeropasslind = 0 # index for list
# State variables
stoptime = read_nextime/1000000000-10 # time when to stop (countdown is set if > time.monotonic)

# Loop variable
command = '' # incoming command string
servo_speed = servo_zero_speed # current speed
servo_rpm = 0.0 # current required rpm (only used if >0)
# Main Loop
while True:
    # Garbage collect
    gc.collect()
    # Set next read time (in ns)
    timemons = monotonic_ns()    
    read_nextime = timemons + int(read_deltime*1000000000)
    readcount += 1
    # Get magnetic field and time
    mag_x, mag_y, mag_z = magsensor.magnetic
    mag_tot = (mag_x**2+mag_y**2+mag_z**2)**0.5
    # Get mag statistics
    magval = mag_y
    medval = 0.99*medval + 0.01*magval
    oldval = fltval
    fltval = 0.8*fltval + 0.2*magval
    # Check if fltval has risen above medval
    if fltval > medval and oldval <= medval:
        # Check if time is reasonable
        if timemons > zeropasstime + int(1000000000*zeropassdelta/5.0) and timemons > zeropasstime + 100000000:
            # update time delta and last time fltval rose above medval
            zeropassdelta = (timemons - zeropasstime)/1000000000.0
            zeropasstime = timemons
            # update list 
            zeropasslind += 1
            if zeropasslind >= len(zeropasslist): zeropasslind = 0
            zeropasslist[zeropasslind] = zeropassdelta
            # If rpm is set, do correction
            if servo_rpm:
                now_rpm = 60*len(zeropasslist)/sum(zeropasslist)
                rpmdiff = abs(servo_rpm-now_rpm) + 0.1
                if servo_rpm > now_rpm: servo_speed += rpmdiff / 1000
                else: servo_speed -= rpmdiff / 1000
                if servo_speed > 1.0: servo_speed = 1.0
                my_servo.throttle = servo_speed
    # Check for stopping
    if stoptime < timemons and stoptime > timemons - 1000000000 and servo_speed != servo_zero_speed:
        servo_speed = servo_zero_speed
        servo_rpm = 0
        my_servo.throttle = servo_zero_speed
        print("Stopping Now")
    # Print message
    if print_nextime < timemons:
        # Set next time to print
        print_nextime = timemons + int(print_deltime*1000000000)
        # Calculate stoptime in minutes
        if stoptime > timemons: stopmin = (stoptime-timemon)/60000000000
        else: stopmin = -1
        print("Status: speed = %.4f, magy/tot (uT) = %.0f/%.0f, rpm_now/set = %.2f/%.2f, stop in %.1f" 
              % (servo_speed,mag_y,mag_tot, 60*len(zeropasslist)/sum(zeropasslist), servo_rpm, stopmin))
        #if show_debug:
        #    print("  valmed/flt =  %.1f/%.1f, dtime = %.2f, <zeropasslist> = %.2f, rdcnt = %d" % 
        #          (medval, fltval, zeropassdelta, sum(zeropasslist)/len(zeropasslist), readcount))
        if len(command):
            print("> %s" % command)
    # Look for command
    if runtime.serial_bytes_available:
        # Get new text
        while runtime.serial_bytes_available:
            command += stdin.read(1)
            #if show_debug: print("New char %s %d" % (command[-1],ord(command[-1])))
            # Treat backspace
            if ord(command[-1]) in [127,8]:
                if len(command) > 1: command = command[:-2]
                else: command = ''
        # If there's a completed command -> isolate and print it
        if '\n' in command:
            fullcomm, command = command.split('\n',1)
            fullcomm = fullcomm.strip()
            # Help command
            if 'help' in fullcomm[:4]:
                print(help_message)
            # Stop command
            elif 'stop' in fullcomm[:4]:
                # Check if there's a stop time in minutes
                if len(fullcomm) > 5:
                    # Get the stop time
                    try:
                        new_stop = float(fullcomm[4:].strip())
                    except:
                        print('Invalid stop time in command <%s>' % fullcomm)
                        continue
                    # Check the rpm
                    if new_stop < 0 or new_stop > 43200:
                        print('Stop time %f out of range [%f . . %f]' %
                              (new_rpm, 0, 43200))
                        continue
                    # Set the stop time
                    stoptime = timemons + int(60 * new_stop)*1000000000
                    print('Set stop time in %.1f minutes' % new_stop)
                else:
                    servo_rpm = 0
                    servo_speed = servo_zero_speed
                    my_servo.throttle = servo_zero_speed
                    print('Stopped Servo')
            # Speed command
            elif 'speed' in fullcomm[:5]:
                # Get the speed
                try:
                    new_speed = float(fullcomm[6:].strip())
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
                servo_rpm = 0
                print('Set servo speed to %.2f' % new_speed)
            # Speed command
            elif 'rpm' in fullcomm[:3]:
                # Get the rpm
                try:
                    new_rpm = float(fullcomm[4:].strip())
                except:
                    print('Invalid rpm in command <%s>' % fullcomm)
                    continue
                # Check the rpm
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
            elif 'read' in fullcomm[:4]:
                # Get the value
                mag_x, mag_y, mag_z = magsensor.magnetic
                mag_tot = (mag_x**2+mag_y**2+mag_z**2)**0.5
                print('Magnetometer (gauss) - x, y, z, tot: %f, %f, %f, %f' %
                      (mag_x, mag_y, mag_z, mag_tot))
            else:
                print('Warning, invalid command <%s>' % fullcomm)
    # Write to file
#    if fileout_nextime < timemons:
        # Set next time to print
#        fileout_nextime = timemons + int(print_deltime*1000000000)    # Sleep to next_readtime (approximate)
        # Check if drive is mounted (else mount it)
#        if not 'sd' in os.listdir('/'):
#            sdcard = SDCard(spi, cs) # have to do this and following line when card is out
#            vfs = storage.VfsFat(sdcard)
#            storage.mount(vfs,'/sd')
        # Make filename
#        t = rtc.datetime
#        filename = 'Datalog_%d-%d-%d_%dh.txt' % (t.tm_year,t.tm_mon,t.tm_mday,t.tm_hour)
        # Open file and write to it
        #filenew = True
        #if filename in os.listdir('/sd'): filenew = False
#        with open('/sd/'+filename, 'at') as f:
#            datastr = ''
            #if filenew:
            #    datastr += 'Date_Time\t'
            #    datastr += 'ServoSpeed\tRPMset\tRPMnow\t'
            #    datastr += 'Magx\tMagy\tMagz\r\n'
#            datastr += '%d-%d-%d_%02d:%02d:%02d\t' % (t.tm_year,t.tm_mon,t.tm_mday,
#                                                      t.tm_hour,t.tm_min,t.tm_sec)
#            datastr += '%.4f\t%.2f\t%.2f' % (servo_speed, 60*len(zeropasslist)/sum(zeropasslist), servo_rpm)
            #datastr += '%.0f\t%.0f\t%.0f\r\n' % (mag_x, mag_y, mag_z)
#            f.write(datastr)
    sleep_delay = (read_nextime - monotonic_ns() ) / 1000000000.0
    if sleep_delay>0: sleep(sleep_delay)