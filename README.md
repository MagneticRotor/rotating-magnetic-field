# Magnetic Field Rotator

## Users Manual

This manual describes how to use the magnetic field rotator. The purpose of this setup is to create a rotating magnetic field. To use it you the following items
 * The rotator box: Contains the Metro M0 microcontroller, the real time clock and the SD card.
 * A USB cable or a 7-9V power source
 * A continuous rotation servo with attached rare earth magnet
 * One of the magnetic sensors (optional but required to run the system in RPM mode).
 * A computer to communicate with the setup (optional but required to send commands to the system).
 * The labview program on a computer (opitonal)
 
### Electrical Setup
Connect the magnetic sensor to any of the RJ-12 connectors. Then connect the servo to the servo connector hanging out of the box. Make sure that the brown wire of the servo cable connects to the black wire of the connector cable. Finally connect the USB cable or power supply.

### Serial Communication with the Rotator
To communicate with the rotator you need either the Arduino program or the Mu editor. Alternatively you can use Putty (Windows) or screen (Linux, Mac). Once you connected the rotator box to the computer you can find the port it's connected to. On windows find the correct COM port in the Device Manager, for Mac and Unix the port will be listed in the /dev folder.

Once the communication is established you should see information streaming from the rotator. Enter the "help" command for a list of available commands.

### Recording Data
The system includes an SD card slot. Whenever a properly formatted SD card is inserted the system records current settings and readouts. Each hour a new file is made, the filename format is YYMMDDHH.TXT composed of the current year, month, day and hour. Unplug the system from the power supply / USB cable before removing the SD card.

### Scripted Operation
You can load a simple script onto the SD card which will run whenever the system starts up. The script has to be in a file called RUNME.TXT. The script can contain any command listed in by the help function. The script can also contain the sleep command, which is the word "sleep" followed by the number of seconds that the system should wait before executing the next command. Entering any command over the serial connection will interrupt execution of the RUNME script.

### LabView Program
The system can also be controlled by a labview program which is the 
    labvcode/MagnetorotGUI.vi
file in this project. This program opens a user interface which is shown in
    testcode/labviewtest/Labview190805_Panel.png
 
Before starting the program, make sure you have the correct COM setting under "VISA resource name" in the upper left. Once the system is connected to the computer via USB and the program is running, you should see system information in the output field. The Speed-Set, RPM-Set and STOP Servo buttons run the speed, rpm and stop functions on the controller. You can directly enter commands in the "Raw String" field. To exit the program press the QUIT button.

## Developer Notes

### Hardware
 * [Metro M0](https://www.adafruit.com/product/3505)
 * [3ft USB Cable](https://www.adafruit.com/product/592)
 * [Continuous Servo](https://www.adafruit.com/product/154)
 * [Magnetometer MLX90393](https://www.adafruit.com/product/4022)
 
### Wiring information
 * The servo is connected to GND, 5V and Pin A2
 * The magnetic sensor(s) is connected to GND, 5V and the I2C pins, SCL and SDA

### Software Reference
 * Adartuit Metro M0 Express Tutorial under [learn.adafruit.com/adafruit-metro-m0-express-designed-for-circuitpython](https://learn.adafruit.com/adafruit-metro-m0-express-designed-for-circuitpython)
 * Adafruit circuitpyton servo tutorial under [learn.adafruit.com/circuitpython-essentials/circuitpython-servo](https://learn.adafruit.com/circuitpython-essentials/circuitpython-servo)
 
### Developer Notes
 * Circtuipython has supervisor.runtime.serial_bytes_available
