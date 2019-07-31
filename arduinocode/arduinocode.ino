/*  Magnetic Rotator Main Program for Arduino
    =========================================
   
    This program manages the rotation of the magnetic field.
    It also provides a communication interface over USB.

    Connections:
    - Servo:
      - GND and 5V
      - Signal to A2

    - Magnetic Sensors:
      - GND and 5V
      - SCL and SDA to corresponding pins

    Commands: See "help_message" below

 */

//**** Includes
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include "RTClib.h"
#include <Wire.h>
#include "Adafruit_MLX90393.h"

//**** Global Variables
// Servo variables: measured in us pulse to the servo
float servo_speed = 1500.0; // servo pulse in us (1500 is middle)
int servo_zero_speed = 1510;
int servo_max_speed = 2000;
int servo_min_speed = 1000;
float servo_rpm = 0.0; // Current rpm (only used if > 0)
// Timing variables: measured in ms
unsigned long read_nextime = 0, read_deltime = 10;
unsigned long fileout_nextime = 0, fileout_deltime = 100;
unsigned long print_nextime = 0, print_deltime = 500;
unsigned long stoptime = 0; // countdown is set if stoptime > 0
// Help message
String help_message = "Magnetic Field Rotator Commands:\n"
"  help - prints this message\n"
"  stop - stops the servo (both speed and rpm mode)\n"
"  speed value - sets the speed to given value (only works if rpm value = 0)\n"
"  rpm value - sets the desired RPM value (set rpm 0 to use speed command)\n"
"  read - returns magnet probe readout";
// Magnet sensor values (in uT)
float magx, magy, magz, magtot; // Current readings
float magval = 0.0; // Magnetic input value
float medval = 0.0, fltval = 0.0; // very long and short filter values
float oldval = 0.0; // previous fltval
// Magnet zero passage times and deltas (to measure rpm)
unsigned long zeropasstime = 0.0; // time of last passage through zero (ms)
float zeropassdelta = 1.0; // current time between last zero passages (s)
float zeropasslist[5] = {1.0, 1.0, 1.0, 1.0, 1.0}; // list of deltas of zero passages
int zeropasslind = 0; // index for list
// Input / Info variables
char command[100] = ""; // Command input
int commlen = 0; // current length of command input
int readcount = 0; // how many reads were done
int show_debug = 0; // flag is debug information is to be shown
String msg; // Output message
// Sensors / IO
Adafruit_MLX90393 magsensor = Adafruit_MLX90393();
RTC_PCF8523 rtc;
const int SDchipSelect = 10;
Servo my_servo;  

//**** Functions

// MagnetServo: Reads the magnet and if appropriate adjusts the servo speed
void magnetservo(){
  // Update time for next readout
  read_nextime = millis() + read_deltime;
  unsigned long timemons = millis();
  readcount ++;
  // read magnet values
  magsensor.readData(&magx, &magy, &magz);
  magtot = sqrt(magx*magx+magy*magy+magz*magz);
  // Get mag statistics
  magval = magy;
  medval = 0.99*medval + 0.01*magval;
  oldval = fltval;
  fltval = 0.9*fltval + 0.1*magval;
  // Check if fltval has risen above medval
  if(fltval > medval && oldval <= medval){
    // Check if time is reasonable
    if (timemons > zeropasstime + int(1000*zeropassdelta/5.0) && timemons > zeropasstime + 100){
      // update time delta and last time fltval rose above medval
      zeropassdelta = (timemons - zeropasstime)/1000.0;
      zeropasstime = timemons;
      // update list 
      zeropasslind += 1;
      if(zeropasslind >= 5){ zeropasslind = 0; }
      zeropasslist[zeropasslind] = zeropassdelta;
      // If rpm is set, do correction
      if(servo_rpm>0){
        float now_rpm = 0.0;
        for(int i=0;i<5;i++){ now_rpm += zeropasslist[i]; }
        now_rpm = 60*5/now_rpm;
        float rpmdiff = abs(servo_rpm-now_rpm) + 0.1;
        //Serial.println("RPMdiff = " + String(rpmdiff));
        if(servo_rpm > now_rpm) { servo_speed += rpmdiff/10.0;
        } else { servo_speed -= rpmdiff/10.0; }
        if(servo_speed > servo_max_speed) { servo_speed = servo_max_speed; }
        if(servo_speed < servo_zero_speed) { servo_speed = servo_zero_speed + 20; }
        my_servo.writeMicroseconds(round(servo_speed));
      }
    }
  }
  // Check for stopping
  if (stoptime < timemons && stoptime > timemons - 1000 && servo_speed != servo_zero_speed){
      servo_speed = servo_zero_speed;
      servo_rpm = 0;
      my_servo.writeMicroseconds(servo_zero_speed);
      Serial.println("Stopping Now");
  }
}

// PrintMessage: Prints messages
void printmessage(){
  // Update time for next print
  print_nextime = millis() + print_deltime;
  // Print message
  Serial.print("Status: speed = " + String(servo_speed,1) + ", magy/tot (uT) = " + String(magy) +
               "/" + String(magtot));
  float rpmnow = 0.0;
  for(int i=0; i<5; i++) { rpmnow += zeropasslist[i]; }
  Serial.print(", rpm_now/set = " + String(60*5/rpmnow,2) + "/" + String(servo_rpm));
  long i = stoptime-millis();
  if(i>0){
    Serial.print(", Stop in " + String(i/60000.,1) + "min");
  }
  Serial.println();
  // Print debug info
  if(show_debug){
    Serial.print(" valmed/flt = " + String(medval,1) + "/" + String(fltval,1) );
    Serial.print(" zpassdelta = " + String(zeropassdelta,2) );
    Serial.print(" Stoptime=" + String(stoptime) + " Millis=" + String(millis()));
    Serial.println(" Readcount = " + String(readcount));
  }
  // Print current command if needed
  if(commlen){
    Serial.print("> ");
    Serial.println(command);
  }
}

// CommandRx: Recieve and process commands
void commandrx(){
  char rxchar = Serial.read();
  if(rxchar != '\n' && rxchar != '\r'){
    // Normal character, add it
    command[commlen++] = rxchar;
    command[commlen] = 0;
  } else {
    // End of line character, handle command
    String comm = command; // convert to a string
    comm.trim(); // cut spaces at ends
    commlen = 0;
    command[commlen] = 0;
    // Help command
    if(comm.startsWith("help")){
      Serial.println(help_message);
    // Stop command
    } else if(comm.startsWith("stop")){
      // Check if there's a stop time in minutes
      if (comm.length() > 5){
        // Get the stop time
        String stopstr = comm.substring(5);
        stopstr.trim();
        float new_stop = stopstr.toFloat();
        // Check if it was successfull
        if(new_stop==0.0){
          Serial.println("Warning: Invalid stop time in command = " + comm);
          return;
        }
        // Check for valid values
        if(new_stop < 0 || new_stop > 43200){
          Serial.println("Warning: Stop time out of range (0..43200) in command = " + comm);
          return;
        }
        // Set the stop time
        stoptime = millis() + int(60000*new_stop);
        Serial.println("Set stop time in " + String(new_stop,1) + " minutes");
      } else {
        servo_rpm = 0;
        servo_speed = servo_zero_speed;
        my_servo.writeMicroseconds(servo_zero_speed);
        stoptime = 0;
        Serial.println("Stopped Servo");
      }
    // Speed command
    } else if(comm.startsWith("speed")){
      // Check if length is correct
      if (comm.length() < 7){
        Serial.println("Warning: Invalid speed value in command = " + comm);
        return;
      }
      // Get the speed
      String speedstr = comm.substring(5);
      speedstr.trim();
      int new_speed = speedstr.toInt();
      // Check if it was successfull
      if(new_speed==0){
        Serial.println("Warning: Invalid speed value in command = " + comm);
        return;
      }
      // Check for valid values
      if(new_speed < servo_min_speed || new_speed > servo_max_speed){
        Serial.println("Warning: Speed value out or range (" + String(servo_min_speed) +
                       ".." + String(servo_max_speed) + ") in command = " + comm);
        return;
      }
      // Set the speed
      servo_speed = new_speed;
      my_servo.writeMicroseconds(new_speed);
      servo_rpm = 0;
      Serial.println("Set servo speed to " + String(new_speed));
    // RPM command
    } else if(comm.startsWith("rpm")){
      // Check if length is correct
      if (comm.length() < 5){
        Serial.println("Warning: Invalid rpm value in command = " + comm);
        return;
      }
      // Get the rpm
      String rpmstr = comm.substring(4);
      rpmstr.trim();
      int new_rpm = rpmstr.toInt();
      // Check if it was successfull
      if(new_rpm==0){
        Serial.println("Warning: Invalid rpm value in command = " + comm);
        return;
      }
      // Check for valid values
      if(new_rpm < 1 || new_rpm > 150){
        Serial.println("Warning: RPM value out or range (1..150) in command = " + comm);
        return;
      }
      // Set the rpm
      servo_rpm = new_rpm;
      Serial.println("Set servo rpm to " + String(new_rpm));
      // If speed is zero, set initial speed at 10%
      if(abs(servo_speed - servo_zero_speed) < 20){
          servo_speed = servo_zero_speed + 100;
          my_servo.writeMicroseconds(round(servo_speed));
      }
    // Read command
    } else if(comm.startsWith("read")){
        Serial.println("Magnetometer (uT) - x, y, z, tot: " + String(magx,1) + " " +
                       String(magy,1) + " " + String(magz) + " " + String(magtot,1) );
    } else {
      Serial.print("Warning: Invalid command = ");
      Serial.println(comm);
    }
  }
}

// DatalogWrite: Writes to datalog file
void datalogwrite(){
  // Update time for next write
  fileout_nextime = millis() + fileout_deltime;
  // Check if drive is mounted (else mount it)
  // - - - - 
  // Make filename
  DateTime now = rtc.now();
  String date = String(now.year(), DEC) + "-" + String(now.month(), DEC) + "-" +
                String(now.day(), DEC);
  String filename = String(now.year(), DEC);
  if(now.month() > 9){
    filename += String(now.month());
  } else {
    filename += "0" + String(now.month());
  }
  if(now.day() > 9){
    filename += String(now.day());
  } else {
    filename += "0" + String(now.day());
  }
  if(now.hour() > 9){
    filename += String(now.hour());
  } else {
    filename += "0" + String(now.hour());
  }
  filename += ".txt";
  filename = filename.substring(2);
  char fname[50];
  filename.toCharArray(fname,50);
  // Check if file exists
  int filenew = 1;
  if(SD.exists(filename)) { filenew = 0; }
  // Open file and write to it
  File dataFile = SD.open(filename, FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    String datastr = "";
    if(filenew){
      datastr += "Date_Time\t";
      datastr += "ServoSpeed\tRPMset\tRPMnow\t";
      datastr += "Magx\tMagy\tMagz\r\n";
    }
    datastr += date + "_" + String(now.hour(), DEC) + ":" + String(now.minute(), DEC) +
               ":" + String(now.second(), DEC) + "\t";
    float rpmnow = 0.0;
    for(int i=0; i<5; i++) { rpmnow += zeropasslist[i]; }
    datastr += String(servo_speed, 1) + "\t" + String(servo_rpm, 1) + "\t" +
               String(60*5/rpmnow, 1) + "\t";
    datastr += String(magx,1) + "\t" + String(magy,1) + "\t" + String(magz);
    dataFile.println(datastr);
    dataFile.close();
  // if the file isn't open, pop up an error:
  } else {
    Serial.println("Warning: Error opening " + String(filename));
  }

}

//**** Setup and Loop
void setup() {
  // Setup Serial
  Serial.begin(115200);
  // Wait for serial on USB platforms.
  while(!Serial) { delay(10); }
  
  // Initialize MLX90393
  Serial.println("Starting MLX90393");
  if (magsensor.begin()) {
    Serial.println("Found a MLX90393 sensor");
  } else {
    Serial.println("No sensor fount ... trying again");
    delay(1000);
    if (magsensor.begin()) {
      Serial.println("Found a MLX90393 sensor");
    } else {
      Serial.println("No sensor found ... check your wiring?");  
      while (1);
    }
  }
  
  // Initialize RTC
  Serial.println("Initializing RTC");
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  if (! rtc.initialized()){
    Serial.println("RTC is NOT running!");
  } else {
    Serial.println("RTC Initialized");
  }

  // Initialize SD card
  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(SDchipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

  // Attach Servo
  my_servo.attach(A2);
}

void loop(){
  // Handle magnet and servo
  magnetservo();
  // Print message
  if(print_nextime < millis() && print_deltime > 0){
    printmessage();
  }
  // Get input
  if(Serial.available()>0){
    commandrx();
  }
  // Write log to file
  if(fileout_nextime < millis() && fileout_deltime > 0){
    datalogwrite();
  }
  // Sleep until next reading
  int sleep_delay = read_nextime - millis();
  if(sleep_delay>0){ delay(sleep_delay); }
}

// Test_Loop: tests functionality of all hardware
void looptst() {
  // Print data from magsensor
  if(magsensor.readData(&magx, &magy, &magz)) {
      Serial.print("X: "); Serial.print(magx, 4); Serial.println(" uT");
      Serial.print("Y: "); Serial.print(magy, 4); Serial.println(" uT");
      Serial.print("Z: "); Serial.print(magz, 4); Serial.println(" uT");
  } else {
      Serial.println("Unable to read XYZ data from the sensor.");
  }
  // Print clock
  DateTime now = rtc.now();
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
  // Add line to data.txt
  //File dataFile = SD.open("datalog.txt", FILE_WRITE);
  File dataFile = SD.open("19073015.txt", FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println("File Done");
    dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening 19073015.txt");
  }
  // Change servo
  if(servo_speed < 1600) {
    servo_speed += 10;
  } else {
    servo_speed = 1400;
  }
  my_servo.writeMicroseconds(servo_speed);
  Serial.print("Servo Speed = ");
  Serial.println(servo_speed);
  delay(1000);
}
