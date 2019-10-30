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

/* Developper Notes:

   2DO:
   ./ Remove servo code
   - Add code for stepmove
   ./ Update magnsensor and stepper code
   - Review timers, make timer for magsensor, make sure no large part in same 5ms (logging at end)
   - redo overall loop to have at least 10ms between long items (optimize motor movement)
   - redefine Neopixel behavior
   ./ edit help menu
   - add logging (have a queue for messages, runs if noone else is running for 5ms)

 */

//**** Defines
#define NeoPin 40
#define DIRA 12
#define PWRA 3
#define DIRB 13
#define PWRB 11

//**** Includes
#include <SPI.h>
#include <SD.h>
//#include "SdFat.h"
#include <RTClib.h>
#include <Wire.h>
#include <Adafruit_MLX90393.h>
#include <Adafruit_NeoPixel.h>

//**** Global Variables
// Steper Motor Variables:
int stepcount = 0; // counter for steps (always 0..1023)
float steprpm = 0; // current set RPM (0 for no move, always agrees with stepdeltgoal)
float stepvelgoal = 0; // goal step velocity steps/sec
float stepvelnow = 0; // current velocity steps/sec
long stepdeltnow = 30000; // current step time for movement [us] (0 for no move, always > 0)
float stepaccel = 100; // acceleration [steps/s/s]
float stepperrev = 200; // steps per revolution
unsigned long stepnextms = 0; // milliseconds at which next step to be taken [ms]
unsigned long stepnextmic = 0; // microseconds at which next step to be taken [us] (0..999)
int stepdir = 0; // variable inside nextstep()
// Timing variables: measured in ms
unsigned long milnow = 0; // current millis (to avoid calls to millis())
unsigned long magstep_nextime = 0, magstep_deltime = 50;
unsigned long fileout_nextime = 0, fileout_deltime = 5000;
unsigned long print_nextime = 0, print_deltime = 1000;
unsigned long runfile_nextime = 0, runfile_deltime = 1000;
unsigned long stoptime = 0; // countdown is set if stoptime > 0
// Help message
String help_message = "Magnetic Field Rotator Commands:\r\n"
"    help - prints this message\r\n"
"    stop (delay) - stops the servo (both speed and rpm mode)\r\n"
"                   optional stop delay in minutes\r\n"
"    rpm value - sets the desired RPM value\r\n"
"    read - returns magnet probe readout\r\n"
"    sleep value - sleep in minutes (only valid inside RUNME.TXT)\r\n"
"  Logs will be stored on the SD card. The SD card may also contain\r\n"
"  a script named RUNME.TXT with is executed on startup.\r\n"
"  For more details, look at instructions at\r\n"
"    https://github.com/lingyuan001/rotating-magnetic-field-";
// Magnet sensor values (in uT)
float magx, magy, magz, magtot; // Current readings
float magval = 0.0; // Magnetic input value
float medval = 0.0, fltval = 0.0; // very long and short filter values
float oldval = 0.0; // previous fltval
// Magnet zero passage times and deltas (to measure rpm)
unsigned long zeropasstime = 0.0; // time of last passage through zero (ms)
float zeropassdelta = 1.0; // current time between last zero passages (s)
float zeropasslist[5] = {0.2, 0.2, 0.2, 0.2, 0.2}; // list of deltas of zero passages
int zeropasslind = 0; // index for list
float now_rpm = 0; // measured RPM from magnetic sensor
// Input / Info variables
char command[200] = ""; // Command input
int commlen = 0; // current length of command input
String comm = ""; // A fully received command, ready for execution
bool runfile_active = false; // Flag indicating we're running "RUNME.TXT"
char runfilecomm[200] = ""; // Runfile command input
int runfilecommlen = 0; // current length of runfile command input
File runfile; // Open runfile
int readcount = 0; // how many reads were done
int show_debug = 1; // flag is debug information is to be shown
String msg; // Output message
// Sensors / IO
Adafruit_MLX90393 magsensor = Adafruit_MLX90393();
int magsensor_present = 0;
RTC_PCF8523 rtc;
//RTC_DS1307 rtc;
const int SDchipSelect = 10;
Adafruit_NeoPixel strip(1, NeoPin, NEO_GRB);
// Variables for software SD card access
//const uint8_t SOFT_MISO_PIN = 12;
//const uint8_t SOFT_MOSI_PIN = 11;
//const uint8_t SOFT_SCK_PIN  = 13;
//const uint8_t SD_CHIP_SELECT_PIN = 10;
// SdFat software SPI template
//dFatSoftSpi<SOFT_MISO_PIN, SOFT_MOSI_PIN, SOFT_SCK_PIN> sd;

//**** Functions

void nextstep(){
  stepcount++;
  if(stepcount==1024){ stepcount = 0; }
  if(stepcount & 2){ stepdir = HIGH;
  } else { stepdir = LOW; }
  if(stepcount & 1){
    digitalWrite(DIRA,stepdir);
    digitalWrite(PWRA,HIGH);
    digitalWrite(PWRB, LOW);
  } else {
    digitalWrite(DIRB,stepdir);
    digitalWrite(PWRB,HIGH);
    digitalWrite(PWRA, LOW);
  }
}

// Stepmove: Determines if motor has to move, moves motor 1 step
void stepmove(){
  if(milnow > stepnextms && stepdeltnow){
    // call the next step
    nextstep();
    //mcount++; // if you don't call nextstep
    // update mildelt and milnext
    stepnextmic += stepdeltnow;
    stepnextms = stepnextms + stepnextmic / 1000;
    // correct stepnextmic
    stepnextmic = stepnextmic % 1000;
    // if milnext == millis -> increase by one
    if(milnow > stepnextms){
      if(milnow - stepnextms < 3){
    	Serial.print("X ");
      } else if(milnow - stepnextms < 10){
      	Serial.print("XX ");
      } else if(milnow - stepnextms < 33){
      	Serial.print("XXX ");
      } else {
        Serial.print("XXXX ");
      }
      while(milnow > stepnextms){ stepnextms++; }
    }
  }
}

// Magnetstep: Reads the magnet and updates stepper speed
void magnetstep(){
  // Update time for next readout
  magstep_nextime = millis() + magstep_deltime;
  readcount ++;
  // read magnet values
  //magsensor.readData(&magx, &magy, &magz);
  magtot = sqrt(magx*magx+magy*magy+magz*magz);
  // Get mag statistics
  magval = magy;
  medval = 0.99*medval + 0.01*magval;
  oldval = fltval;
  fltval = 0.9*fltval + 0.1*magval;
  // Check if fltval has risen above medval
  if(fltval > medval && oldval <= medval){
    // Check if time is reasonable
    if (milnow > zeropasstime + int(1000*zeropassdelta/5.0) && milnow > zeropasstime + 100){
      // update time delta and last time fltval rose above medval
      zeropassdelta = (milnow - zeropasstime)/1000.0;
      zeropasstime = milnow;
      // update list 
      zeropasslind += 1;
      if(zeropasslind >= 5){ zeropasslind = 0; }
      zeropasslist[zeropasslind] = zeropassdelta;
      // Calculate now_rpm
      now_rpm = 0.0;
      for(int i=0;i<5;i++){ now_rpm += zeropasslist[i]; }
      now_rpm = 60*5/now_rpm;
    }
  }
  // Check for stopping
  if (stoptime < milnow && stoptime > milnow - 1000){
    steprpm = 0;
    stepvelgoal = 0;
    Serial.println("Stopping motor now");
  }
  // Update motor speed (assume this happens every magstep_deltime)
  // If goal is smaller
  if(stepvelgoal < stepvelnow){
    if( stepvelnow - stepvelgoal > stepaccel * magstep_deltime / 1000.){
      stepvelnow = stepvelnow - stepaccel * magstep_deltime / 1000.;
    } else {
      stepvelnow = stepvelgoal;
    }
    if(abs(stepvelnow)>1.0){
      stepdeltnow = 1000000.0 / stepvelnow;
    } else {
      stepdeltnow = 0;
    }
  // Else if need to accelerate
  } else if( stepvelnow < stepvelgoal ){
    if( stepvelgoal - stepvelnow > stepaccel * magstep_deltime / 1000.){
      stepvelnow = stepvelnow + stepaccel * magstep_deltime / 1000.;
    } else {
      stepvelnow = stepvelgoal;
    }
    if(abs(stepvelnow)>1.0){
      stepdeltnow = 1000000.0 / abs(stepvelnow);
    } else {
      stepdeltnow = 0;
    }
  }
}

// PrintMessage: Prints messages
void printmessage(){
  // Update time for next print
  print_nextime = millis() + print_deltime;
  // Print message
  Serial.print("Status: speed_now/goal = " + String(stepvelnow,1) + "/" + String(stepvelgoal,1) +
		       ", magy/tot (uT) = " + String(magy) + "/" + String(magtot));
  Serial.print(", rpm_now/set = " + String(now_rpm,2) + "/" + String(steprpm,2));
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
    Serial.print(" StepDeltNow = " + String(stepdeltnow));
    Serial.println(" Stepcount = " + String(stepcount));
  }
  // Print current command if needed
  if(commlen){
    Serial.print("> ");
    Serial.println(command);
  }
  if(!steprpm){
    strip.setPixelColor(0,0,0,255);
  } else {
    strip.setPixelColor(0,0,255,0);
  }
  strip.show();
}

// CommandRx: Recieve and process commands
void commandrx(){
  // Check if serial data is available, if so get character
  if(Serial.available()>0){
    char rxchar = Serial.read();
    if(rxchar != '\n' && rxchar != '\r'){
      // Regular character, add it
      command[commlen++] = rxchar;
      command[commlen] = 0;
    } else {
      // End of line character, handle command
      comm = command; // convert to a string and store in comm
      commlen = 0;
      command[commlen] = 0;
      // deactivate runfile if it's active
      if(runfile_active){
        runfile_active = false;
        runfile_nextime = 0;
        runfile.close();
        Serial.println("Running RUNME.TXT aborted");
      }
      // run the command in comm
      commandexec();
    }
  }
  if(runfile_active){
    // Exit if timer is running
	  if(runfile_nextime > milnow){
 	    return;
	  }
	  // Check if more characters available
	  if(!runfile.available()){
  	  // Reached end of file, exit runfile
	    runfile_active = false;
	    runfile_nextime = 0;
	    runfile.close();
	    Serial.println("Finished running RUNME.TXT");
    }
	  // Get next line
	  char rxchar = runfile.read();
	  runfilecommlen = 0;
	  runfilecomm[0] = 0;
	  while(runfile.available() && rxchar != '\n' && rxchar != '\r'){
      runfilecomm[runfilecommlen++] = rxchar;
	    runfilecomm[runfilecommlen] = 0;
      rxchar = runfile.read();
	}
	// If command has text -> execute command
	if(runfilecomm[0] > 0){
	  comm = runfilecomm;
	  comm.trim(); // cut spaces
      Serial.println(comm);
      // It's a comment -> ignore
      if(comm.startsWith("#")){
        return;
      }
	  // If it's a sleep command, set up timer
      if(comm.startsWith("sleep")) {
  		  // Check if length is correct
 		    if (comm.length() < 7){
		      Serial.println("Warning: Invalid sleep value in command = " + comm);
		      return;
		    }
		    // Get the delay
		    String delaystr = comm.substring(6);
		    delaystr.trim();
		    float new_delay = delaystr.toFloat();
		    // Check if it was successfull
		    if(new_delay==0.0){
		      Serial.println("Warning: Invalid sleep value in command = " + comm);
		      return;
		    }
		    // Set the sleep
		    runfile_nextime  = millis() + new_delay * 60000;
		    Serial.println("Pausing script for " + String(new_delay) + " minutes");
	    } else {
		    // Run command
		    commandexec();
		    // Set nextime to give pause between commands
		    runfile_nextime = millis() + runfile_deltime;
	    }
	  }
  }
}

// CommandExec: Executest any command in comm
void commandexec(){
  comm.trim(); // cut spaces at ends
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
      steprpm = 0;
      stepvelgoal = 0;
      stoptime = 0;
      Serial.println("Stopped Motor");
    }
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
    steprpm = new_rpm;
    stepvelgoal = stepperrev * steprpm / 60.0;
    Serial.println("Set motor rpm to " + String(new_rpm));
  // Read command
  } else if(comm.startsWith("read")){
      Serial.println("Magnetometer (uT) - x, y, z, tot: " + String(magx,1) + " " +
                     String(magy,1) + " " + String(magz) + " " + String(magtot,1) );
  } else {
    Serial.print("Warning: Invalid command = ");
    Serial.println(comm);
  }
}

// DatalogWrite: Writes to datalog file
void datalogwrite(){
  // Update time for next write
  fileout_nextime = milnow + fileout_deltime;
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
  filename += ".TXT";
  filename = filename.substring(2);
  char fname[50];
  filename.toCharArray(fname,50);
  // Check if file exists
  int filenew = 1;
  //#####if(SD.exists(filename)) { filenew = 0; }
  // Open file and write to it
  File dataFile = SD.open(filename, FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    String datastr = "";
    if(filenew){
      datastr += "Date_Time\t";
      datastr += "MotorSpeed\tRPMset\tRPMnow\t";
      datastr += "Magx\tMagy\tMagz\r\n";
    }
    datastr += date + "_" + String(now.hour(), DEC) + ":" + String(now.minute(), DEC) +
               ":" + String(now.second(), DEC) + "\t";
    datastr += String(stepvelnow, 1) + "\t" + String(steprpm, 1) + "\t";
    datastr += String(now_rpm, 1) + "\t";
    datastr += String(magx,1) + "\t" + String(magy,1) + "\t" + String(magz);
    dataFile.println(datastr);
    dataFile.close();
  // if the file isn't open, pop up an error:
  } else {
    Serial.println("Warning: Error opening " + String(filename));
    strip.setPixelColor(0,255,0,0);
    strip.show();
  }
}

//**** Setup and Loop
void setup() {
  // Setup Serial
  Serial.begin(115200);
  // Set Pinmodes
  pinMode(DIRA, OUTPUT);
  pinMode(PWRA, OUTPUT);
  pinMode(DIRB, OUTPUT);
  pinMode(PWRB, OUTPUT);
  // Wait for serial on USB platforms.
  if(!Serial) {
    delay(1000);
  }

  // Set up neopixel - set red
  strip.begin();
  strip.show();
  strip.setPixelColor(0,255,0,0);
  strip.show();


  // Initialize MLX90393
  Serial.println("Starting MLX90393");
  magsensor_present = 1;
  if (magsensor.begin()) {
    Serial.println("Found a MLX90393 sensor");
  } else {
    Serial.println("No sensor found ... trying again");
    delay(1000);
    if (magsensor.begin()) {
      Serial.println("Found a MLX90393 sensor");
    } else {
      Serial.println("No sensor found ... check your wiring?");  
      delay(5000);
      if(magsensor.begin()) {
        Serial.println("Found a MLX90393 sensor");
      } else {
        Serial.println("No sensor found ... keep going without it");  
        magsensor_present = 0;
      }
    }
  }
  if(magsensor.setGain(MLX90393_GAIN_2_5X)){
    Serial.println("MLX90393 sensor - gain set");
  } else {
    Serial.println("MLX90393 sensor - gain set failure");
  }
  // Initialize RTC
  Serial.println("Initializing RTC");
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  //if (! rtc.initialized()){
  //  Serial.println("RTC is NOT running!");
  //} else {
  //  Serial.println("RTC Initialized");
  //}

  // Initialize SD card
  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  //if (!SD.begin(SDchipSelect)) {
  if(!SD.begin(SDchipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

  // Check if RUNME.TXT is available
  runfile = SD.open("runme.txt");
  if(runfile){
	  Serial.println("RUNME.TXT found - setting up script");
	  runfile_active = true;
  } else {
	  Serial.println("No RUNME.TXT found");
  }

  // Greeting
  Serial.println("System is ready!");
  Serial.println("Type \"help\" for a list of commands.");
}

void loop(){
  // Do next step if needed
  milnow = millis();
  // Move motor
  stepmove();
  // Get input
  commandrx();
  // Handle magnet and stepper change
  if(magstep_nextime < milnow && magstep_deltime > 0){
    magnetstep();
  // Print message
  } else if(print_nextime < milnow && print_deltime > 0){
    printmessage();
  // Write log to file
  } else if(fileout_nextime < milnow && fileout_deltime > 0){
    datalogwrite();
  }
  // Write to command log file
  // Sleep until next reading
  //if ( millis() == milnow ){
	//  delay(1);
  //}
}
