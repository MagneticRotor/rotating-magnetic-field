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
   ./ Add code for stepmove
   ./ Update magnsensor and stepper code
   ./ Review timers, make timer for magsensor, make sure no large part in same 5ms (logging at end)
   ./ redo overall loop to have at least 10ms between long items (optimize motor movement)
   ./ Upgrade stepper code for use of interrupts
   ./ redefine Neopixel behavior
   ./ edit help menu
   ./ add logging (have a queue for messages, runs if noone else is running for 5ms)
   - test with checking if file exists, otherwise do other program
   - Implement AutoRestart: autodetect if zeropt missing after 20s
                            -> stop for 5s then restart

 */

//**** Defines
#define NeoPin 40
/*
// Settings for old motor shield
#define DIRA 12
#define PWRA 3
#define DIRB 13
#define PWRB 11
*/
// Settings for new motor shield
#define DIRA 2
#define PWRA 3
#define DIRB 4
#define PWRB 11
#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 256

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
volatile int stepcount = 0; // counter for steps (always 0..1023)
int stepdir = 0; // motor direction 1 for positive, -1 for negative, 0 for stopped
float steprpm = 0; // current set RPM (0 for no move, always agrees with stepdeltgoal)
float stepvelgoal = 0; // goal step velocity steps/sec
float stepvelnow = 0; // current velocity steps/sec
long stepdeltnow = 30000; // current step time for movement [us] (0 for no move, always > 0)
float stepaccel = 100; // acceleration [steps/s/s]
float stepperrev = 200; // steps per revolution
unsigned long stepnextms = 0; // milliseconds at which next step to be taken [ms]
unsigned long stepnextmic = 0; // microseconds at which next step to be taken [us] (0..999)
volatile int coildir = 0; // variable inside interrupt
// Timing variables: measured in ms
unsigned long milnow = 0; // current millis (to avoid calls to millis())
unsigned long magstep_nextime = 0, magstep_deltime = 25;
unsigned long fileout_nextime = 0, fileout_deltime = 100;
unsigned long print_nextime = 0, print_deltime = 1000;
unsigned long runfile_nextime = 0, runfile_deltime = 200;
unsigned long stoptime = 0; // countdown is set if stoptime > 0
// Help message
String help_message = "Magnetic Field Rotator Commands:\r\n"
"    help - prints this message\r\n"
"    stop (delay) - stops the servo (both speed and rpm mode)\r\n"
"                   optional stop delay in minutes\r\n"
"    off - turns off motor immediately\r\n"
"    rpm value - sets the desired RPM value\r\n"
"    read - returns magnet probe readout\r\n"
"    sleep value - sleep in minutes (only valid inside RUNME.TXT)\r\n"
"    autorestart on/off - activate / deactivate autorestart feature"
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
int rpm_good = 0; // number of good RPM measurements
int autorestart = 0; // autorestart flag: ==0 inactive, ==1 active >1 waiting & stopped
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
String logmessage = ""; // log message (there is a log message len() > 0 )
// Sensors / IO
Adafruit_MLX90393 magsensor = Adafruit_MLX90393();
int magsensor_present = 0;
RTC_PCF8523 rtc;
DateTime now;
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

void interruptSetFrequency(int frequencyHz) {
  int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1; // calculate compare value
  TcCount16* TC = (TcCount16*) TC3;
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue; // Set Compare value
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

void interruptInit(int frequencyHz) {
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

  TcCount16* TC = (TcCount16*) TC3; // Get Time Counter

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use the 16-bit timer
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16; // Set 16 bit count mode
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use match mode so that the timer counter resets when the count matches the compare register
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Set prescaler to 1024
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV256;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  interruptSetFrequency(frequencyHz);

  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

  NVIC_EnableIRQ(TC3_IRQn);

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

void TC3_Handler() {
  TcCount16* TC = (TcCount16*) TC3;
  // If this interrupt is due to the compare register matching the timer count
  // we send a pulse to the motor
  if (TC->INTFLAG.bit.MC0 == 1) {
    TC->INTFLAG.bit.MC0 = 1;
    // Switch coils off
    digitalWrite(PWRA, LOW);
    digitalWrite(PWRB, LOW);
    if(stepdir){
      // Increase (and limit) step count
      stepcount+=stepdir;
      if(stepcount==1024){ stepcount = 0; }
      if(stepcount<0){ stepcount = 1023; }
      // Get active coil direction
      if(stepcount & 2){ coildir = HIGH;
      } else { coildir = LOW; }
      // Set active coil
      if(stepcount & 1){
        digitalWrite(DIRA,coildir);
        digitalWrite(PWRA,HIGH);
      } else {
        digitalWrite(DIRB,coildir);
        digitalWrite(PWRB,HIGH);
      }
    }
  }
}

// Magnetstep: Reads the magnet and updates stepper speed
void magnetstep(){
  // Update time for next readout
  magstep_nextime = milnow + magstep_deltime;
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
      // check if rpm is good -> increase rpm_good
      if(abs(now_rpm-steprpm) < steprpm/20){
    	  if(rpm_good<100){ rpm_good++; }
        } else {
    	  rpm_good = 0;
      }
    }
  }
  // Check for stopping from delayed stop command
  if (stoptime < milnow && stoptime > milnow - 1000){
    steprpm = 0;
    stepvelgoal = 0;
    logmessage = "Stopping motor now";
    Serial.println(logmessage);
  }
  // Check if we're stopped after an autorestart
  if(autorestart > 1){
	autorestart--;
  // Check zero pass has been missing for 5 rotations plus 5 sec -> trigger autorestart
  // - also make sure we're supposed to be at speed -
  } else if(autorestart && milnow - zeropasstime > (300000/steprpm+5000) &&
		    abs(stepvelnow - stepvelgoal) > stepvelgoal/10) {
	// Autorestart required, do an OFF and set delay
	stepvelnow = 0;
	stepdir = 0;
	autorestart = 5000 / magstep_deltime; // set to start up again in 5s
  // Update motor speed (assume this happens every magstep_deltime)
  // If goal is smaller
  } else if(stepvelgoal < stepvelnow){
    if( stepvelnow - stepvelgoal > stepaccel * magstep_deltime / 1000.){
      stepvelnow = stepvelnow - stepaccel * magstep_deltime / 1000.;
    } else {
      stepvelnow = stepvelgoal;
    }
    if(abs(stepvelnow)>1.0){
      stepdeltnow = 1000000.0 / stepvelnow;
      stepdir = 1;
    } else {
      stepdeltnow = 10000;
      stepdir = 0;
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
      stepdir = 1;
    } else {
      stepdeltnow = 10000;
      stepdir = 0;
    }
  }
  interruptSetFrequency(1000000/stepdeltnow);
  // Set RGB LED color
  if(autorestart > 1){
	// Autorestart triggered and stopped -> Red
	strip.setPixelColor(0,255,0,0);
  } else if(abs(stepvelnow - stepvelgoal) > stepvelgoal/10 ){
	// Not at speed -> Orange
	strip.setPixelColor(0,255,153,0);
  } else if(rpm_good > 3){
	// Speed good -> Blue
    strip.setPixelColor(0,51,51,255);
  } else {
	// At speed -> Green
    strip.setPixelColor(0,0,255,0);
  }
  strip.show();
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
    Serial.print(" StepDeltNow = " + String(stepdeltnow*stepdir));
    Serial.println(" Stepcount = " + String(stepcount));
  }
  // Print current command if needed
  if(commlen){
    Serial.print("> ");
    Serial.println(command);
  }
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
	    logmessage = "Finished running RUNME.TXT";
	    Serial.println(logmessage);
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
 		      logmessage = "Warning: Invalid sleep value in command = " + comm;
		      Serial.println(logmessage);
		      return;
		    }
		    // Get the delay
		    String delaystr = comm.substring(6);
		    delaystr.trim();
		    float new_delay = delaystr.toFloat();
		    // Check if it was successfull
		    if(new_delay==0.0){
		      logmessage = "Warning: Invalid sleep value in command = " + comm;
		      Serial.println(logmessage);
		      return;
		    }
		    // Set the sleep
		    runfile_nextime  = millis() + new_delay * 60000;
		    logmessage = "Pausing script for " + String(new_delay) + " minutes";
		    Serial.println(logmessage);
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
    	logmessage = "Warning: Invalid stop time in command = " + comm;
        Serial.println(logmessage);
        return;
      }
      // Check for valid values
      if(new_stop < 0 || new_stop > 43200){
    	logmessage = "Warning: Stop time out of range (0..43200) in command = " + comm;
        Serial.println(logmessage);
        return;
      }
      // Set the stop time
      stoptime = millis() + int(60000*new_stop);
      logmessage = "Set stop time in " + String(new_stop,1) + " minutes";
      Serial.println(logmessage);
    } else {
      steprpm = 0;
      stepvelgoal = 0;
      stepvelnow = 0;
      stoptime = 0;
      logmessage = "Stopping Motor";
      Serial.println(logmessage);
    }
  // OFF command
  } else if(comm.startsWith("off")){    
    steprpm = 0;
    stepvelgoal = 0;
    stepvelnow = 0;
    stepdir = 0;
    logmessage = "Turning motor off now";
    Serial.println(logmessage);
  // RPM command
  } else if(comm.startsWith("rpm")){
    // Check if length is correct
    if (comm.length() < 5){
      logmessage = "Warning: Invalid rpm value in command = " + comm;
      Serial.println(logmessage);
      return;
    }
    // Get the rpm
    String rpmstr = comm.substring(4);
    rpmstr.trim();
    int new_rpm = rpmstr.toInt();
    // Check if it was successfull
    if(new_rpm==0){
      logmessage = "Warning: Invalid rpm value in command = " + comm;
      Serial.println(logmessage);
      return;
    }
    // Check for valid values
    if(new_rpm < 1 || new_rpm > 180){
      logmessage = "Warning: RPM value out or range (1..180) in command = " + comm;
      Serial.println(logmessage);
      return;
    }
    // Set the rpm
    steprpm = new_rpm;
    stepvelgoal = stepperrev * steprpm / 60.0;
    logmessage = "Set motor rpm to " + String(new_rpm);
    Serial.println(logmessage);
  // Read command
  } else if(comm.startsWith("read")){
	  logmessage = "Magnetometer (uT) - x, y, z, tot: " + String(magx,1) + " " +
                  String(magy,1) + " " + String(magz) + " " + String(magtot,1);
    Serial.println(logmessage);
  // Autorestart command
  } else if(comm.startsWith("autorestart on")){
    autorestart = 1;
    logmessage = "Autorestart enabled";
    Serial.println(logmessage);
  } else if(comm.startsWith("autorestart off")){
    autorestart = 0;
    logmessage = "Autorestart disabled";
    Serial.println(logmessage);
  } else {
    logmessage = "Warning: Invalid command = " + comm;
    Serial.println(logmessage);
  }
}

// DatalogWrite: Writes to datalog file
void datalogwrite(){
  // Update time for next write
  fileout_nextime = milnow + fileout_deltime;
  // Check if drive is mounted (else mount it)
  // - - - - 
  // Make filename
  now = rtc.now(); // get new now value
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

// DatalogWrite: Writes to datalog file
void messagelog(){
  // Make filename
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
  filename += "LG.TXT";
  filename = filename.substring(2);
  // Open file and write to it
  File dataFile = SD.open(filename, FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    String datastr = "";
    datastr += date + "_" + String(now.hour(), DEC) + ":" + String(now.minute(), DEC) +
               ":" + String(now.second(), DEC) + "\t";
    datastr += logmessage;
    dataFile.println(datastr);
    dataFile.close();
  // if the file isn't open, pop up an error:
  } else {
    Serial.println("Warning: Error opening " + String(filename));
    strip.setPixelColor(0,255,0,0);
    strip.show();
  }
  logmessage = "";
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
  now = rtc.now(); // get new now value
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
  // Initialize timer interrupt
  interruptInit(1); // at 1Hz - irrelevant since it's off


  // Greeting
  Serial.println("System is ready!");
  Serial.println("Type \"help\" for a list of commands.");
  logmessage = "Init Complete";
}

void loop(){
  // Do next step if needed
  milnow = millis();
  // Move motor
  //stepmove();
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
  // If a logmessage is available -> write it
  } else if(logmessage.length()){
	messagelog();
  }
  // Write to command log file
  // Sleep until next reading
  //if ( millis() == milnow ){
	//  delay(1);
  //}
}
