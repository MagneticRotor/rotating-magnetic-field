/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->	http://www.adafruit.com/products/1438
*/


#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #2 (M3 and M4)
Adafruit_StepperMotor *myMotor = AFMS.getStepper(48, 2); // NW motor
//Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 1);


void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Stepper test!");

  AFMS.begin(100);  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  myMotor->setSpeed(100);  // 10 rpm   
}

void loop() {
  //Serial.println("Single coil steps");
  //myMotor->step(200, FORWARD, SINGLE); 
  //myMotor->step(200, BACKWARD, SINGLE); 

  Serial.println("5RPM");
  myMotor->setSpeed(5);  // 10 rpm   
  myMotor->step(200, FORWARD, DOUBLE); 
  delay(500);
  Serial.println("20RPM");
  myMotor->setSpeed(20);
  myMotor->step(1000, FORWARD, DOUBLE);
  delay(500);
  Serial.println("100RPM");
  myMotor->setSpeed(100);
  myMotor->step(2000, FORWARD, DOUBLE);
  delay(500);
  
  /*Serial.println("Interleave coil steps");
  myMotor->step(200, FORWARD, INTERLEAVE); 
  myMotor->step(200, BACKWARD, INTERLEAVE); 
  
  Serial.println("Microstep steps");
  myMotor->step(50, FORWARD, MICROSTEP); 
  myMotor->step(50, BACKWARD, MICROSTEP);//*/
}

