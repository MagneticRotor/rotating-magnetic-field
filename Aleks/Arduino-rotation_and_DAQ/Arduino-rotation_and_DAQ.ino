#include <Adafruit_MLX90393.h>

#define DIRA 2
#define PWRA 3
#define DIRB 4
#define PWRB 11
#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 256


//**** Global Variables
// Steper Motor Variables:
volatile int stepcount = 0; // counter for steps (always 0..1023)
int stepdir = 1; // motor direction 1 for positive, -1 for negative, 0 for stopped

// 30 RPM
// long stepdeltnow = 10000; // current step time for movement [us] (0 for no move, always > 0)
// 50 RPM
//long  stepdeltnow = 6000;
long  stepdeltnow = 6000;


volatile int coildir = 0; // variable inside interrupt
float magx, magy, magz, magtot; // Current readings
Adafruit_MLX90393 magsensor = Adafruit_MLX90393();

unsigned long t1;
unsigned long t2;

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

void setup() {
  // Setup Serial
  Serial.begin(2000000);
  // Set Pinmodes
  pinMode(DIRA, OUTPUT);
  pinMode(PWRA, OUTPUT);
  pinMode(DIRB, OUTPUT);
  pinMode(PWRB, OUTPUT);
  // Wait for serial on USB
  interruptInit(1000000/stepdeltnow);

if (! magsensor.begin_I2C()) {          // hardware I2C mode, can pass in address & alt Wire
  //if (! sensor.begin_SPI(MLX90393_CS)) {  // hardware SPI mode
    Serial.println("No sensor found ... check your wiring?");
    while (1) { delay(10); }
  }

  magsensor.setGain(MLX90393_GAIN_5X);
  magsensor.setResolution(MLX90393_X, MLX90393_RES_16);
  magsensor.setResolution(MLX90393_Y, MLX90393_RES_16);
  magsensor.setResolution(MLX90393_Z, MLX90393_RES_16);
  magsensor.setFilter(MLX90393_FILTER_1);
}

void loop() {
  t1 = millis();
  magsensor.readData(&magx, &magy, &magz);
  t2 = millis();
  //Serial.println(t2-t1);
  magtot = sqrt(magx*magx+magy*magy+magz*magz);
  Serial.print(t1);
  Serial.print("\t");
  Serial.print(magx);
  Serial.print("\t");
  Serial.print(magy);
  Serial.print("\t");
  Serial.print(magz);
  //Serial.print("\t");
  //Serial.print(magtot);
  Serial.print("\n");
  //delay(1000);
}
