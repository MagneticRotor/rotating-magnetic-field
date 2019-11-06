/* My Code Sparkfun Shield
 *  
 *   # Tested with Baudouin's motor with 25 Ohm resistor in supply.
 *     Worked from 10 to 300RPM (use delayMicroseconds)
 *     
 * For timer and interrupt:
 *   # I need CTC (clear timer on compare match) mode and need interrupt on that.
 *   # need to set OCF2A flag set in the TIMSK2 register
 * 
 * Formula RPM to deltime: deltime = 300000 / rpm
 */

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
#define DTIME 5 // In microseconds
#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 256

unsigned long milnow = 0; // current millis
unsigned long milnext = 0; // Next mil counter at which step is called
unsigned long milbase = 0; // base count in milisecond
unsigned long mildelt = 0; // microseconds for next count
unsigned long deltime = 33000; // time between steps (microseconds)
unsigned long milprint = 0; // next print
int chcount = 0; // count to next change of step
int mdir = 1; // motor direction (motor is off for mdir == 0)
volatile int mcount = 0; // motor count
volatile int stepdir = 0; // step dir inside nextstep
int acc = 1; // accelerate at start

void startTimer(int frequencyHz);

void setTimerFrequency(int frequencyHz);

void TC3_Handler();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(DIRA, OUTPUT);
  pinMode(PWRA, OUTPUT);
  pinMode(DIRB, OUTPUT);
  pinMode(PWRB, OUTPUT);
  digitalWrite(PWRA, LOW);
  digitalWrite(PWRB, LOW);
  if(deltime<2000){ deltime = 2000; } // Just to make sure
  startTimer(10);
}

/* Steps
 * - Aon, Aoff
 * - Aon Bon
 * - A+ B+ A- B-
 */

void loop() {
  if(milprint < millis()){
    TcCount16* TC = (TcCount16*) TC3;
    milprint += 2000;
    Serial.print("Speed = "); Serial.print(deltime); Serial.print(" Freq = "); Serial.println(1000000/deltime);
    Serial.print("Comparator Value = "); Serial.println(TC->CC[0].reg);
    if(!chcount){
      if(acc){
        if(deltime>2000){
          deltime = 9*deltime/10;
        } else {
          acc = 0;
          deltime = 2000;
        }
      }
      chcount=1;
      deltime = 51*deltime/50;
      if(deltime>50000){ deltime = 2000; }
      setTimerFrequency(1000000/deltime);
    } else {
      chcount--;
    }
  }
}

void setTimerFrequency(int frequencyHz) {
  int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1; // calculate compare value
  TcCount16* TC = (TcCount16*) TC3;
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue; // Set Compare value
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

void startTimer(int frequencyHz) {
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

  setTimerFrequency(frequencyHz);

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
    if(mdir){
      // Increase (and limit) step count
      mcount+=mdir;
      if(mcount==1024){ mcount = 0; }
      if(mcount<0){ mcount = 1023; }
      // Get active coil direction
      if(mcount & 2){ stepdir = HIGH;
      } else { stepdir = LOW; }
      // Set active coil
      if(mcount & 1){
        digitalWrite(DIRA,stepdir);
        digitalWrite(PWRA,HIGH);
      } else {
        digitalWrite(DIRB,stepdir);
        digitalWrite(PWRB,HIGH);
      }
    }
  }
}
