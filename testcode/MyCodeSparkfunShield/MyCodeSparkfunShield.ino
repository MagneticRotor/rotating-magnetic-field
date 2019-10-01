/* My Code Sparkfun Shield
 *  
 *   # Tested with Baudouin's motor with 25 Ohm resistor in supply.
 *     Worked from 10 to 300RPM (use delayMicroseconds)
 *     
 * For timer and interrupt:
 *   # I need CTC (clear timer on compare match) mode and need interrupt on that.
 *   # need to set OCF2A flag set in the TIMSK2 register
 *   NAH forgett about it, just use mils and rounding as in the code below
 * 
 */

#define DIRA 12
#define PWRA 3
#define DIRB 13
#define PWRB 11
#define DTIME 5 // In microseconds

int mcount = 0; // motor count
unsigned long milnow = 0; // current millis
unsigned long milnext = 0; // Next mil counter at which step is called
unsigned long milbase = 0; // base count in milisecond
unsigned long mildelt = 0; // microseconds for next count
unsigned long deltime = 120494; // time between steps (microseconds)
unsigned long milprint = 0; // next print
int stepdir = 0; // step dir inside nextstep

void nextstep(){
  digitalWrite(PWRA, LOW);
  digitalWrite(PWRB, LOW);
  mcount++;
  if(mcount==1024){ mcount = 0; }
  if(mcount & 2){ stepdir = HIGH;
  } else { stepdir = LOW; }
  if(mcount & 1){
    digitalWrite(DIRA,stepdir);
    digitalWrite(PWRA,HIGH);
  } else {
    digitalWrite(DIRB,stepdir);
    digitalWrite(PWRB,HIGH);
  } 
}

void countup(){
  milnow = millis();
  if(milnow > milnext){
    // call the next step
    nextstep();
    //mcount++; // if you don't call nextstep
    // update mildelt and milnext
    mildelt += deltime;
    milnext = milbase + mildelt / 1000;
    // if mildelt getting too large (larger than 0.1s -> shrink)
    while(mildelt >= 10000){
      mildelt -= 10000;
      milbase += 10;
    }
    // if milnext == millis -> increase by one
    if(milnow > milnext){
      Serial.println("OVF");
      while(milnow > milnext){ milnext++; }
    }
  }
}

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
}

/* Steps
 * - Aon, Aoff
 * - Aon Bon
 * - A+ B+ A- B-
 */

void loop() {
  if(milprint < millis()){
    milprint += 2000;
    Serial.println(mcount);
    Serial.println(milprint);
    Serial.print(milbase); Serial.print(" "); Serial.println(mildelt);
  }
  countup();
}

void stepsManually(){
  // A+
  digitalWrite(DIRA, HIGH);
  digitalWrite(PWRA, HIGH);
  //Serial.println("A+");
  delay(DTIME);
  //delayMicroseconds(DTIME);
  digitalWrite(PWRA, LOW);
  // B+
  digitalWrite(DIRB, HIGH);
  digitalWrite(PWRB, HIGH);
  //Serial.println("B+");
  delay(DTIME);
  //delayMicroseconds(DTIME);
  digitalWrite(PWRB, LOW);
  // A-
  digitalWrite(DIRA, LOW);
  digitalWrite(PWRA, HIGH);
  //Serial.println("A-");
  delay(DTIME);
  //delayMicroseconds(DTIME);
  digitalWrite(PWRA, LOW);
  // B+
  digitalWrite(DIRB, LOW);
  digitalWrite(PWRB, HIGH);
  //Serial.println("B-");
  delay(DTIME);
  //delayMicroseconds(DTIME);
  digitalWrite(PWRB, LOW);
}
