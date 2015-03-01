#include <Wire.h>
#include "rtcBOB.h"
#include "Time.h"

const int nENABLE_PIN = 2;
const int nSLEEP_PIN = 3;
const int STEP_PIN = 4;
const int DIR_PIN = 5;
const int M0_PIN = 6;
const int M1_PIN = 7;
const int N_STEP = 200;
const bool CW = true;
const bool CCW = !CW;
const bool ENABLED = false;
const bool DISABLED = !ENABLED;
const byte SLOW = 8;
const byte FAST = 3;
const byte uSTEP = 31;

// Connect a stepper motor with 48 steps per revolution (7.5 degree)
// to motor port #2 (M3 and M4)
const int STEPS_PER_REV = 200;

const int DT = 3600/STEPS_PER_REV;
const unsigned int PULSE_DURATION_uS = 2000;

void setSpeed(int speed){ // speed is FAST or SLOW
  pinMode(M0_PIN, (bool)
(speed & 1 << 0));
  pinMode(M1_PIN, (bool)(speed & 1 << 1));
  digitalWrite(M0_PIN, (bool)(speed & 1 << 2));
  digitalWrite(M1_PIN, (bool)(speed & 1 << 3));
}

void slowtick(long n_tick, bool dir){
  setSpeed(SLOW);
  digitalWrite(DIR_PIN, dir);
  digitalWrite(nENABLE_PIN, ENABLED);

  for(int i=0; i < n_tick; i++){
    for(int j=0; j < uSTEP; j++){
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(PULSE_DURATION_uS / 2);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(PULSE_DURATION_uS / 2);
    }
  }
  digitalWrite(DIR_PIN, CW);
  digitalWrite(nENABLE_PIN, DISABLED);
}

void step(bool dir){
  digitalWrite(nENABLE_PIN, ENABLED); 
  digitalWrite(DIR_PIN, dir);
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(PULSE_DURATION_uS / 2);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(PULSE_DURATION_uS / 2);
}

void step(bool dir, int n){
  for(int i=0; i<n; i++){
    step(dir);
  }
}

time_t _next_tick = 0;

time_t read_next_tick(){
  uint8_t *time_bytes_p;
  time_t out;
  
  time_bytes_p = (uint8_t*)(&out);  
  rtc_raw_read(DS3231_ALARM1_OFSET, 4, false, time_bytes_p);
  return out;
}

time_t write_next_tick(time_t next_tick){
  uint8_t *time_bytes_p;
  
  time_bytes_p = (uint8_t*)(&next_tick);  
  rtc_raw_write(DS3231_ALARM1_OFSET, 4, false, time_bytes_p);
  Serial.print("next_tick written: ");
  Serial.println(next_tick);
}

void setup() {
  uint8_t n_tick;
  int i;
  time_t current_time, last_tick;
  setSpeed(FAST);
  Serial.begin(115200);
  Serial.println("Gear Clock 3");
  Serial.println("WyoLum 2014");
  Serial.println("Buy Open Hardware and own your future!");
  Serial.print("DT: ");
  Serial.println(DT);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(nENABLE_PIN, OUTPUT);
  pinMode(nSLEEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(nENABLE_PIN, ENABLED);
  digitalWrite(nSLEEP_PIN, LOW);

  Wire.begin();
  Serial.println("Wire initialized");
  // setRTC(10*3600 + 12 * 60);
  // step(CCW);
  // while(1) delay(100); // hold in place while setting alignment.
  
  // spin indef to test mechanics
  /*
  while(1){
    for(i = 0; i < STEPS_PER_REV; i++){
      step(CW);
    }
    delay(800);
  }
  */
  pinMode(13, OUTPUT);
  //digitalWrite(13, HIGH);
  // delay(100);
  digitalWrite(13, LOW);

  // alarm time on RTC is the next tick time.
  Serial.print("current_time... ");
  current_time = getTime();
  Serial.println(current_time);

  // write_next_tick(getTime() - 3600);
  _next_tick = read_next_tick();
  if (_next_tick > getTime() && _next_tick - getTime() > DT){
    _next_tick = getTime(); // strange behavior
    Serial.println("Resetting time, _next_tick too far in future?!");
  }

  last_tick = _next_tick - DT;
  Serial.print("   last_tick: ");
  Serial.println(last_tick);
  Serial.print("  _next_tick: ");
  Serial.println(_next_tick);
  // catch up to number of steps in cycle
  if((last_tick / DT) % 4 == 1){
    step(CW);
  }
  if((last_tick / DT) % 4 == 2){
    step(CW);
    step(CW);
  }
  if((last_tick / DT) % 4 == 3){
    step(CW);
  }
 
  // for(int ii=0; ii < 200; ii++) step(1, CW);
  
  if(current_time > _next_tick){
    last_tick = _next_tick;
    _next_tick = current_time - (current_time) % DT  + DT;
    n_tick = ((current_time - last_tick) % (86400 / 2)) / DT + 1;
    Serial.print("current_time + n_tick * DT: ");
    Serial.println(current_time + n_tick * DT);
    Serial.print("                         _next_tick: ");
    Serial.println(_next_tick);
    for(i = 0; i < n_tick; i++){
      step(CW);
      write_next_tick(_next_tick - (n_tick - 1) * DT);
    }
  }
}

void loop(){
  int togo;
  uint8_t hh, mm, ss;

  digitalWrite(nENABLE_PIN, DISABLED); 
  while(_next_tick > getTime()){
    togo = _next_tick - getTime();
    delay(1000);
    Serial.print("_next_tick:");
    Serial.print(_next_tick);
    Serial.print(", getTime():");
    Serial.print(getTime());
    Serial.print(", togo ");
    Serial.println(togo);
  }
  // slowtick(1, CW);
  step(CW);
  _next_tick += DT;
  write_next_tick(_next_tick);
}
