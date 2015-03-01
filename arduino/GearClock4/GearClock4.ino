#include <Wire.h>
#include "Time.h"
#include "rtcBOB.h"
#include "ticklib.h"
#include "Adafruit_NeoPixel.h"

const int nENABLE_PIN = 8;
const int nSLEEP_PIN = 3;
const int STEP_PIN = 4;
#define ARTHERC
#ifdef ARTHERC
const int DIR_PIN = 9; // for ArtherC
const int M0_PIN = A1;
const int M1_PIN = A2;
#else
const int DIR_PIN = 5; // orig
const int M0_PIN = 6;
const int M1_PIN = 7;
#endif
const int DT_ms = 60000/TICKS_PER_MINUTE;
const bool CW = false;
const bool CCW = !CW;
const bool ENABLED = false;
const bool DISABLED = !ENABLED;
const int SW = A6;
const int SQW_LED = 13;
const int SQW_PIN = 2;

const bool ASLEEP = false;
const bool AWAKE = !ASLEEP;
const byte REAL_TIME = 8;
const byte SLOW = 8;
const byte FAST = 3;
const byte uSTEP = 31;

bool dir = CW;
bool mode = 0;
int step_count = 0;
int pw_ms = 2;
int pw_us = 2000L;

const long TICK_12_HR = (long)TICKS_PER_MINUTE * 12L * 60L;

void setSpeed(int speed){ // speed is FAST or SLOW
  pinMode(M0_PIN, (bool)(speed & 1 << 0));
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
      delayMicroseconds(pw_us / 2);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(pw_us / 2);
    }
  }
  digitalWrite(DIR_PIN, CW);
  digitalWrite(nENABLE_PIN, DISABLED);
}

void fasttick(long n_step, bool dir){
  // execute the number of steps the fastest way possible
  int pw_us = 1300;
  int min_pw = 1000;
  int ramp_steps = pw_us - min_pw;
  long i;
  unsigned long completed = 0;

  Serial.print("n_step: ");
  Serial.println(n_step);

  if(n_step < 2 * ramp_steps){
    ramp_steps = n_step / 2;
  }

  setSpeed(FAST);
  digitalWrite(DIR_PIN, dir);
  digitalWrite(nENABLE_PIN, ENABLED);
  
  // ramp up
  for(i = 0; i < ramp_steps; i++){
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(pw_us / 2);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(pw_us / 2);
    pw_us--;
    completed++;
  }
  
  Serial.print("completed: ");
  Serial.println(completed);

  // cruse
  for(i=ramp_steps; i < n_step - (long)ramp_steps; i++){
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(pw_us / 2);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(pw_us / 2);
    completed++;
  }
  Serial.print("completed: ");
  Serial.println(completed);

  // ramp_down
  for(i = 0; i < ramp_steps; i++){
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(pw_us / 2);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(pw_us / 2);
    pw_us++;
    completed++;
  }

  digitalWrite(nENABLE_PIN, DISABLED);
  setSpeed(SLOW);
  digitalWrite(DIR_PIN, CW);
  Serial.print("completed: ");
  Serial.println(completed);
}

HMT get_now_tick(){
  float fnow = getTime() % 86400 + millisecond() / 1000.;
  byte hh, mm, ttt;
  
  hh = fnow / 3600;
  fnow -= (long)hh * 3600;

  mm = fnow / 60;
  fnow -= mm * 60;
  
  ttt = (fnow * TICKS_PER_MINUTE) / 60;
  return HMT(hh, mm, ttt);
}

HMT read_next_tick(){
  byte buff[3];
  rtc_raw_read(DS3231_ALARM1_OFSET, 3, false, buff);
  return HMT(buff[0], buff[1], buff[2]);
}

void write_next_tick(HMT hmt){
  uint8_t *time_bytes_p;
  byte hours_minutes_tick[3];
  
  hours_minutes_tick[0] = hmt.hour;
  hours_minutes_tick[1] = hmt.minute;
  hours_minutes_tick[2] = hmt.tick;
  
  time_bytes_p = (uint8_t*)(hours_minutes_tick);  
  rtc_raw_write(DS3231_ALARM1_OFSET, 3, false, time_bytes_p);
}

bool sqw_led_state = true;
unsigned int rtc_start_us = 0;
void rtc_interrupt(){
  long  now_us = micros();
  long drift_us;

  if(sqw_led_state){
    sqw_led_state = false;
    PORTB &= ~0b00001000;
  }
  else{
    sqw_led_state = true;
    PORTB |= 0b00001000;
  }
  rtc_start_us = now_us;
  digitalWrite(SQW_LED, sqw_led_state); // not required.
}

/*
 * convert from hours:minutes:steps to seconds past midnight floating point
 */
float toFloatTime(byte *hms){
  return (hms[0] * 3600. + 
	  hms[1] * 60. + 
	  hms[2] * DT_ms / 1000.);  
}

float getFloatTime(){
  byte hms[3];
  float out = 0.;
  hms[0] = hour();
  hms[1] = minute();
  hms[2] = (second() * 1000 + millisecond()) / DT_ms;
  Serial.print("HH : MM : STEP ");
  
  for(int i=0; i<3; i++){
    Serial.print(" : ");
    Serial.print(hms[i]);
  }
  out = toFloatTime(hms);
  Serial.print(" :: ");
  Serial.println(out);
  Serial.println();
  return out;
}

HMT next_tick;
const int LED_PIN = 13;
 Adafruit_NeoPixel led = Adafruit_NeoPixel(1, LED_PIN, NEO_GRB + NEO_KHZ800);
void led_setup(){
  led.begin();
  led.show(); // Initialize all pixels to 'off'
}

void led_set(uint32_t c) {
  led.setPixelColor(0, c);
  led.show();
}

void setup(){
  HMT now_tick;

  Wire.begin();
  led_setup();
  led_set(255);

  pinMode(SW, INPUT); 
  pinMode(STEP_PIN, OUTPUT);
  pinMode(nENABLE_PIN, OUTPUT);
  pinMode(nSLEEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  pinMode(SQW_LED, OUTPUT);
  pinMode(SQW_PIN, INPUT);


  digitalWrite(nSLEEP_PIN, AWAKE);
  digitalWrite(nENABLE_PIN, ENABLED);

  digitalWrite(M0_PIN, HIGH);
  digitalWrite(M1_PIN, HIGH);
  digitalWrite(SW, HIGH); 
  digitalWrite(SQW_LED, HIGH); 
  digitalWrite(DIR_PIN, CW);

  Serial.begin(115200);
  Serial.println("Buy Open Hardware and own the future!");
  enable_sqw();
  set_1Hz_ref(getTime(), SQW_PIN, rtc_interrupt, FALLING); 
  
  next_tick = read_next_tick();
  now_tick = get_now_tick();
  Serial.print("NEXT ");
  next_tick.print();
  Serial.print("NOW ");
  now_tick.print();
  Serial.println(now_tick.greater(next_tick));
  if((next_tick.hour > 23) ||
     (next_tick.minute > 60) ||
     (next_tick.tick > 200)){ // this clock must be brand new!  set clock sto 12:00 midnight
    write_next_tick(HMT(0, 0, 0));
    setTime(0);
  }
  if(next_tick.greater(now_tick)){ // should be less than.. cant reboot in less than a tick
    next_tick.hour += 24;
  }

  long n_tick = now_tick.toTicks() - next_tick.toTicks();
  Serial.print("n_tick:");
  Serial.println(n_tick);
  long spin_tick = n_tick % TICK_12_HR;
  if(spin_tick > TICK_12_HR / 2){
    spin_tick = TICK_12_HR - spin_tick;
    dir = CCW;
  }
  else{
    dir = CW;
  }
  Serial.print("n_hrs:");
  Serial.println(spin_tick * DT_ms / 1000. / 3600.);
  if(spin_tick < TICKS_PER_MINUTE){
    slowtick(spin_tick, dir);
  }
  else{
    fasttick(spin_tick, dir);
  }
  next_tick.add(n_tick);

  now_tick = get_now_tick();
  Serial.print("NEXT ");
  next_tick.print();
  Serial.print("NOW ");
  now_tick.print();
  while(get_now_tick().greater(next_tick)){
    slowtick(1, CW);
    next_tick.add(1);
  }
  now_tick = get_now_tick();
  Serial.print("NEXT ");
  next_tick.print();
  Serial.print("NOW ");
  now_tick.print();
  next_tick.add(1);
  write_next_tick(next_tick);
}

const byte uSTEPS = 32;

const int    B_NONE= 0;
const int     B_UP = 1;
const int   B_DOWN = 2;
const int B_MIDDLE = 3;
const int   B_LEFT = 4;
const int  B_RIGHT = 5;

int read_buttons(){
#ifndef ARTHERC
  int reading = analogRead(SW);
  int out;
  if (reading > 900){
    out = B_UP;
  }
  else if (reading > 700){
    out = B_DOWN;
  }
  else if (reading > 500){
    out = B_MIDDLE;
  }
  else if (reading > 300){
    out = B_LEFT;
  }
  else if (reading > 100){
    out = B_RIGHT;
  }
  else{
    out = B_NONE;
  }
  return out;
#else
  return 0;
#endif
}

byte speed = REAL_TIME;
void loop(){
  while(next_tick.greater(get_now_tick())){
  }
  slowtick(1, CW);
  next_tick.add(1);
  write_next_tick(next_tick);
  next_tick.print();
  return;
}
