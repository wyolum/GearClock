#include "ticklib.h"

HMT::HMT(){
  hour = 0;
  minute = 0;
  tick = 0;
}

HMT::HMT(byte _hour, byte _minute, byte _tick){
  hour = _hour;
  minute = _minute;
  tick = _tick;
}

long HMT::toTicks(){
  return ((long)hour * 60L  + minute) * (long)TICKS_PER_MINUTE + tick;
}

float HMT::toSeconds(){
  return hour * 3600 + minute * 60 + tick / float(TICKS_PER_MINUTE);
}

bool HMT::greater(HMT other){
  return toTicks() > other.toTicks();
}

bool HMT::equal(HMT other){
  return toTicks() == other.toTicks();
}

void HMT::print(){
  if(hour < 10){
    Serial.print("0");
  }
  Serial.print(hour);

  Serial.print(":");

  if(minute < 10){
    Serial.print("0");
  }
  Serial.print(minute);
  Serial.print("::");

  if(tick < 10){
    Serial.print("0");
  }
  if(tick < 100){
    Serial.print("0");
  }
  Serial.print(tick);
  Serial.println();
}

void HMT::add(long n_tick){
  byte hh, mm, ttt;
  
  n_tick += toTicks();

  ttt = n_tick % TICKS_PER_MINUTE;
  n_tick -= ttt;

  mm += (n_tick / TICKS_PER_MINUTE) % 60;
  n_tick -= mm * TICKS_PER_MINUTE;
  hh += n_tick / (TICKS_PER_MINUTE * 60);

  hour = hh;
  minute = mm;
  tick = ttt;
}
