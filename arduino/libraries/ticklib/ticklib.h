#ifndef __TICKLIB_H
#define __TICKLIB_H
#include <Arduino.h>

const byte TICKS_PER_MINUTE = 200;
// long TICK_12_HR = TICKS_PER_MINUTE * 12 * 60;

class HMT{
 public:
  byte hour;
  byte minute;
  byte tick;
  HMT();
  HMT(byte hour, byte minute, byte tick);

  long toTicks();
  float toSeconds();
  bool greater(HMT other);
  bool equal(HMT other);
  void add(long n_tick);
  void print();
};

#endif
