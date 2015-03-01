#include <Wire.h>
#include "AFMotor.h"
#include "rtcBOB.h"
#include "Time.h"
#include "rtcBOB.h"

// Connect a stepper motor with 48 steps per revolution (7.5 degree)
// to motor port #2 (M3 and M4)
const int STEPS_PER_REV = 48;
AF_Stepper motor(STEPS_PER_REV, 2);
const int DT = 3600/STEPS_PER_REV;

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
}

void setup() {
  uint8_t n_tick;
  int i;
  time_t current_time, last_tick;

  Serial.begin(115200);
  Serial.println("Gear Clock");
  Serial.println("WyoLum 2014");
  Serial.println("Buy Open Hardware and own your future!");
  
  Wire.begin();
  // setRTC(10*3600 + 12 * 60);
  motor.setSpeed(40);  // 10 rpm   
  motor.step(1, FORWARD, SINGLE); // 48 steps
  motor.step(1, BACKWARD, SINGLE); // 48 steps
  // while(1) delay(100); // hold in place while setting alignment.
  
  /*
  // spin indef to test mechanics
  while(1){
    motor.step(STEPS_PER_REV, FORWARD, SINGLE); // 48 steps
    delay(200);
  }
  */
  pinMode(13, OUTPUT);
  //digitalWrite(13, HIGH);
  // delay(100);
  digitalWrite(13, LOW);

  // alarm time on RTC is the next tick time.
  current_time = getTime();
  // write_next_tick(current_time - 48 * DT); // ### DBG


  _next_tick = read_next_tick();

  last_tick = _next_tick - DT;
  // catch up to number of steps in cycle
  if((last_tick / DT) % 4 == 1){
    motor.step(1, FORWARD, SINGLE);
  }
  if((last_tick / DT) % 4 == 2){
    motor.step(2, FORWARD, SINGLE);
  }
  if((last_tick / DT) % 4 == 3){
    motor.step(1, BACKWARD, SINGLE);
  }
 
  
  if(current_time > _next_tick){
    last_tick = _next_tick;
    _next_tick = current_time - (current_time) % DT  + DT;
    n_tick = ((current_time - last_tick) % (86400 / 2)) / DT + 1;
    Serial.print("current_time + n_tick * DT: ");
    Serial.println(current_time + n_tick * DT);
    Serial.print("                         _next_tick: ");
    Serial.println(_next_tick);
    for(i = 0; i < n_tick; i++){
      motor.step(1, FORWARD, SINGLE);
      write_next_tick(_next_tick - (n_tick - 1) * DT);
    }
    Serial.print("_next_tick: ");
    Serial.println(_next_tick);
    write_next_tick(_next_tick);
    Serial.print("read_next_tick(): ");
    Serial.println(read_next_tick());
    Serial.print("_next_tick - getTime(): ");
    Serial.println(_next_tick - getTime());
  }
}

void loop(){
  int togo;
  uint8_t hh, mm, ss;
  motor.release();

  while(_next_tick > getTime()){
    togo = _next_tick - getTime();
    delay(1000);
    Serial.println(togo);
  }
  motor.step(1, FORWARD, SINGLE);
  _next_tick += DT;
  write_next_tick(_next_tick);
}
