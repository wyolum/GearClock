#if defined(ARDUINO) && ARDUINO >= 100
#include <SoftwareSerial.h>
#else
// #include <NewSoftSerial.h>
#endif
#include <TinyGPS.h>
#include "Time.h"
#include "Wire.h"
#include "rtcBOB.h"

/* This sample code demonstrates the normal use of a TinyGPS object.
   It requires the use of NewSoftSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 2(rx) and 3(tx).
*/

TinyGPS gps;
#if defined(ARDUINO) && ARDUINO >= 100
SoftwareSerial sws(5, 4);
#else
NewSoftSerial sws(5, 4);
#endif

#define THIRTY_YEARS 946771200
const int SQW_PIN = 2;
const int PPS_PIN = 3;
const int  PPS_LED = 13;
const int SYNC_LED = 12;
const int  SQW_LED = 11;

void gpsdump(TinyGPS &gps);
bool feedgps();
void printFloat(double f, int digits = 2);

void setup()
{
  Serial.begin(115200);
  sws.begin(9600);
  Wire.begin();
  
  Serial.print("Testing TinyGPS library v. "); Serial.println(TinyGPS::library_version());
  Serial.println("by Mikal Hart");
  Serial.println();
  Serial.print("Sizeof(gpsobject) = "); Serial.println(sizeof(TinyGPS));
  Serial.println();
  //setRTC(0);
  set_1Hz_ref(getTime(), SQW_PIN, rtc_interrupt, FALLING);  // attachInterrupt(0, rtc_interrupt, RISING);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(2, HIGH);
  digitalWrite(3, HIGH);
  digitalWrite(13, HIGH);
  delay(200);
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(13, LOW);

}

void loop()
{
  bool newdata = false;
  unsigned long age, start = millis();
  int Year;
  byte Month, Day, Hour, Minute, Second, Hundredths;
  tmElements_t tm;

  if (newdata){
    gps.crack_datetime(&Year, &Month, &Day, &Hour, &Minute, &Second, &Hundredths, &age);
    Serial.print("Date: "); Serial.print(static_cast<int>(Month)); Serial.print("/"); Serial.print(static_cast<int>(Day)); Serial.print("/"); Serial.print(Year);
    // Serial.print("  Time: "); Serial.print(static_cast<int>(Hour)); Serial.print(":"); Serial.print(static_cast<int>(Minute)); Serial.print(":"); Serial.print(static_cast<int>(Second)); Serial.print("."); Serial.print(static_cast<int>(Hundredths));
    // Serial.print("Second:"); Serial.println(Second, DEC);
    // Serial.print("second():"); Serial.println(second(), DEC);
    // Serial.print("millisecond():"); Serial.println(millisecond(), DEC);
    if(age < 500 && second() != Second || minute() != Minute || hour() != Hour ||
       day() != Day || month() != Month || year() != Year){
     coarse_sync();
    }
  }
  else{
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
    digitalWrite(13, LOW);
  }
}

void coarse_sync(){
  int Year;
  byte Month, Day, Hour, Minute, Second, Hundredths;
  unsigned long age;

  // TODO: check pps current
  gps.crack_datetime(&Year, &Month, &Day, &Hour, &Minute, &Second, &Hundredths, &age);
  Serial.print(second());
  Serial.print("!=");
  delay(1500 - age - 1); // RTC is 500ms behind GPS
  setRTC(Year, Month, Day, Hour, Minute, Second + 1);
  setTime(Hour, Minute, Second + 1, Day, Month, Year);
  Serial.print(second());
  Serial.print("=");
  Serial.print(Second + 1);
  Serial.println(" Coarse Sync");
  sws.end();
}
void printFloat(double number, int digits)
{
  // Handle negative numbers
  if (number < 0.0)
  {
     Serial.print('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  Serial.print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    Serial.print("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint; 
  } 
}

void gpsdump(TinyGPS &gps)
{
  long lat, lon;
  float flat, flon;
  unsigned long age, date, time, chars;
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned short sentences, failed;

  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  Serial.print("Date: "); Serial.print(static_cast<int>(month)); Serial.print("/"); Serial.print(static_cast<int>(day)); Serial.print("/"); Serial.print(year);
  Serial.print("  Time: "); Serial.print(static_cast<int>(hour)); Serial.print(":"); Serial.print(static_cast<int>(minute)); Serial.print(":"); Serial.print(static_cast<int>(second)); Serial.print("."); Serial.print(static_cast<int>(hundredths));
  Serial.print("  Fix age: ");  Serial.print(age); Serial.println("ms.");
  
}

bool feedgps()
{
  while (sws.available())
  {
    if (gps.encode(sws.read()))
      return true;
  }
  return false;
}

void rtc_interrupt(){
}
