// Sundial corrected for civil time

#include <Arduino.h>
#include <SPI.h>
#include <time.h>
#include <Math.h>
#include "Stepper.h"
#include "RTClib.h" // Date and time functions using a DS3231 RTC connected via I2C and Wire lib
#include <util/usa_dst.h> //utility to calculate whether the current date is in daylight saving time
#include <util/eu_dst.h>
#define MYDST usa_dst //Daylight Saving Time zone: change if in the EU to eu_dst

const float lat = 42.7335157; //degrees latitude N is positive
const float lon = -73.6876147; //degrees longitude E is positive
const int time_zone = -5;       //EST (for NY) = -5, Don't asjust for daylight saving time here

const int CLOCK_INTERRUPT_PIN = 2; //must be a pin that works with interrupts
const int IN1Pin = 3; //pin to connect to the ULN2003 driver module
const int IN2Pin = 4;
const int IN3Pin = 5;
const int IN4Pin = 6;
const int leftPin = 7; //pins used to adjust sundial on startup
const int donePin = 8;
const int rightPin = 9; 
const int stepsPerRevolution = 2048;
const int teethPerRevolution = 12; //# of teeth onthe 3D printed gear that is on the stepper motor
const float mmPerTooth = 2.8; //the distance between teeth on the linear gear (rack) glued to the sundial
const int mmPerHour = 20;  //the distance between hour markings on the sundial
const int minPerButtonPress = 1; //how far to move the sundial on button press during startup routine
const float stepsPerMin = stepsPerRevolution / teethPerRevolution / mmPerTooth * mmPerHour / 60;
Stepper myStepper = Stepper(stepsPerRevolution, IN1Pin, IN3Pin, IN2Pin, IN4Pin);
int DEBUG = 1;
int TESTOffset[] = {0, 3600, -1800, -900, 1800};
int TESTi = 0;
int lastOffset = 0;

// Real Time Clock (RTC) uses I2C: 
//I2C on Uno SDA and SCL pins (pins hear the AREF pin) on Nano: SCL is pin A5 SDA is pin A4)
RTC_DS3231 rtc;
DateTime now;

void startRTC();
void printDateTime();
void setUpStepper();
void manuallyCenterDial();
int secondsToAdjust();
void moveToOffset(int off);
void onAlarm();

void setup() {
  pinMode(leftPin, INPUT_PULLUP);
  pinMode(rightPin, INPUT_PULLUP);
  pinMode(donePin, INPUT_PULLUP);
  pinMode(CLOCK_INTERRUPT_PIN, INPUT_PULLUP);

  Serial.begin(9600);
  setUpStepper();
  manuallyCenterDial();
  startRTC();

  rtc.disable32K();
  attachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN), onAlarm, FALLING);
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);
  rtc.writeSqwPinMode(DS3231_OFF);
  rtc.disableAlarm(2);
  now = rtc.now();
  Serial.print("Today is: ");
  printDateTime();
  moveToOffset(secondsToAdjust());
}

void loop() {
  //every day, check for a new offset
  moveToOffset(secondsToAdjust());
  delay(2000);
}

void startRTC() {
   if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
}

void printDateTime(){
  now = rtc.now();
  Serial.print(now.year()); Serial.print('/');
  Serial.print(now.month()); Serial.print('/');
  Serial.print(now.day()); Serial.print(' ');
  Serial.print(now.hour()); Serial.print(':');
  Serial.print(now.minute()); Serial.print(':');
  Serial.println(now.second()); Serial.print('\t');
}

int secondsToAdjust() {
  if (DEBUG == 1) {
    int offset = TESTOffset[TESTi];
    TESTi = (TESTi + 1) % (sizeof(TESTOffset)/sizeof(TESTOffset[0]));
    return offset;
  }
  //Uses global variable now, which is the current datetime from RTC
  //Return: number of minutes to move the sundial markings to adjust from solar noon to civil noon
  time_t now_t, solarNoon_t, civilNoon_t; //AVRLibc timestamp seconds since Y2K
  struct tm tmp_time; //noon, lt; //AVRLibc time structure to hold time components
  //char buff[26];
  
  set_dst(MYDST); //set function to calculate whether it is daylight saving time
  set_zone(time_zone * ONE_HOUR);
  set_position(lat * ONE_DEGREE, lon * ONE_DEGREE);
 
  // convert RTC datetime to timestamp 
  tmp_time.tm_year = now.year() - 1900; //years since 1900 for some reason
  tmp_time.tm_mon = now.month() - 1; //Jan = 0 for some reason
  tmp_time.tm_mday = now.day();
  tmp_time.tm_hour = now.hour();
  tmp_time.tm_min = now.minute();
  tmp_time.tm_sec = now.second();
  now_t = mktime(&tmp_time);

  tmp_time.tm_hour = 12;
  tmp_time.tm_min = 0;
  tmp_time.tm_sec = 0;
  civilNoon_t = mktime(&tmp_time);
  solarNoon_t = solar_noon(&now_t);
  long  secondsToShift = (long) solarNoon_t - (long) civilNoon_t;
  Serial.print("Seconds to shift ");
  Serial.println(secondsToShift);
/* 
  localtime_r(&solarNoon_t,&noon);
  isotime_r(&noon,buff);
  Serial.print("Solar noon : ");
  Serial.print(&buff[11]);
  Serial.println(" local time");
  Serial.write(10);
  */
  return (int) secondsToShift;
}

void onAlarm() {
}
void setUpStepper() {
  myStepper.setSpeed(5); // Set the speed to 5 rpm:
}

void manuallyCenterDial() {
  while (digitalRead(donePin) == HIGH) {
    if (digitalRead(leftPin) == LOW) {
      myStepper.step(- minPerButtonPress * stepsPerMin);
    } else if (digitalRead(rightPin) == LOW) {
      myStepper.step(minPerButtonPress * stepsPerMin);
    }
  }
}

void moveToOffset(int currentOffset) {
  Serial.print("moving to ");
  Serial.print(currentOffset); 
  Serial.print(" by moving ");
  Serial.println(( currentOffset - lastOffset) * stepsPerMin / 60);
  myStepper.step(( currentOffset - lastOffset) * stepsPerMin / 60);  
  lastOffset = currentOffset;
}