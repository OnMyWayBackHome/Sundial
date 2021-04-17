// Sundial corrected for civil time

#include <Arduino.h>
#include <SPI.h>
#include <time.h>
#include <Math.h>
#include "Stepper.h"
#include "RTClib.h" // Date and time functions using a DS3231 RTC connected via I2C and Wire lib
#include <util/usa_dst.h> //utility to calculate whether the current date is in daylight saving time
#include <util/eu_dst.h>
#define MYDST usa_dst //change if in the EU to eu_dst

const float lat = 42.7335157; //degrees latitude N is positive
const float lon = -73.6876147; //degrees longitude E is positive
const int time_zone = -5;       //EST (for NY) = -5, Don't asjust for daylight saving time here

const int CLOCK_INTERRUPT_PIN = 2; //must be a pin that works with interrupts
const int IN1Pin = 3; //pin to connect to the ULN2003 driver module
const int IN2Pin = 4;
const int IN3Pin = 5;
const int IN4Pin = 6;
const int stepsPerRevolution = 2048;
const int teethPerRevolution = 12; //of the 3D printed gear that is on the stepper motor
const float mmPerTooth = 1.6; //the distance between teeth on the linear gear glued to the sundial
const int mmPerHour = 20;  //the distance between hour markings on the sundial
Stepper myStepper = Stepper(stepsPerRevolution, IN1Pin, IN3Pin, IN2Pin, IN4Pin);

// Real Time Clock (RTC) uses I2C: 
//I2C on Uno SDA and SCL pins (pins hear the AREF pin) on Nano: SCL is pin A5 SDA is pin A4)
RTC_DS3231 rtc;
DateTime now;

void startRTC();
void printDateTime();
int minutesToAdjust();
int calculateDayOfYear(int day, int month, int year);
int equationOfTime(int doy);
void onAlarm();

void setup() {
  myStepper.setSpeed(5); // Set the speed to 5 rpm:
  
  Serial.begin(9600);
  startRTC();
  rtc.disable32K();
  pinMode(CLOCK_INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN), onAlarm, FALLING);
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);
  rtc.writeSqwPinMode(DS3231_OFF);
  rtc.disableAlarm(2);
  now = rtc.now();
  Serial.print("Today is: ");
  printDateTime();
  minutesToAdjust();

  /*
  int doy = calculateDayOfYear(now.day(), now.month(), now.year());
  Serial.print("Day of year: ");
  Serial.println(doy);
  int eot = equationOfTime(doy);
  Serial.print("My eot calculation: ");
  Serial.println(eot);
  */
}

void loop() {
  
  // Step one revolution in one direction:
  Serial.println("clockwise");
  myStepper.step(stepsPerRevolution);
  delay(3000);
  
  // Step one revolution in the other direction:
  Serial.println("counterclockwise");
  myStepper.step(-stepsPerRevolution);
  delay(500);
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

int calculateDayOfYear(int day, int month, int year) {

  // by jrleeman https://gist.github.com/jrleeman/3b7c10712112e49d8607
  // Given a day, month, and year (4 digit), returns 
  // the day of year. Errors return 999.
  
  int daysInMonth[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  
  // Verify we got a 4-digit year
  if (year < 1000) {
    return 999;
  }
  
  // Check if it is a leap year, this is confusing business
  // See: https://support.microsoft.com/en-us/kb/214019
  if (year%4  == 0) {
    if (year%100 != 0) {
      daysInMonth[1] = 29;
    }
    else {
      if (year%400 == 0) {
        daysInMonth[1] = 29;
      }
    }
   }

  // Make sure we are on a valid day of the month
  if (day < 1) 
  {
    return 999;
  } else if (day > daysInMonth[month-1]) {
    return 999;
  }
  
  int doy = 0;
  for (int i = 0; i < month - 1; i++) {
    doy += daysInMonth[i];
  }
  
  doy += day;
  return doy;
}

int equationOfTime(int doy) {
  //calculates approximate equation of time
  //Input the day of the year
  //Returns: the number of minutes the actual sun is ahead (positive) or behind (neg) the mean sun
  doy--; //these calculations call Jan 1, day 0.
  float w = TWO_PI / 365.24; //angular velocity of earth in radians/day
  float a = w * (doy + 10); //angle moved by earth at mean spead note: approx 10 days from solstice to Jan 1
  float b = a + 2 * 0.0167 * sin(w * (doy - 2)); //angle plus correction for eccentricity
  float c = (a - atan(tan(b) / cos(PI * 23.44 / 180)))/ PI;
  int eot = 720 * (c - int(c + 0.5));
  return eot;
}

int minutesToAdjust(){
  //Uses global variable now, which is the current datetime from RTC
  //Return: number of minutes to move the sundial markings to adjust from solar noon to civil noon
  time_t now_t, solarNoon_t, civilNoon_t; //AVRLibc timestamp seconds since Y2K
  struct tm tmp_time, noon, lt; //AVRLibc time structure to hold time components
  char buff[26];
  
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

/*  localtime_r(&now_t,&lt);
  Serial.println();
  Serial.println();
  Serial.print("Local time");
  Serial.print(isotime(&lt));
  if(lt.tm_isdst > 0) {
    Serial.println(" Daylight Savings is in effect");
  } else {
    Serial.println(" Daylight Savings is NOT in effect");
  }
  //Serial.write(10);
 

  Serial.print("Equation of time AVR Libc: ");
  Serial.println(equation_of_time(&now_t) / 60);
*/
  solarNoon_t = solar_noon(&now_t);
  Serial.print("Minutes to shift ");
  
  long  minutesToShift = ((long) solarNoon_t - (long) civilNoon_t)/60;
  Serial.println(minutesToShift);
/* 
  localtime_r(&solarNoon_t,&noon);
  isotime_r(&noon,buff);
  Serial.print("Solar noon : ");
  Serial.print(&buff[11]);
  Serial.println(" local time");
  Serial.write(10);
  */
  return (int) minutesToShift;
}

void onAlarm() {
  
}