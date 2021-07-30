// Sundial automatically corrected to civil time

#include <Arduino.h>
#include <time.h>
#include "Stepper.h"
#include "LowPower.h"
#include "RTClib.h" // Date and time functions using a DS3231 RTC connected via I2C and Wire lib
#include <util/usa_dst.h> //utility to calculate whether the current date is in daylight saving time
#include <util/eu_dst.h>
#define MYDST usa_dst //Daylight Saving Time zone: change if in the EU to eu_dst

const float lat = 41.76; //degrees latitude N is positive
const float lon = -73.1; //degrees longitude E is positive
const int time_zone = -5;       //EST (for NY) = -5, Don't asjust for daylight saving time here

const int CLOCK_INTERRUPT_PIN = 2; //must be a pin that works with interrupts
const int IN1Pin = 3; //pin to connect to the ULN2003 driver module
const int IN2Pin = 4;
const int IN3Pin = 5;
const int IN4Pin = 6;
const int leftPin = 7; //pins used to adjust sundial on startup
const int rightPin = 8; 
const int donePin = 11;
const int stepsPerRevolution = 2048; //depends on the stepper motor you use
const int teethPerRevolution = 12; //# of teeth onthe 3D printed gear that is on the stepper motor
const float mmPerTooth = 2.8; //the distance in mm between teeth on the linear gear (rack) glued to the sundial
                              //in practice I had to tweak this number to get it to match actual movement
const int mmPerHour = 20;  //the distance between hour markings on the sundial in mm
const int motorDirection = -1; //1 = CW, -1 = CCW
const int minPerButtonPress = 1; //how far to move the sundial on each button press during startup routine

const float stepsPerMin = stepsPerRevolution / teethPerRevolution / mmPerTooth * mmPerHour / 60 * motorDirection;
Stepper myStepper = Stepper(stepsPerRevolution, IN1Pin, IN3Pin, IN2Pin, IN4Pin);

const bool DEBUG = true;
int DEBUGOffset[] = {0, 3600, -1800, -900, 1800};
int DEBUGi = 0;
int lastOffset = 0;

// Real Time Clock (RTC) uses I2C: 
//I2C on Uno SDA and SCL pins (pins near the AREF pin) on Nano: SCL is pin A5 SDA is pin A4)
RTC_DS3231 rtc;
DateTime now;

void startRTC();
void setAlarm();
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

  setUpStepper();
  manuallyCenterDial();
  startRTC();
  now = rtc.now();
  if (DEBUG){
    Serial.begin(9600);
    Serial.print("Today is: ");
    printDateTime();
    int tmpsec = secondsToAdjust();
    Serial.println(tmpsec);
  }
    
  moveToOffset(secondsToAdjust());
}

void loop() {
  //go into low power mode and wake up on an alarm from the RTC.
  if (DEBUG){
    printDateTime();
    Serial.println("going to sleep"); 
  } 
  attachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN), onAlarm, FALLING);
  if (DEBUG) Serial.flush();
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
  detachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN));  //disable interrupts until after you deal with the first.
  printDateTime();
  rtc.disableAlarm(1);
  rtc.clearAlarm(1);
  if (DEBUG) Serial.println("awake and moving to offset ");
  moveToOffset(secondsToAdjust()); //move dial on sundial 
  delay(1000);
  setAlarm(); 
}

void startRTC() {
  if (! rtc.begin()) {
    if (DEBUG) Serial.println("Couldn't find RTC");
    if (DEBUG) Serial.flush();
    abort();
  }

  if (rtc.lostPower()) {
    if (DEBUG) Serial.println("RTC lost power, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  //configure the RTC for alarms
  rtc.disable32K();
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);
  rtc.writeSqwPinMode(DS3231_OFF);  // stop oscillating signals at SQW Pin otherwise setAlarm1 will fail
  rtc.disableAlarm(2);
  setAlarm();
}

void printDateTime(){
  now = rtc.now();
  delay(100);
  Serial.print(now.year()); Serial.print('/');
  Serial.print(now.month()); Serial.print('/');
  Serial.print(now.day()); Serial.print(' ');
  Serial.print(now.hour()); Serial.print(':');
  Serial.print(now.minute()); Serial.print(':');
  Serial.println(now.second());
}

int secondsToAdjust() {
  /*  if (DEBUG) {
    int offset = DEBUGOffset[DEBUGi];
    DEBUGi = (DEBUGi + 1) % (sizeof(DEBUGOffset)/sizeof(DEBUGOffset[0]));
    return offset;
  }
  */  
  //Uses global variable now, which is the current datetime from RTC
  //Return: number of seconds to move the sundial markings to adjust from solar noon to civil noon
  //Functions to get solar and civil noon are implemented in avr-libc, the C library for AVR platform
  //that underpins the Arduino core library.
  time_t now_t, solarNoon_t, civilNoon_t; //AVRLibc timestamp seconds since Y2K
  struct tm tmp_time;  //AVRLibc time structure to hold time components
 
  
  set_dst(MYDST); //set function to calculate whether it is daylight saving time
  set_zone(time_zone * ONE_HOUR);
  set_position(lat * ONE_DEGREE, lon * ONE_DEGREE);
 
  // convert RTC datetime to timestamp 
  tmp_time.tm_year = now.year() - 1900; //years since 1900
  tmp_time.tm_mon = now.month() - 1; //Jan = 0
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
  if (DEBUG) {
    Serial.print("Solar noon for today: ");
    Serial.println(ctime(&solarNoon_t));
    Serial.print("Seconds to shift: ");
    Serial.println(secondsToShift);
  }
  return (int) secondsToShift;
}

void onAlarm() {

}
void setUpStepper() {
  myStepper.setSpeed(5); // Set the speed to 5 rpm:
}

void manuallyCenterDial() {
  //To allow the user to adjust the dial to the 12:00 mark when the sundial is powered up
  //The user pushes a left and right button to get the dial centered and then presses the "done" button.
  while (digitalRead(donePin) == HIGH) {
    if (digitalRead(leftPin) == LOW) {
      myStepper.step(- minPerButtonPress * stepsPerMin);
    } else if (digitalRead(rightPin) == LOW) {
      myStepper.step(minPerButtonPress * stepsPerMin);
    }
  }
}

void moveToOffset(int currentOffset) {
  if (DEBUG) {
    Serial.print("Moving to offset ");
    Serial.print(currentOffset); 
    Serial.print(" by moving ");
    Serial.println(( currentOffset - lastOffset) * stepsPerMin / 60);
    Serial.println(" steps.");
  }
  myStepper.step(( currentOffset - lastOffset) * stepsPerMin / 60);  
  lastOffset = currentOffset;
  digitalWrite(IN1Pin,LOW);  //Turn off the motor pins to save power.
  digitalWrite(IN2Pin,LOW);
  digitalWrite(IN3Pin,LOW);
  digitalWrite(IN4Pin,LOW);
}

void setAlarm(){
  if (DEBUG) {
    Serial.print("Setting the alarm for every minute ");
    printDateTime();
    rtc.setAlarm1(DateTime(2015, 3, 14, 5, 30, 0), DS3231_A1_Second);   //alarm every minute on the minute
  } else {
    rtc.setAlarm1(DateTime(2015, 3, 14, 5, 30, 0), DS3231_A1_Hour);   //alarm at 5:30 every morning (year and date ignored)
  }
}