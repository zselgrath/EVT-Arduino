/*
 * TimeRTC.pde
 * Example code illustrating Time library with Real Time Clock.
 *
 * Use SyncArduinoClock.pde sketch to set time (need Processing 3 installed)
 * This sketch can be found in the teensy time library folder
 */

//UNCOMMENT THE BELOW LINE IF TRYING TO SET THE RTC
//#define SET_TIME

#include <Arduino.h>
#include <TimeLib.h>

#ifndef SET_TIME
//HEY JOE PUT YOUR CODE HERE
#endif

#ifdef SET_TIME
time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

void printDigits(int digits)
{
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void digitalClockDisplay()
{
  digitalWrite(13, HIGH);
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year());
  Serial.println();
  digitalWrite(13, LOW);
}



/*  code to process time sync messages from the serial port   */
#define TIME_HEADER "T" // Header tag for serial time sync message

unsigned long processSyncMessage()
{
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

  if (Serial.find(TIME_HEADER))
  {
    pctime = Serial.parseInt();
    return pctime;
    if (pctime < DEFAULT_TIME)
    {              // check the value is a valid time (greater than Jan 1 2013)
      pctime = 0L; // return 0 to indicate that the time is not valid
    }
  }
  return pctime;
}



void setup()
{
  pinMode(13, OUTPUT);
  
  // set the Time library to use Teensy 3.0's RTC to keep time
  setSyncProvider(getTeensy3Time);
  
  Serial.begin(115200);
  while (!Serial)
    ; // Wait for Arduino Serial Monitor to open
  delay(100);
  if (timeStatus() != timeSet)
  {
    Serial.println("Unable to sync with the RTC");
  }
  else
  {
    Serial.println("RTC has set the system time");
  }
}

int loops = 5;

void loop()
{
  
  if (Serial.available())
  {
    time_t t = processSyncMessage();
    if (t != 0)
    {
      Teensy3Clock.set(t); // set the RTC
      setTime(t);
    }
  }
   
  if(loops>1){loops -= 1;}
  else{
    loops = 5;
    adjustTime(3600L);
    }
  digitalClockDisplay();
}
#endif