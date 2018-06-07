/*
 Controlling a servo position using a potentiometer (variable resistor)
 by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

 modified on 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Knob
*/

#include <Servo.h>

Servo talonSE;  // create servo object to control a servo

int potpin = 0;  // analog pin used to connect the potentiometer 
int val;
int valMap;

void setup() {
  talonSE.attach(6);  // attaches the servo on pin 9 to the servo object
  Serial.begin(9600);
}

void loop() {
  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
  Serial.print(val);
  Serial.print("  ");
  valMap = map(val, 0, 1023, 90, 180);     // scale it to use it with the servo (value between 0 and 180)
  Serial.println(valMap);
  talonSE.write(valMap);                  // sets the servo position according to the scaled value
  delay(5);                           // waits for the servo to get there
}

