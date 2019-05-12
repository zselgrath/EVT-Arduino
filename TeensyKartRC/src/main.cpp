//Includes
#include <Arduino.h>
#include <SatelliteReceiver.h>

//Defines
#define ledPin 13
SatelliteReceiver SpektrumRx;

//Setup
void setup(){
  pinMode(ledPin, OUTPUT);
  SerialUSB.begin(9600);
  Serial1.begin(115200);
  Serial1.setTimeout(1);
}

//Loop
void loop()
{
  delay(10);
  SpektrumRx.getFrame();
  /*
  Serial.print(SpektrumRx.getBindType());
  Serial.print(" ");
  */
  Serial.print(SpektrumRx.getThro());
  Serial.print(" "); 
  Serial.print(SpektrumRx.getAile());
  Serial.print(" ");
  Serial.print(SpektrumRx.getElev());
  Serial.print(" ");
  Serial.print(SpektrumRx.getRudd());
  Serial.print(" ");
  Serial.print(SpektrumRx.getGear());
  Serial.print(" ");
  Serial.print(SpektrumRx.getAux1());
  Serial.print(" ");
  Serial.println();
  
  /*
  for(uint8_t i=0;i<16;i++){
    Serial.print(SpektrumRx.getRaw(i));
    Serial.print("\t");
  }
  */
  
  Serial.println();
  
 
  if(SpektrumRx.getAux1() > RXCENTER) {
  	digitalWrite(ledPin,1);
  }
  else {digitalWrite(ledPin,0);
  }
  
}


/*DOC
0   |   Bind Type (0xa2 or 162 = 22MS 2048 DSMX)
1   |   Aileron High + ID (RATE AFFECTED)
2   |   Aileron Low
3   |   Trainer High
4   |   Trainer Low
5   |   Elevator High + ID (RATE AFFECTED)
6   |   Elevator Low
7   |   Ch 5 High
8   |   Ch 5 Low
9   |   Rudder High + ID (RATE AFFECTED)
10   |   Rudder Low
11   |   Missed frames(capped at 255)
12   |   0 (UNUSED)
13   |   Throttle High + ID
14   |   Throttle Low
15   |   0,1 (UNUSED)
*/