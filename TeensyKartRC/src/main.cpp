//Includes
#include <ros.h>
#include <mavros_msgs/RCIn.h>
#include <Arduino.h>
#include <SatelliteReceiver.h>

ros::NodeHandle nh; //takes care of serial port communications
mavros_msgs::RCIn rcin;
ros::Publisher rcPublisher("/mavros/rc/in",&rcin);

//Defines
#define ledPin 13
SatelliteReceiver SpektrumRx;
#define USE_USBCON

//Setup
void setup(){
  pinMode(ledPin, OUTPUT);
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(rcPublisher);
  
  Serial1.begin(115200);
  Serial1.setTimeout(1);
}

//Loop
void loop()
{
  delay(20);
  SpektrumRx.getFrame();
  /*
  Serial.print(SpektrumRx.getBindType());
  Serial.print(" ");
  
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
  
  for(uint8_t i=0;i<16;i++){
    Serial.print(SpektrumRx.getRaw(i));
    Serial.print("\t");
  }
  
  Serial.println();
  */

  rcin.rssi = 0;
  rcin.channels[0] = SpektrumRx.getAile();
  rcin.channels[1] = SpektrumRx.getElev();
  rcin.channels[2] = SpektrumRx.getThro();
  rcin.channels[3] = SpektrumRx.getRudd();
  rcin.channels[4] = SpektrumRx.getGear();
  rcin.channels[5] = SpektrumRx.getAux1();
  rcin.channels[6] = SpektrumRx.getAux1();

  if (SpektrumRx.getAux1() > RXCENTER)
  {
    digitalWrite(ledPin, 1);
  }
  else
  {
    digitalWrite(ledPin, 0);
  }

  rcPublisher.publish(&rcin);
  nh.spinOnce();
}

/*DOC
SPEKTRUM RAW FROM RECEIVER:
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

PIXHAWK MAVROS RCIN TOPIC:
0   |   Aileron
1   |   Elevator
2   |   Throttle
3   |   Rudder
4   |   Gear (Ch 5)
5   |   Aux1
6   |   UNUSED
*/
