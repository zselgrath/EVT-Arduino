//Includes
#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
#include <Arduino.h>
#include <SatelliteReceiver.h>

ros::NodeHandle nh; //takes care of serial port communications
std_msgs::UInt16MultiArray array_msg;
std_msgs::MultiArrayDimension myDim;
std_msgs::MultiArrayLayout myLayout;
ros::Publisher int16Pub("teensyRX", &array_msg);

#define ledPin 13
SatelliteReceiver SpektrumRx;
#define USE_USBCON
uint16_t update = 0;

//Setup
void setup(){
  pinMode(ledPin, OUTPUT);
  
  nh.initNode();

  myDim.label = "channels";
  myDim.size = 7;
  myDim.stride = 1;
  myLayout.dim = (std_msgs::MultiArrayDimension *)malloc(sizeof(std_msgs::MultiArrayDimension) * 1);
  myLayout.dim[0] = myDim;
  myLayout.data_offset = 0;
  array_msg.layout = myLayout;
  array_msg.data = (uint16_t *)malloc(sizeof(uint16_t) * 7);
  array_msg.data_length = 7;

  nh.advertise(int16Pub);
  
  Serial1.begin(115200);
  Serial1.setTimeout(1);
}

//Loop
void loop()
{
  SpektrumRx.getFrame();

  array_msg.data[0] = SpektrumRx.getAile();
  array_msg.data[1] = SpektrumRx.getElev();
  array_msg.data[2] = SpektrumRx.getThro();
  array_msg.data[3] = SpektrumRx.getRudd();
  array_msg.data[4] = SpektrumRx.getGear();
  array_msg.data[5] = SpektrumRx.getAux1();
  array_msg.data[6] = update;

  if (SpektrumRx.getAux1() > 1000)
  {
    digitalWrite(ledPin, 1);
  }
  else
  {
    digitalWrite(ledPin, 0);
  }
  
  int16Pub.publish(&array_msg);
  nh.spinOnce();
  update++;
  delay(20);
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