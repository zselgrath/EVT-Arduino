/*
 * The goal is: combine Zack's SerialWindowPOT.ino with receive_check.ino from 
 * Github and make sure everything that is received prints to the serial window.
 */

/*Serial Peripheral Interface is used by microcontrollers for communicating
with one or more peripheral devices quickly. Can set up a master-slave condition.*/
#include <SPI.h>

/*CAN BUS library*/
#include "mcp_can.h"

/*Will add when potentiometer method is added
int sensorPin = A0;
int sensorValue = 0;
*/
const int SPI_CS_PIN = 9; //The Seeed studio can bus shields use pin 9 for SPI communication to the board
MCP_CAN CAN(SPI_CS_PIN);  //Declaring a new instance of MCP can with the default pin
char rx_byte = 0;
String command_str = "";                      //setting the command string null until a command is received
unsigned int CANaddress = 528;                //default CAN address of Bamocar D3 Motor controller (can change later)
unsigned char canMsg[3] = {0x90, 0xD0, 0x07}; //This corresponds to decimal values of 144, 208, 7,
                                              //which sends the actual torque value of 2000 (07D0)
                                              // to the motor contoller's 144 (0x90) register

void setup()
{
  Serial.begin(115200);
  // init can bus: baudrate 250k
  while (CAN_OK != CAN.begin(CAN_250KBPS))
  {
    Serial.println("CAN BUS Shield init fail");
    Serial.println("Init CAN BUS Shield again");
    delay(100);
  }
  /*Zack's stuff*/
  Serial.println("CAN BUS Shield init ok!");
  delay(100);
  Serial.println("Valid commands: send, edit, address, pot");
  delay(100);
}

void loop()
{
  readSerial();

  setFlag();
}

/*
 * Zack's original code, sent 01/25/18. To be modified.
 */

void readSerial()
{
  while (Serial.available() > 0) //is a character still available in the terminal?
  {
    //get the character
    rx_byte = Serial.read();
    //check if the line is over
    if (rx_byte == '\n')
    {
      Serial.println(">>>" + command_str);
    }
    else
    {
      //if not, add the character to a string
      command_str.concat(rx_byte);
    }
  }
}

void setFlag()
{
  switch (command_str)
  {
  case "edit":
    edit();
    break;
  case "send":
    send();
    break;
  case "pot":
    //TODO: build a way to pot stored CAN emssages by specifying an order and a delay
    pot();
    Serial.println("pot is WIP, reading potentiometer values");
    break;
  case "address":
    Serial.println("Enter an address (integer 0 to 4095) ex. 0x210 is 528");
    address();
    break;
  default:
    break;
  }
  command_str = "";
}

void send()
{
  CAN.sendMsgBuf(CANaddress, 0, 3, canMsg); //Sending the actual CAN message using the mcp_can library
  printCanMsg(); //For debugging only, remove once deployed
  delay(50); //adjust according to maximum communication rate
}

void edit()
{
  Serial.println("Enter your message in pairs of bytes (integer values 0-255!!!)");
  for (int i = 0; i < 3; i++)
  {
    Serial.print("Enter the X value: [ ");
    for (int j = 0; j < 3; j++)
    {
      if (i == j)
        Serial.print("X ");
      else
        Serial.print("0 ");
    }
    Serial.println("]");
    canMsg[i] = command_str.toInt() & 0xFF;
  }
  Serial.println("Values stored in array for CAN message!");
  printCanMsg();
}

void address()
{
  CANaddress = command_str.toInt();
  command_str = "";
  Serial.println('\n' + "Address set to: " + CANaddress);
  addressFlag = false;
}

/*
void pot()
{
  //read potentiometer data from pin
  potVal = analogRead(sensorPin);

  canMsg[0] = 0x90;
  canMsg[1] = lowByte(potVal) & 0xFF;
  canMsg[2] = highByte(potVal) & 0xFF;

  printCanMsg();
  delay(50);
}
*/

void printCanMsg()
{
  // print the data
  for (int i = 0; i < 3; i++)
    Serial.print(canMsg[i] + '\t');
  Serial.println("");
  Serial.print("CAN message sent to ID " + CANaddress);
}
