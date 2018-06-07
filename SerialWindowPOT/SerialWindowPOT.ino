/*********************************************************************************************************
  TO DO:
  Protect against non integer or out of range values from being entered
    Check for 0<=X<=255
    
  CHASE WUZ HERE
*********************************************************************************************************/
#include <mcp_can.h>
#include <SPI.h>

int sensorPin = A0;    // select the input pin for the potentiometer
int sensorValue = 0;
const int SPI_CS_PIN = 9;
MCP_CAN CAN(SPI_CS_PIN);

void setup() {
  Serial.begin(115200);
  while (CAN_OK != CAN.begin(CAN_250KBPS))              // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(100);
    }
  Serial.println("CAN BUS Shield init ok!");
  delay(100);
  Serial.println("Valid commands: send , edit , address, pot");
  delay(100);
}

char rx_byte = 0;
String rx_str = "";
String command_str = "";
boolean editFlag = false;         // user will edit the CAN Message
boolean addressFlag = false;      // user will edit the receiving CAN Address 
boolean sendFlag = false;         // user wants the existing CAN message to be sent
boolean potFlag = false;          // user wants to start pot values in CAN messages to be sent
int editInt = 0;
unsigned int CANaddress = 528;

unsigned char canMsg[3] = {0x90, 0xD0, 0x07}; //This corresponds to decimal values of 144, 208, 7,
                                              //which sends the actual torque value of 2000 (07D0)
                                              // to the motor contoller's 144 (0x90) register


void loop() {
  
  readSerial();
  
  if(sendFlag){
    send();
  }
  else if(editFlag && command_str != ""){
    edit();
  }
  else if(addressFlag && command_str != ""){
    address();
  }
  else if (potFlag && command_str != "")
  {
    address();
  }
  else{
    setFlag();
  }

}

void readSerial(){
  while (Serial.available() > 0) {    // is a character available?
      
      rx_byte = Serial.read();       // get the character
      
      if (rx_byte == '\n') {         // check if the line is over
        Serial.println(">>>" + rx_str);
        command_str = rx_str;
        rx_str = "";                 // clear the string for reuse
      }
      else{
        rx_str += rx_byte;           // if it is not, add the character to a string 
      }
  }
}

void setFlag(){
  if (command_str == "edit") {
      //MAKE THIS
      Serial.println("Enter your message in pairs of bytes (integer values 0-255!!!)");
      Serial.println("Enter the X value: [X,0,0]");
      editFlag = true;
      command_str = "";
  }
  if (command_str == "send") {
      sendFlag = true;
      command_str = "";
  }
  if (command_str == "pot") {
      //TODO: build a way to pot stored CAN messages by specifying an order and a delay
      potFlag = true;
      Serial.println("pot is WIP, reading potentiometer values");
      command_str = "";
  }
  if (command_str == "address") {
      Serial.println("Enter an address (integer 0 to 4095) ex. 0x210 is 528");
      addressFlag = true;
      command_str = "";
  }
}

void send(){
  CAN.sendMsgBuf(CANaddress, 0, 3, canMsg);
  delay(50);
  for(int i = 0; i<3; i++)    // print the data
    {
      Serial.print(canMsg[i]);Serial.print("\t");
    }
  Serial.println("");
  Serial.print("CAN message sent to ID ");
  Serial.println(CANaddress);
  sendFlag = false;
}

void edit(){
  switch (editInt) {
  case 0:    // your hand is on the sensor
    canMsg[0] = command_str.toInt() & 0xFF;
    //c[0] = i & 0xFF;
    editInt++;
    command_str = "";
    Serial.println("Enter the X value: [0,X,0]");
    break;
  case 1:    // your hand is close to the sensor
    canMsg[1] = command_str.toInt() & 0xFF;
    editInt++;
    command_str = "";
    Serial.println("Enter the X value: [0,0,X]");
    break;
  case 2:    // your hand is close to the sensor
    canMsg[2] = command_str.toInt() & 0xFF;
    editInt=0;
    command_str = "";
    Serial.println("Values stored in array for CAN message!");
    for(int i = 0; i<3; i++)    // print the data
      {
        Serial.print(canMsg[i]);Serial.print("\t");
      }
    
    editFlag = false;
    
    break;
  }
}

void address(){
  CANaddress = command_str.toInt();
  command_str = "";
  Serial.println("");
  Serial.print("Address set to: ");
  Serial.println(CANaddress);
  addressFlag= false;
}
/*
void pot()
{
   // read the value from the sensor:
   // potVal = analogRead(sensorPin);
    
    //byte high = highByte(potVal);
    //byte low = lowByte(potVal);

    canMsg[0] = 0x90;
    canMsg[1] = low & 0xFF;
    canMsg[2] = high & 0xFF;
    CAN.sendMsgBuf(CANaddress, 0, 3, canMsg);
    for (int i = 0; i < 3; i++) // print the data
    {
      Serial.print(canMsg[i]);
      Serial.print("\t");
    }
    delay(50);
}
*/
