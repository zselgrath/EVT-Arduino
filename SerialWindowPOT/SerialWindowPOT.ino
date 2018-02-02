/*********************************************************************************************************
  TO DO:
  Protect against non integer or out of range values from being entered
    Check for 0<=X<=255
    Zack typed this
    Last line i'll add
*********************************************************************************************************/
#include <mcp_can.h>
#include <SPI.h>

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
  Serial.println("Valid commands: send , edit , address, continuous");
  delay(100);
}

char rx_byte = 0;
String rx_str = "";
String command_str = "";
boolean editFlag = false;         // user will edit the CAN Message
boolean addressFlag = false;      // user will edit the receiving CAN Address 
boolean sendFlag = false;         // user wants the existing CAN message to be sent
boolean continuousFlag = false;   // user wants to continuously send torque values in CAN messages
int editInt = 0;
unsigned int CANaddress = 528;

unsigned char canMsg[3] = {0x90, 0xD0, 0x07}; //This corresponds to decimal values of 144, 208, 7,
                                              //which sends the actual torque value of 2000 (07D0)
                                              // to the motor contoller's 144 (0x90) register


void loop() {
  
  readSerial();
  
  if(editFlag && command_str != ""){
    editMessage();
  }
  else if(addressFlag && command_str != ""){
    editAddress();
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
      Serial.println("Enter the torque value as an integer from 0 to 32767");
      editFlag = true;
      command_str = "";      
    }
    if (command_str == "send") {
      sendMessage();
      command_str = "";
    }
    if (command_str == "continuous") {
      Serial.println("continuous is WIP, entering loop");
      command_str = "";
      continuous();
    }
    if (command_str == "address") {
      Serial.println("Enter an address (integer 0 to 4095) ex. 0x210 is 528");
      addressFlag = true;
      command_str = "";
    }
}

void sendMessage(){
  CAN.sendMsgBuf(CANaddress, 0, 3, canMsg);
  delay(50);
  for(int i = 0; i<3; i++)    // print the data
    {
      Serial.print(canMsg[i]);Serial.print("\t");
    }
  Serial.println("");
  Serial.print("CAN message sent to ID ");
  Serial.println(CANaddress);
}

void editMessage(){ //lol works yay
  
    int sentValue = command_str.toInt();

    canMsg[0] = 0x90;
    canMsg[1] = lowByte(sentValue) & 0xFF;
    canMsg[2] = highByte(sentValue) & 0xFF;
    
    command_str = "";
    Serial.println(sentValue);
    for(int i = 0; i<3; i++)    // print the data
      {
        Serial.print(canMsg[i]);Serial.print("\t");
      }
    Serial.println(" were stored in array for CAN message.");
    editFlag = false;
}

void editAddress(){
  CANaddress = command_str.toInt();
  command_str = "";
  Serial.println("");
  Serial.print("Address set to: ");
  Serial.println(CANaddress);
  addressFlag= false;
}

void continuous(){
    while(command_str != "stop")
    {
      readSerial();
      int sentValue = command_str.toInt();
      canMsg[0] = 0x90;
      canMsg[1] = lowByte(sentValue) & 0xFF;
      canMsg[2] = highByte(sentValue) & 0xFF;
      Serial.println(sentValue);
      CAN.sendMsgBuf(CANaddress, 0, 3, canMsg);
      delay(50);
    }
    command_str = "";
    canMsg[0] = 0x90;
    canMsg[1] = 0;
    canMsg[2] = 0;
    CAN.sendMsgBuf(CANaddress, 0, 3, canMsg);
    Serial.println("STOPPED");
   
}
