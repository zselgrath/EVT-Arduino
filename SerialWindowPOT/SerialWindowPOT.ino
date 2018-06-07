/*********************************************************************************************************
  TO DO:
  Protect against non integer or out of range values from being entered
    Check for 0<=X<=255
    Chase edited this
    Whomsdtve is thou
*********************************************************************************************************/
#include <mcp_can.h>
#include <SPI.h>

const int SPI_CS_PIN = 9;
MCP_CAN CAN(SPI_CS_PIN);
char rx_byte = 0;
String rx_str = "";
String command_str = "";
unsigned char len = 0; //length of the data received
unsigned char buf[8];  //array that stores the data in the message
boolean editFlag = false;         // user will edit the CAN Message
boolean addressFlag = false;      // user will edit the receiving CAN Address 
boolean readFlag = false;         // user wants to continuously print can messages over serial
boolean unusedFlag = false;      // user wants to ...
int editInt = 0;
unsigned int canAddress = 528;
unsigned char canMsg[3] = {0x90, 0xD0, 0x07}; //This corresponds to decimal values of 144, 208, 7,
                                              //which sends the actual torque value of 2000 (07D0)
                                              // to the motor contoller's 144 (0x90) register
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
  Serial.println("enable , disable , read, mute");
  delay(100);
}

void loop() {
  
  readSerial();
  
  if (command_str != ""){
    if(editFlag){
      editMessage();
    }
    else if(addressFlag){
      editAddress();
    }
    else{
      setFlag();
    }
  }

  if (readFlag){
    readMessage();
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
    if (command_str == "read")
    {
      readFlag = true;
      command_str = "";
    }
    if (command_str == "mute")
    {
      readFlag = false;
      command_str = "";
    }
    if (command_str == "disable") {
      disable();
      command_str = "";
    }
    if (command_str == "enable") {
      enable();
      command_str = "";
    }
    if (command_str == "voltage") {
      bamocarRequest(0x66,0x00);
      command_str = "";
    }
    if (command_str == "RPM") {
      bamocarRequest(0x30,0x00);
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

void readMessage(){
  if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
    {
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

        unsigned int canId = CAN.getCanId();
        
        Serial.print("New Message: ");
        Serial.println(canId, HEX);

        for(int i = 0; i<len; i++)    // print the data
        {
            Serial.print(buf[i], HEX);
            Serial.print("\t");
        }
        Serial.println();
    }
}

void sendMessage(){
  CAN.sendMsgBuf(canAddress, 0, 3, canMsg);
  delay(50);
  for(int i = 0; i<3; i++)    // print the data
    {
      Serial.print(canMsg[i]);Serial.print("\t");
    }
  Serial.println("");
  Serial.print("CAN message sent to ID ");
  Serial.println(canAddress);
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
  canAddress = command_str.toInt();
  command_str = "";
  Serial.println("");
  Serial.print("Address set to: ");
  Serial.println(canAddress);
  addressFlag= false;
}

void continuous(){ //enter a loop where the arduino constantly reads in torque values over serial and sends them if they change
    int sentValue = 0;
    int lastValue = 0;
    enable();
    while(command_str != "stop") //while the command is an integer or word other than stop
    {
      readMessage();
      readSerial();
      sentValue = command_str.toInt();
      if (sentValue != lastValue){ //if the data changed from the last value that was received
        canMsg[0] = 0x90;
        canMsg[1] = lowByte(sentValue) & 0xFF;
        canMsg[2] = highByte(sentValue) & 0xFF;
        Serial.print("changed to ");
        Serial.println(sentValue);
        CAN.sendMsgBuf(canAddress, 0, 3, canMsg);
        lastValue = sentValue;
        delay(20);
      }
    }
    command_str = ""; //"stop" was sent, cancelling torque values and disabling motor controller
    canMsg[0] = 0x90;
    canMsg[1] = 0;
    canMsg[2] = 0;
    CAN.sendMsgBuf(canAddress, 0, 3, canMsg);
    //TODO: add enable/disable methods
    Serial.println("***STOPPED***");
   
}

void disable(){
  canMsg[0] = 0x51;
  canMsg[1] = 04 & 0xFF;
  canMsg[2] = 00 & 0xFF;
  CAN.sendMsgBuf(canAddress, 0, 3, canMsg);
  Serial.println("***DISABLED***");
}

void enable(){
  canMsg[0] = 0x51;
  canMsg[1] = 00 & 0xFF;
  canMsg[2] = 00 & 0xFF;
  CAN.sendMsgBuf(canAddress, 0, 3, canMsg);
  Serial.println("***ENABLED***");
}

void bamocarRequest(int internalReg, int cycle){
  canMsg[0] = 0x3D;
  canMsg[1] = cycle & 0xFF;
  canMsg[2] = internalReg & 0xFF;
  CAN.sendMsgBuf(canAddress, 0, 3, canMsg);
  Serial.println("***REQUESTED***");
}
