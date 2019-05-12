// demo: CAN-BUS Shield, receive data with check mode
// send data coming to fast, such as less than 10ms, you can use this way
// loovee, 2014-6-13


#include <SPI.h>
#include "mcp_can.h"


// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;
const int LED=8;
boolean ledON=1;
MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

    unsigned char stmp0[8] = {0,0,0,0, 0,0,0,0};    
    unsigned char stmp2[8] = {0,0,0,0, 0,32,0,0};    
    unsigned char stmp3[8] = {0,0,0,0, 102,32,0,0};
    //unsigned char stmp[4] = {0, 0, 0, 127};
void setup()
{
    Serial.begin(115200);
    pinMode(LED,OUTPUT);

START_INIT:

    if(CAN_OK == CAN.begin(CAN_1000KBPS))                   // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Shield init ok!");
    }
    else
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println("Init CAN BUS Shield again");
        delay(100);
        goto START_INIT;
    }
    
    // send data:  id = 0x00, standrad frame, data len = 8, stmp: data buf
    //CAN.sendMsgBuf(0x02040000, 0, 8, stmp);
    //CAN.sendMsgBuf(0x00, 0, 8, stmp);
    CAN.sendMsgBuf( (INT32U)0x2040000, 1, 8, stmp3);
}

void loop()
{
    unsigned char len = 0;
    unsigned char buf[8];
    //delay(3);
    //CAN.sendMsgBuf(0x2040000, 0, 8, stmp);
    //CAN.sendMsgBuf(0x2040000, 0, 8, stmp2);
    //CAN.sendMsgBuf( (INT32U)0x2040000, 1, 8, stmp3);
    //CAN.sendMsgBuf( (INT32U)0x2040000, 1, 8, stmp0);

    if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
    {
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

        INT32U canId = CAN.getCanId();
        if(true) { //for only showing a specific ID
        
        Serial.print("ID: 0x");
        Serial.print(canId,HEX);
        Serial.print("\t");

        for(int i = 0; i<len; i++)    // print the data
        {
            Serial.print(buf[i]);
            Serial.print("\t");
        }
        Serial.println();
        }
    }
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
