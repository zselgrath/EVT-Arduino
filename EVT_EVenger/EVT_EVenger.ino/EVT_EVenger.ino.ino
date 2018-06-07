// demo: CAN-BUS Shield, receive data with check mode
// send data coming to fast, such as less than 10ms, you can use this way
// loovee, 2014-6-13


#include <SPI.h>
#include "mcp_can.h"
#include <LiquidCrystal.h> // includes the LiquidCrystal Library 
LiquidCrystal lcd(18, 24, 34, 36, 38, 40); // Creates an LC object. Parameters: (rs, enable, d4, d5, d6, d7)


unsigned char len = 0; //length of the data received
unsigned char buf[4];  //array that stores the data in the message

String currentRaw; 
float iFloatRaw;
float iFloatFinal;

String voltageRaw; 
float vFloatRaw;
float vFloatFinal;

float wattFinal;

// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

void setup()
{
    //LCD Stuff
    // set up the LCD  's number of columns and rows:
    lcd.begin(16, 2);

    //CAN Shield Stuff    
    Serial.begin(115200);

    Serial.println("Starting in");
    delay(100);
    Serial.print("5--");
    delay(100);
    Serial.print("4--");
    delay(100);
    Serial.print("3--");
    delay(100);
    Serial.print("2--");
    delay(100);
    Serial.println("1--");
    delay(100);

    while (CAN_OK != CAN.begin(CAN_250KBPS))              // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");
}


void loop()
{
    readMessage();
    //fakeMessage();
    updateLCD();
}

void readMessage(){
  if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
    {
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
        
        for(int i = 0; i<len; i++)    // print the data
        {
            Serial.print(buf[i], HEX);
            Serial.print("\t");
        }
        Serial.println();

       currentRaw = String(buf[0]*16*16+buf[1]);
       iFloatRaw = currentRaw.toInt();
       iFloatFinal = iFloatRaw/10;
       Serial.println(vFloatRaw);
       Serial.println(vFloatFinal);
       
       voltageRaw = String(buf[2]*16*16+buf[3]);
       vFloatRaw = voltageRaw.toInt();
       vFloatFinal = vFloatRaw/10;
       Serial.println(vFloatRaw);
       Serial.println(vFloatFinal);

       wattFinal = vFloatFinal*iFloatFinal;
       Serial.println(wattFinal);
       
        
    }
}

/*
void fakeMessage(){
        buf[0] = 1;
        buf[1] = 180;
        buf[2] = 2;
        buf[3] = 241;
        
        for(int i = 0; i<len; i++)    // print the data
        {
            Serial.print(buf[i], HEX);
            Serial.print("\t");
        }
        Serial.println();

       currentRaw = String(buf[0]*16*16+buf[1]);
       iFloatRaw = currentRaw.toInt();
       iFloatFinal = iFloatRaw/10;
       Serial.println(vFloatRaw);
       Serial.println(vFloatFinal);
       
       voltageRaw = String(buf[2]*16*16+buf[3]);
       vFloatRaw = voltageRaw.toInt();
       vFloatFinal = vFloatRaw/10;
       Serial.println(vFloatRaw);
       Serial.println(vFloatFinal);      
}
*/


void updateLCD(){
    lcd.print("V:");
    lcd.print(vFloatFinal);            
    lcd.setCursor(0,1);
    lcd.print("I:");
    lcd.print(iFloatFinal);
    lcd.setCursor(9,0);
    lcd.print("W:");
    lcd.print(wattFinal);
    
    delay(40);
    lcd.clear();
    lcd.home();
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
