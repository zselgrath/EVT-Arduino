/***************************************************
  This is our GFX example for the Adafruit ILI9341 Breakout and Shield
  ----> http://www.adafruit.com/products/1651

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

//FPS RECORDS:
//120MHz:
//--Touch: 209
//--Non: 11490
//168MHz:
//--Touch: 212
//--Non: 11111

#define DEBUG

#include "SPI.h"

// For the Adafruit shield, these are the default.
#include "ILI9341_t3.h"
#define TFT_DC 9
#define TFT_CS 10
#define TFT_RST 255 // 255 = unused, connect to 3.3V
#define TFT_MISO 12
#define TFT_MOSI 11
#define TFT_SCLK 13

//--Touchscreen defines--
#include <stdint.h>
#include "TouchScreen.h"
#define resX 240
#define resY 320
#define YP A14 // must be an analog pin, use "An" notation!
#define XM A15 // must be an analog pin, use "An" notation!
#define YM A16 // can be a digital pin
#define XP A17 // can be a digital pin
bool blinkState = false;
bool newBLinkState = false;

//--CAN bus--
#include "FlexCAN.h"
#define canBps 250000
#define canAltTxRx 1

//--Bamocar Regisers
#define regReadBamocarData        0x3D
#define regSysModeBits            0x51
#define regLogicReadBits          0xD8
#define regTorqueCommanded        0x90
#define regSignedRPM              0xA8
#define regSignedPackCurrent      0x20
#define regSignedPhaseCurrent     0x5F
#define regRPMmax                 0xC8

//--Global Vars--

//Keep global uptime in millis
uint32_t upTime = 0;
//Struct to contain all vars that are accessed frequently 
struct bamocarStatus
{
  union logicBitUnion{
    //order 0th bit top 
    struct logicReadBits //0xd8 - these exactly mirror the Bamocar
    {
      bool LMT1;  //UNUSED
      bool LMT2;  //UNUSED
      bool IN2;   //UNUSED
      bool IN1;   //UNUSED
      bool RUN;   //2nd step to enabling output, enabled 200ms after RFE
      bool RFE;   //1st step to enabling output
      bool UNUS1; //UNUSED
      bool UNUS2; //UNUSED
      bool OUT1;  //TEST
      bool OUT2;  //TEST
      bool RDY;   //True when there are no warnings stored
      bool GO;    //True when there is power and control on the output to the motor (NOT DISABLED)
      bool OUT3;  //TEST
      bool OUT4;  //TEST
      bool G_OFF; //TEST
      bool BRK1;  //TEST
    } critBools;
    uint16_t critWord;
  } critStat;
  union statusBitUnion{
    //order 0th bit top
    struct statusBits //0x51 - these exactly mirror the Bamocar
    {
      bool UNUS3;   //UNUSED
      bool SPD0;    //UNUSED
      bool DISABLE; //THE DIGITAL DISABLE BIT
      bool C_OFF;   //UNUSED
      bool TSTA;    //TEST
      bool ILIM;    //TEST
      bool NCLIP;   //TEST
      bool MIX;     //TEST
      bool SYNC;    //TEST
      bool HWHL;    //TEST
      bool UNK1;    //UNUSED
      bool UNK2;    //UNUSED
      bool UNK3;    //UNUSED
      bool UNK4;    //UNUSED
      bool UNK5;    //UNUSED
      bool UNK6;    //UNUSED
    } genBools;
    uint16_t genWord;
  } genStat;
  struct commandedParams
  {
    int16_t signedTorque; //0x90
    char lastMessageStatus;
  } commanded;
  struct updatedParams
  {
    int16_t signedTorque;       //0x90
    int16_t signedRPM;          //0xa8
    int16_t signedPackCurrent;  //0x20
    int16_t signedPhaseCurrent; //0x5f
  } updated;
  struct staticParams
  {
    int16_t Nmax; //0xc8
    uint16_t rxAddress;
    //int16_t PWMfreq;            //0x5a[22-20] of 32bit integer
  } statics;
};

//--UI Defines--
#define uiStartHeight 15
#define buttonWidth 120
#define disableHeight 85
#define enableHeight 85

struct touchPoint{
  uint16_t touchX = 0;
  uint16_t touchY = 0;
  char touchID = 0;
  bool isPressed = false;
} touchPoint;

//--Object instantiation--
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 318); //rx is resistance across x pins of touchscreen
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCLK, TFT_MISO);
CAN_filter_t defaultedMask; //Needed to do full declaration for can bus on secondary pins
FlexCAN canBus(250000);
static CAN_message_t msg, rxmsg;
bamocarStatus bamStat;

//--Debug Only Defines--
#ifdef DEBUG
uint64_t microsec = 0;
char mainLoops = 0;
#endif

//-----------------SETUP-----------------
void setup()
{
  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setTextSize(2);
  drawButtonUI();
  
  defaultedMask.ext = 0;
  defaultedMask.rtr = 0;
  defaultedMask.id = 0;
  bamStat.statics.rxAddress = 0x210;
  canBus.begin(canBps, defaultedMask, canAltTxRx, canAltTxRx);

  tft.setCursor(0, 140);
  tft.println("BEGIN CAN");
  delay(200);

  bamocarDisable();
  tft.print(F("DISABLE: "));
  tft.println(sendOK() ? "OK" : "ERR");
  delay(100);

  bamocarCoast();
  tft.print(F("COAST: "));
  tft.println(sendOK() ? "OK" : "ERR");
  delay(100);

  bamocarQueueData();
  tft.print(F("QUEUE: "));
  tft.println(sendOK() ? "OK" : "ERR");
  delay(100);
}
//-----------------END SETUP-----------------
//-----------------MAIN LOOP-----------------
void loop()
{
  //Keep global uptime in millis
  upTime = millis();
  #ifdef DEBUG
    //start timer for FPS
    microsec = micros();
  #endif

  if (canBus.available())
  {
    canBus.read(rxmsg);
    tft.setTextColor(ILI9341_YELLOW);
    tft.setCursor(130, 100);
    tft.print(rxmsg.len);
    delay(2000);
  }

  //only update the button UI if the screen has been pressed
  inputCheck();
  if (touchPoint.isPressed)
  {
    switch (touchPoint.touchID)
    {
    case 1:
      bamocarDisable();
      break;
    case 2:
      bamocarEnable();
      break;
    case 0:
      bamStat.commanded.signedTorque = pow((((resY - touchPoint.touchY) - 160) / 8), 3);
      bamocarTorque(bamStat.commanded.signedTorque);
      break;
    default:
      break;
    }
  }
  
  drawButtonUI();

  #ifdef DEBUG
    if (touchPoint.isPressed)
    {
      drawFPS();
      drawDebug();
    }
    else if (mainLoops > 250) //dont want to do this too often cause limits performance
    {
      newBLinkState = true;
      drawFPS();
      mainLoops = 0;
    }
    else
    {
      mainLoops++;
    }
  #endif
}
//-----------------END MAIN LOOP-----------------

//DEBUG ONLY METHODS
#ifdef DEBUG
void drawDebug()
{
  tft.drawRect(220, 300, 20, 20, ILI9341_RED);
  tft.fillRect(buttonWidth, 305, (resX-buttonWidth-20), 15, ILI9341_BLACK);
  tft.fillRect(0, 0, 140, uiStartHeight, ILI9341_BLACK);
  tft.setCursor(0, 0);
  tft.print(F("X:"));
  tft.print(touchPoint.touchX);
  tft.setCursor(75, 0);
  tft.print(F("Y:"));
  tft.println(touchPoint.touchY);
  tft.fillCircle(touchPoint.touchX, touchPoint.touchY, 1, ILI9341_CYAN);
  if (touchPoint.touchX > 220 && touchPoint.touchY > 300)
  {
    tft.fillScreen(ILI9341_BLACK);
    drawButtonUI();
  }
}

bool sendOK(){
  byte stat = bamStat.commanded.lastMessageStatus;
  return (stat == 14 | stat == 15 | stat==254);
}

void drawFPS(){
  microsec = micros() - microsec;
  tft.fillRect(140, 0, resX, uiStartHeight, ILI9341_BLACK);
  tft.setCursor(140, 0);
  tft.setTextColor(ILI9341_YELLOW);
  tft.print(F("Hz:"));
  tft.println((int)(1 / (microsec / 1e6)));
}
#endif

void drawButtonUI()
{
  if(newBLinkState){  
    tft.setCursor(0, 0);
     //Draw DISABLE
    if (bamStat.genStat.genBools.DISABLE)
    {
      if (blinkState)
      {
        tft.fillRect(0, uiStartHeight, buttonWidth, disableHeight, ILI9341_RED);
        tft.setCursor(18, 50);
        tft.setTextColor(ILI9341_BLACK);
        tft.print(F("DISABLE"));
        blinkState = !blinkState;
      }
      else
      {
        tft.fillRect(0, uiStartHeight, buttonWidth, disableHeight, ILI9341_BLACK);
        tft.setCursor(18, 50);
        tft.setTextColor(ILI9341_RED);
        tft.print(F("DISABLE"));
        blinkState = !blinkState;
      }
    }
    //Draw Enable
    else
    {
      if (blinkState)
      {
        tft.fillRect(0, (resY - enableHeight), buttonWidth, enableHeight, ILI9341_GREEN);
        tft.setCursor(24, 270);
        tft.setTextColor(ILI9341_BLACK);
        tft.print(F("ENABLE"));
        blinkState = !blinkState;
      }
      else
      {
        tft.fillRect(0, (resY - enableHeight), buttonWidth, enableHeight, ILI9341_BLACK);
        tft.setCursor(24, 270);
        tft.setTextColor(ILI9341_GREEN);
        tft.print(F("ENABLE"));
        blinkState = !blinkState;
      }
    }
  newBLinkState = false;
  }
}

void inputCheck()
{
  TSPoint p = ts.getPoint();
  if (p.z > ts.pressureThreshhold)
  {
    touchPoint.isPressed = true;
    touchPoint.touchX = (((p.x * resX) / 1023) - 29) * 1.3; //convert to pixels for ease of use
    touchPoint.touchY = (((p.y * resY) / 1023) - 20) * 1.25;

    if (touchPoint.touchX > 10 && touchPoint.touchX < buttonWidth) //if the button side is pressed
    {
      if (touchPoint.touchY > uiStartHeight && touchPoint.touchY < disableHeight) //if disable is pressed
      {
        touchPoint.touchID = 1;
      }
      else if (touchPoint.touchY > (resY - enableHeight) && touchPoint.touchY < resY) //if enable is pressed
      {
        touchPoint.touchID = 2;
      }
    }
    else
    {
      touchPoint.touchID = 0; //if no buttons are pressed, assume other UI element
    }
  }
  else
  {
    touchPoint.isPressed = false;
  }
}

//Flips the disable bit to 0
void bamocarEnable(){
  bamocarCoast();
  bamStat.genStat.genBools.DISABLE = false;
  msg.id = bamStat.statics.rxAddress;
  msg.len = 3;
  msg.timeout = 20;
  msg.buf[0] = regSysModeBits;
  msg.buf[1] = 0x00;
  msg.buf[2] = 0x00;
  bamStat.commanded.lastMessageStatus = canBus.write(msg);
  //delay(500);
}

//Flips the disable bit to 1
void bamocarDisable()
{
  bamocarCoast();
  bamStat.genStat.genBools.DISABLE = true;
  msg.id = bamStat.statics.rxAddress;
  msg.len = 3;
  msg.timeout = 20;
  msg.buf[0] = regSysModeBits;
  msg.buf[1] = 0x04;
  msg.buf[2] = 0x00;
  bamStat.commanded.lastMessageStatus = canBus.write(msg);
  //delay(500);
}

//Writes to the Torque Setpoint register in the speed controller
void bamocarTorque(int16_t _torque)
{
  msg.id = bamStat.statics.rxAddress;
  msg.len = 3;
  msg.timeout = 20;
  msg.buf[0] = regTorqueCommanded;
  msg.buf[1] = lowByte(bamStat.commanded.signedTorque);
  msg.buf[2] = highByte(bamStat.commanded.signedTorque);
  bamStat.commanded.lastMessageStatus = canBus.write(msg);
}

//Write a torque value of 0 to disable the position PID loop and coast but
//DOES NOT DISABLE HV OUTPUT
void bamocarCoast(){
  bamStat.commanded.signedTorque = 0;
  bamocarTorque(bamStat.commanded.signedTorque);
  delay(20);
}

void bamocarQueueData()
{
  msg.id = bamStat.statics.rxAddress;
  msg.len = 3;
  msg.timeout = 20;
  msg.buf[0] = regReadBamocarData;
  msg.buf[1] = regSignedRPM;
  msg.buf[2] = 0x64;
  bamStat.commanded.lastMessageStatus = canBus.write(msg);
}