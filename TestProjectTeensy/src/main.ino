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
#define resy 320
#define YP A14 // must be an analog pin, use "An" notation!
#define XM A15 // must be an analog pin, use "An" notation!
#define YM A16 // can be a digital pin
#define XP A17 // can be a digital pin

//--CAN bus--
#include "FlexCAN.h"
#define canBps 500000
#define canAltTxRx 1

//--Global Vars--

//Keep global uptime in millis
uint32_t upTime = 0;
struct bamocarStatus
{
  union logicBitUnion{
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
    struct statusBits //0x51 - these exactly mirror the Bamocar
    {
      bool UNUS3;   //UNUSED
      bool SPD0;    //UNUSED
      bool DISABLE; //THE DIGITAL DISABLE BIT
      bool C_OFF;   //UNUSED
      bool TSTA;    //2nd step to enabling output, enabled 200ms after RFE
      bool ILIM;    //1st step to enabling output
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
    uint8_t rxAddress;
    //int16_t PWMfreq;            //0x5a[22-20] of 32bit integer
  } statics;
};

#ifdef DEBUG
  uint64_t microsec = 0;
  char mainLoops = 0;
#endif

//--UI Defines--
#define uiStartHeight 15
#define buttonWidth 120
#define disableHeight 85
#define enableHeight 30

//--Object instantiation--
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 318);
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCLK, TFT_MISO);
CAN_filter_t defaultedMask; //Needed to do full declaration for can bus on secondary pins
FlexCAN canBus;
static CAN_message_t msg, rxmsg;
bamocarStatus bamStat;

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
  initContinuousData(canBus);
}

void loop(void)
{
  //Keep global uptime in millis
  upTime = millis();
  #ifdef DEBUG
    //start timer for FPS
    microsec = micros();
  #endif

  if(canBus.available()){
    canBus.read(rxmsg);
    tft.print(rxmsg.id);
    delay(20);
  }

  TSPoint p = ts.getPoint();
  p.x = (((p.x * resX) / 1023) - 29) * 1.3; //convert to pixels for ease of use
  p.y = (((p.y * resy) / 1023) - 20) * 1.25;

  #ifdef DEBUG
    if (p.z > ts.pressureThreshhold)
    {
      drawDebug(p);
    }
  #endif
  
  //only update the button UI if the screen has been pressed
  if (p.z > ts.pressureThreshhold)
  {
    drawButtonUI();
  }
  

  #ifdef DEBUG
  if (p.z > ts.pressureThreshhold)
  {
    drawFPS();
  }
  else if (mainLoops > 250){ //dont want to do this too often cause limits performance
    drawFPS();
    mainLoops = 0;
  }
  else{
    mainLoops++;
  }
  #endif
}

void drawDebug(TSPoint &_p)
{
  tft.drawRect(220, 300, 20, 20, ILI9341_RED);
  tft.fillRect(0, 0, 140, uiStartHeight, ILI9341_BLACK);
  tft.setCursor(0, 0);
  tft.print(F("X:"));
  tft.print(_p.x);
  tft.setCursor(75, 0);
  tft.print(F("Y:"));
  tft.println(_p.y);
  tft.fillCircle(_p.x, _p.y, 1, ILI9341_CYAN);
  if (_p.x > 220 && _p.y > 300)
  {
    tft.fillScreen(ILI9341_BLACK);
  }
}

#ifdef DEBUG
void drawFPS(){
  microsec = micros() - microsec;
  tft.fillRect(140, 0, 240, uiStartHeight, ILI9341_BLACK);
  tft.setCursor(140, 0);
  tft.setTextColor(ILI9341_YELLOW);
  tft.print(F("Hz:"));
  tft.println((int)(1 / (microsec / 1e6)));
  //delay(20);
}
#endif

void drawButtonUI()
{
  tft.setTextColor(ILI9341_BLACK);
  tft.setCursor(0,0);
  //Draw DISABLE
  tft.fillRect(0, uiStartHeight, buttonWidth, disableHeight, ILI9341_RED);
  tft.setCursor(18,50);
  tft.print(F("DISABLE"));
  //Draw Enable
  tft.fillRect(0, (320-enableHeight), buttonWidth, enableHeight, ILI9341_GREEN);
  tft.setCursor(24, 300);
  tft.print(F("ENABLE"));
}

void initContinuousData(FlexCAN &_canBus){
  CAN_message_t msgContinuousData;
  msgContinuousData.id = bamStat.statics.rxAddress;
  msgContinuousData.len = 3;
  msgContinuousData.buf[0] = 0x3D;
  msgContinuousData.buf[1] = 0xA8;
  msgContinuousData.buf[2] = 0x64;
  _canBus.write(msgContinuousData);
}