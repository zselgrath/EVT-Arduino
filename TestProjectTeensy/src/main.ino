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
//Touch: 188
//Non: 255

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

//Touchscreen defines
#include <stdint.h>
#include "TouchScreen.h"
#define resX 240
#define resy 320
#define YP A14 // must be an analog pin, use "An" notation!
#define XM A15 // must be an analog pin, use "An" notation!
#define YM A16  // can be a digital pin
#define XP A17  // can be a digital pin

//Object instantiation
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 318);
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCLK, TFT_MISO);

void setup()
{
  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setTextSize(2);
}

void loop(void)
{
  //start timer for FPS
  int microsec = micros();
  //Get and show touch point
  TSPoint p = ts.getPoint();
  p.x = (((p.x * resX) / 1023)-29)*1.3; //convert to pixels for ease of use
  p.y = (((p.y * resy) / 1023)-20)*1.25;

  #ifdef DEBUG
  if (p.z > ts.pressureThreshhold)
    {
      drawDebug(p);
    }
  #endif

    microsec = micros() - microsec;
    tft.print("Hz:");
    tft.println(1 / (microsec / 1e6));
}

void drawDebug(TSPoint &_p)
{
  tft.drawRect(220, 300, 20, 20, ILI9341_RED);
  tft.fillRect(0, 0, 210, 30, ILI9341_BLACK);
  tft.setCursor(0, 0);
  tft.print("X:");
  tft.print(_p.x);
  tft.setCursor(75, 0);
  tft.print("Y:");
  tft.println(_p.y);
  tft.fillCircle(_p.x, _p.y, 3, ILI9341_CYAN);
  if (_p.x > 220 && _p.y > 300)
  {
    tft.fillScreen(ILI9341_BLACK);
  }
}