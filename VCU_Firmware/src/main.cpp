/*
  VCU Testsketch

  TODO:
    BSE pulldown and capacitor
    Thermistor pullups and capacitors
    Get M3 daughterboard standoffs (may be printed)
    Get 0.1" header jumpers
    Create usability functionality, e.g., calibration of throttle positions
    Create future task functionality in place of delays? (For things like delays in startup sequence... or just use state machine tasks?)
    Implement a watchdog timer which calls an interrupt which writes to EEPROM with the fact it messed up, as well as increments a total number
      of shutdowns, so that if the EEPROM may get worn out the Teensy reports it and won't run

  Bring:
    Banana plugs
    Wire wrap tools

  Note that, when selecting code optimizations, going too high (possibly above -O2?) can be dangerous
 
 */
#include <Arduino.h>
#include "VCU.h"
 
int led = 13;

VCU vcu;

// the setup routine runs once when you press reset:
void setup() {
  vcu.init();
  Serial.begin(115200);

  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  vcu.vcuLoop();
  if(VCU::anySwitchPressed()){
    digitalWrite(led, LOW);
    if(vcu.down()) {
      //vcu.requestStartup();
    }
  }else{
    digitalWrite(led, HIGH);
  }
  if(vcu.left()){
    vcu.requestShutdown();
  }
  if(vcu.start()){
    digitalWrite(led, LOW);
    if(vcu.carCanStart()){
      vcu.requestStartup();
    }else if(vcu.carIsOn()){
      vcu.requestShutdown();
    }
  }

  //delayMicroseconds(50);
}

// Use SPI1 when using the MCP2515 on the VCU:
// MCP2515.cpp from Sandeep Mistry
//  pinMode(_csPin, OUTPUT);
//  // start SPI
//  SPI1.setBitOrder(LSBFIRST); 
//  SPI1.setDataMode(SPI_MODE3); 
//  SPI1.setClockDivider(SPI_CLOCK_DIV16); 
//  SPI1.setMOSI(0);
//  SPI1.setMISO(1);
//  SPI1.setSCK(32);
//  SPI1.begin();
