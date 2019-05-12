/*
 * SpectrumReceiver.h
 *
 *  Created on: 9. jan. 2011
 *      Author: develop
 * 		Edit: zselgrath
 */

#ifndef SPECTRUMRECEIVER_H_
#define SPECTRUMRECEIVER_H_

#define MASK_2048_CHANID 0x7800
#define MASK_2048_SXPOS 0x07FF 

#define RXCENTER 511
#define RXTRAVEL 358 // (511 center - 63 trim) / 1,25 maxTravel
#define RXMIN 0//153 // 511center - 358travel
#define RXMAX 1022//869 // 511center + 358travel

#define ID_THROTTLE 0
#define ID_AILERON 1
#define ID_ELEVATOR 2
#define ID_RUDDER 3
#define ID_GEAR 4
#define ID_AUX1 5
#define ID_AUX2 6
#define ID_AUX3 7
#define ID_AUX4 8

#include "Arduino.h"

class SatelliteReceiver{
public:
	// void regByte(int byte);
	void getFrame(void);
	uint8_t getRaw(uint8_t pos);
	uint8_t getErrors();
	uint8_t getBindType();
	uint16_t getAile();
	uint16_t getThro();
	uint16_t getElev();
	uint16_t getRudd();
	uint16_t getGear();
	uint16_t getAux1();
	uint16_t getAux2();

private:
	// int channels[8];
	// int prevByte;
	// int channel;
	// bool cByte;
	
	uint8_t inData[16];
	uint8_t index;
	uint8_t inByte;
	uint16_t tempChannelID;
};


#endif /* SPECTRUMRECEIVER_H_ */
