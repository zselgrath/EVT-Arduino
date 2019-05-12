/*
 * SatelliteReceiver.cpp
 *
 *  Created on: 9. jan. 2011
 *      Author: develop
 * 			Edit: zselgrath
 */
#include "SatelliteReceiver.h"
#include "HardwareSerial.h"

// The preamble byte vary between Rx
// The first byte is number of errors and maxs out at FF
// The second byte is the bind type
// 01 == DSM2 1024/22ms
// A2 == DSMX 22ms



void SatelliteReceiver::getFrame(void){
	if (Serial1.peek()==162){
		while(Serial1.available() > 16)
		{
			for(index = 0; index <= 15; index++)
			{
				inByte = Serial1.read();
				inData[index] = inByte;
			}
		}
	}
	else
	{
		Serial1.read();
	}
	
}

uint8_t SatelliteReceiver::getRaw(uint8_t pos){
	return inData[pos];
}

uint8_t SatelliteReceiver::getErrors(){
	return inData[0];
}	
uint8_t SatelliteReceiver::getBindType(){
	return inData[0];
}
uint16_t SatelliteReceiver::getAile(){
	uint16_t tempAileron = inData[1]*255 + inData[2];
	tempChannelID = (tempAileron & MASK_2048_CHANID)/2048;
	if(tempChannelID==ID_AILERON){
		tempAileron = (tempAileron & MASK_2048_SXPOS);
		return tempAileron;
	}else{
		return 0;
	}
}
uint16_t SatelliteReceiver::getAux1(){
	uint16_t tempAux1 = inData[3]*255 + inData[4];
	tempChannelID = (tempAux1 & MASK_2048_CHANID)/2048;
	if(tempChannelID==ID_AUX1){
		tempAux1 = (tempAux1 & MASK_2048_SXPOS);
		return tempAux1;
	}else{
		return 0;
	}
}
uint16_t SatelliteReceiver::getGear(){
	uint16_t tempGear = inData[7]*255 + inData[8];
	tempChannelID = (tempGear & MASK_2048_CHANID)/2048;
	if(tempChannelID==ID_GEAR){
		tempGear = (tempGear & MASK_2048_SXPOS);
		return tempGear;
	}else{
		return 0;
	}
}
uint16_t SatelliteReceiver::getElev(){
	uint16_t tempElevator = inData[5]*255 + inData[6];
	tempChannelID = (tempElevator & MASK_2048_CHANID)/2048;
	if(tempChannelID==ID_ELEVATOR){
		tempElevator = (tempElevator & MASK_2048_SXPOS);
		return tempElevator;
	}else{
		return 0;
	}
}
uint16_t SatelliteReceiver::getAux2(){
	return 0;
}
uint16_t SatelliteReceiver::getThro(){
	uint16_t tempThrottle = inData[13]*255 + inData[14];
	tempChannelID = (tempThrottle & MASK_2048_CHANID)/2048;
	if(tempChannelID==ID_THROTTLE){
		tempThrottle = (tempThrottle & MASK_2048_SXPOS);
		return tempThrottle;
	}else{
		return 0;
	}
}
uint16_t SatelliteReceiver::getRudd(){
	uint16_t tempRudder = inData[9]*255 + inData[10];
	tempChannelID = (tempRudder & MASK_2048_CHANID)/2048;
	if(tempChannelID==ID_RUDDER){
		tempRudder = (tempRudder & MASK_2048_SXPOS);
		return tempRudder;
	}else{
		return 0;
	}
}


