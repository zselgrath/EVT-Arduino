#include "VCU.h"

#ifndef CANMESSAGES
#define CANMESSAGES


//class StopMotorControllerMessage {
//  public:
//    unsigned char* getCanPacket()  {
//      unsigned char packet[3] = {0x90, 0, 0};
//      return packet;
//    }
//    unsigned short getCanPacketLength() {
//      return 3;
//    }
//    void send(VCU& vcu) {
//      vcu.sendMotorControllerMessage(this->getCanPacket(), this->getCanPacketLength());
//    }
//    
//    explicit StopMotorControllerMessage() { };
//};
////static StopMotorControllerMessage stopMotorControllerMessage;
//
//class DisableMotorControllerMessage {
//  public:
//    unsigned char* getCanPacket()  {
//      unsigned char packet[3] = {0x51, 0x04 & 0xFF, 0x00 & 0xFF};
//      return packet;
//    }
//    unsigned short getCanPacketLength() {
//      return 3;
//    }
//    void send(VCU& vcu) {
//      vcu.sendMotorControllerMessage(this->getCanPacket(), this->getCanPacketLength());
//    }
////    // Send a message to both disable the motor controller and set the torque to zero
////    void send(VCU& vcu)  {
////      CanMessage::send(vcu);
////      stopMotorControllerMessage.send(vcu);
////    }
//};
//static DisableMotorControllerMessage disableMotorControllerMessage;
//
//class RequestMotorPositionMessage {
//  public:
//    unsigned char* getCanPacket() {
//      unsigned char packet[getCanPacketLength()] = {0x3D, 0x6D, 0x00};
//      return packet;
//    }
//    unsigned short getCanPacketLength() {
//      return 3;
//    }
//    void send(VCU& vcu) {
//      vcu.sendMotorControllerMessage(this->getCanPacket(), this->getCanPacketLength());
//    }
//};
//static RequestMotorPositionMessage requestMotorPositionMessage;

#endif
