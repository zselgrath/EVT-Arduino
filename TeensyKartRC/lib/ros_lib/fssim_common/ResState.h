#ifndef _ROS_fssim_common_ResState_h
#define _ROS_fssim_common_ResState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace fssim_common
{

  class ResState : public ros::Msg
  {
    public:
      typedef bool _emergency_type;
      _emergency_type emergency;
      typedef bool _on_off_switch_type;
      _on_off_switch_type on_off_switch;
      typedef bool _push_button_type;
      _push_button_type push_button;
      typedef bool _communication_interrupted_type;
      _communication_interrupted_type communication_interrupted;

    ResState():
      emergency(0),
      on_off_switch(0),
      push_button(0),
      communication_interrupted(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_emergency;
      u_emergency.real = this->emergency;
      *(outbuffer + offset + 0) = (u_emergency.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->emergency);
      union {
        bool real;
        uint8_t base;
      } u_on_off_switch;
      u_on_off_switch.real = this->on_off_switch;
      *(outbuffer + offset + 0) = (u_on_off_switch.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->on_off_switch);
      union {
        bool real;
        uint8_t base;
      } u_push_button;
      u_push_button.real = this->push_button;
      *(outbuffer + offset + 0) = (u_push_button.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->push_button);
      union {
        bool real;
        uint8_t base;
      } u_communication_interrupted;
      u_communication_interrupted.real = this->communication_interrupted;
      *(outbuffer + offset + 0) = (u_communication_interrupted.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->communication_interrupted);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_emergency;
      u_emergency.base = 0;
      u_emergency.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->emergency = u_emergency.real;
      offset += sizeof(this->emergency);
      union {
        bool real;
        uint8_t base;
      } u_on_off_switch;
      u_on_off_switch.base = 0;
      u_on_off_switch.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->on_off_switch = u_on_off_switch.real;
      offset += sizeof(this->on_off_switch);
      union {
        bool real;
        uint8_t base;
      } u_push_button;
      u_push_button.base = 0;
      u_push_button.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->push_button = u_push_button.real;
      offset += sizeof(this->push_button);
      union {
        bool real;
        uint8_t base;
      } u_communication_interrupted;
      u_communication_interrupted.base = 0;
      u_communication_interrupted.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->communication_interrupted = u_communication_interrupted.real;
      offset += sizeof(this->communication_interrupted);
     return offset;
    }

    const char * getType(){ return "fssim_common/ResState"; };
    const char * getMD5(){ return "2c68d942044efe0714c25879acd65327"; };

  };

}
#endif