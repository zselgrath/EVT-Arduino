#ifndef _ROS_fssim_common_TopicState_h
#define _ROS_fssim_common_TopicState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace fssim_common
{

  class TopicState : public ros::Msg
  {
    public:
      typedef const char* _topic_name_type;
      _topic_name_type topic_name;
      typedef float _expected_frequency_type;
      _expected_frequency_type expected_frequency;
      typedef float _measured_frequency_type;
      _measured_frequency_type measured_frequency;
      typedef bool _passed_type;
      _passed_type passed;

    TopicState():
      topic_name(""),
      expected_frequency(0),
      measured_frequency(0),
      passed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_topic_name = strlen(this->topic_name);
      varToArr(outbuffer + offset, length_topic_name);
      offset += 4;
      memcpy(outbuffer + offset, this->topic_name, length_topic_name);
      offset += length_topic_name;
      union {
        float real;
        uint32_t base;
      } u_expected_frequency;
      u_expected_frequency.real = this->expected_frequency;
      *(outbuffer + offset + 0) = (u_expected_frequency.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_expected_frequency.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_expected_frequency.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_expected_frequency.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->expected_frequency);
      union {
        float real;
        uint32_t base;
      } u_measured_frequency;
      u_measured_frequency.real = this->measured_frequency;
      *(outbuffer + offset + 0) = (u_measured_frequency.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_measured_frequency.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_measured_frequency.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_measured_frequency.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->measured_frequency);
      union {
        bool real;
        uint8_t base;
      } u_passed;
      u_passed.real = this->passed;
      *(outbuffer + offset + 0) = (u_passed.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->passed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_topic_name;
      arrToVar(length_topic_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_topic_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_topic_name-1]=0;
      this->topic_name = (char *)(inbuffer + offset-1);
      offset += length_topic_name;
      union {
        float real;
        uint32_t base;
      } u_expected_frequency;
      u_expected_frequency.base = 0;
      u_expected_frequency.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_expected_frequency.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_expected_frequency.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_expected_frequency.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->expected_frequency = u_expected_frequency.real;
      offset += sizeof(this->expected_frequency);
      union {
        float real;
        uint32_t base;
      } u_measured_frequency;
      u_measured_frequency.base = 0;
      u_measured_frequency.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_measured_frequency.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_measured_frequency.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_measured_frequency.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->measured_frequency = u_measured_frequency.real;
      offset += sizeof(this->measured_frequency);
      union {
        bool real;
        uint8_t base;
      } u_passed;
      u_passed.base = 0;
      u_passed.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->passed = u_passed.real;
      offset += sizeof(this->passed);
     return offset;
    }

    const char * getType(){ return "fssim_common/TopicState"; };
    const char * getMD5(){ return "5557167df4d3920fba79516729b9f245"; };

  };

}
#endif