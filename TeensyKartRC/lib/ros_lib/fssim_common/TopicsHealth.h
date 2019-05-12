#ifndef _ROS_fssim_common_TopicsHealth_h
#define _ROS_fssim_common_TopicsHealth_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "fssim_common/TopicState.h"

namespace fssim_common
{

  class TopicsHealth : public ros::Msg
  {
    public:
      typedef bool _topics_check_passed_type;
      _topics_check_passed_type topics_check_passed;
      typedef float _precision_type;
      _precision_type precision;
      uint32_t topics_check_length;
      typedef fssim_common::TopicState _topics_check_type;
      _topics_check_type st_topics_check;
      _topics_check_type * topics_check;

    TopicsHealth():
      topics_check_passed(0),
      precision(0),
      topics_check_length(0), topics_check(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_topics_check_passed;
      u_topics_check_passed.real = this->topics_check_passed;
      *(outbuffer + offset + 0) = (u_topics_check_passed.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->topics_check_passed);
      union {
        float real;
        uint32_t base;
      } u_precision;
      u_precision.real = this->precision;
      *(outbuffer + offset + 0) = (u_precision.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_precision.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_precision.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_precision.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->precision);
      *(outbuffer + offset + 0) = (this->topics_check_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->topics_check_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->topics_check_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->topics_check_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->topics_check_length);
      for( uint32_t i = 0; i < topics_check_length; i++){
      offset += this->topics_check[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_topics_check_passed;
      u_topics_check_passed.base = 0;
      u_topics_check_passed.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->topics_check_passed = u_topics_check_passed.real;
      offset += sizeof(this->topics_check_passed);
      union {
        float real;
        uint32_t base;
      } u_precision;
      u_precision.base = 0;
      u_precision.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_precision.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_precision.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_precision.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->precision = u_precision.real;
      offset += sizeof(this->precision);
      uint32_t topics_check_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      topics_check_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      topics_check_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      topics_check_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->topics_check_length);
      if(topics_check_lengthT > topics_check_length)
        this->topics_check = (fssim_common::TopicState*)realloc(this->topics_check, topics_check_lengthT * sizeof(fssim_common::TopicState));
      topics_check_length = topics_check_lengthT;
      for( uint32_t i = 0; i < topics_check_length; i++){
      offset += this->st_topics_check.deserialize(inbuffer + offset);
        memcpy( &(this->topics_check[i]), &(this->st_topics_check), sizeof(fssim_common::TopicState));
      }
     return offset;
    }

    const char * getType(){ return "fssim_common/TopicsHealth"; };
    const char * getMD5(){ return "2e4a29cd88c13c0624f8c9a144bda96c"; };

  };

}
#endif