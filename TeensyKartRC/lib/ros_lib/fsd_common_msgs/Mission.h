#ifndef _ROS_fsd_common_msgs_Mission_h
#define _ROS_fsd_common_msgs_Mission_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace fsd_common_msgs
{

  class Mission : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _mission_type;
      _mission_type mission;
      typedef bool _finished_type;
      _finished_type finished;

    Mission():
      header(),
      mission(""),
      finished(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_mission = strlen(this->mission);
      varToArr(outbuffer + offset, length_mission);
      offset += 4;
      memcpy(outbuffer + offset, this->mission, length_mission);
      offset += length_mission;
      union {
        bool real;
        uint8_t base;
      } u_finished;
      u_finished.real = this->finished;
      *(outbuffer + offset + 0) = (u_finished.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->finished);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_mission;
      arrToVar(length_mission, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_mission; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_mission-1]=0;
      this->mission = (char *)(inbuffer + offset-1);
      offset += length_mission;
      union {
        bool real;
        uint8_t base;
      } u_finished;
      u_finished.base = 0;
      u_finished.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->finished = u_finished.real;
      offset += sizeof(this->finished);
     return offset;
    }

    const char * getType(){ return "fsd_common_msgs/Mission"; };
    const char * getMD5(){ return "aa152a842fbf6b840a20d415a574e050"; };

  };

}
#endif