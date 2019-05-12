#ifndef _ROS_fsd_common_msgs_ControlCommand_h
#define _ROS_fsd_common_msgs_ControlCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "std_msgs/Float32.h"

namespace fsd_common_msgs
{

  class ControlCommand : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef std_msgs::Float32 _throttle_type;
      _throttle_type throttle;
      typedef std_msgs::Float32 _steering_angle_type;
      _steering_angle_type steering_angle;

    ControlCommand():
      header(),
      throttle(),
      steering_angle()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->throttle.serialize(outbuffer + offset);
      offset += this->steering_angle.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->throttle.deserialize(inbuffer + offset);
      offset += this->steering_angle.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "fsd_common_msgs/ControlCommand"; };
    const char * getMD5(){ return "fca5f8768fc3871e4f1d86aed4bfd13f"; };

  };

}
#endif