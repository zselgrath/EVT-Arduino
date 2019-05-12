#ifndef _ROS_fsd_common_msgs_CarState_h
#define _ROS_fsd_common_msgs_CarState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose2D.h"
#include "fsd_common_msgs/CarStateDt.h"

namespace fsd_common_msgs
{

  class CarState : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Pose2D _car_state_type;
      _car_state_type car_state;
      typedef fsd_common_msgs::CarStateDt _car_state_dt_type;
      _car_state_dt_type car_state_dt;

    CarState():
      header(),
      car_state(),
      car_state_dt()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->car_state.serialize(outbuffer + offset);
      offset += this->car_state_dt.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->car_state.deserialize(inbuffer + offset);
      offset += this->car_state_dt.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "fsd_common_msgs/CarState"; };
    const char * getMD5(){ return "be929b6a6d1d1d2c80065a8e530992fd"; };

  };

}
#endif