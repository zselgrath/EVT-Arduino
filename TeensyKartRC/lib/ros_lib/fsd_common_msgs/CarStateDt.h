#ifndef _ROS_fsd_common_msgs_CarStateDt_h
#define _ROS_fsd_common_msgs_CarStateDt_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose2D.h"

namespace fsd_common_msgs
{

  class CarStateDt : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Pose2D _car_state_dt_type;
      _car_state_dt_type car_state_dt;

    CarStateDt():
      header(),
      car_state_dt()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->car_state_dt.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->car_state_dt.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "fsd_common_msgs/CarStateDt"; };
    const char * getMD5(){ return "c42f1095fd820a5883bb3d0c390ca952"; };

  };

}
#endif