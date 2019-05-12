#ifndef _ROS_fssim_common_State_h
#define _ROS_fssim_common_State_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace fssim_common
{

  class State : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _x_type;
      _x_type x;
      typedef float _y_type;
      _y_type y;
      typedef float _yaw_type;
      _yaw_type yaw;
      typedef float _vx_type;
      _vx_type vx;
      typedef float _vy_type;
      _vy_type vy;
      typedef float _r_type;
      _r_type r;

    State():
      header(),
      x(0),
      y(0),
      yaw(0),
      vx(0),
      vy(0),
      r(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->x);
      offset += serializeAvrFloat64(outbuffer + offset, this->y);
      offset += serializeAvrFloat64(outbuffer + offset, this->yaw);
      offset += serializeAvrFloat64(outbuffer + offset, this->vx);
      offset += serializeAvrFloat64(outbuffer + offset, this->vy);
      offset += serializeAvrFloat64(outbuffer + offset, this->r);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->y));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->yaw));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->vx));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->vy));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->r));
     return offset;
    }

    const char * getType(){ return "fssim_common/State"; };
    const char * getMD5(){ return "ecebba9ceec0b87935afad8421291538"; };

  };

}
#endif