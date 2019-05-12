#ifndef _ROS_fssim_common_Cmd_h
#define _ROS_fssim_common_Cmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace fssim_common
{

  class Cmd : public ros::Msg
  {
    public:
      typedef float _dc_type;
      _dc_type dc;
      typedef float _delta_type;
      _delta_type delta;

    Cmd():
      dc(0),
      delta(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->dc);
      offset += serializeAvrFloat64(outbuffer + offset, this->delta);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->dc));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->delta));
     return offset;
    }

    const char * getType(){ return "fssim_common/Cmd"; };
    const char * getMD5(){ return "7c30c8d10cd397c67459b00d587e06f1"; };

  };

}
#endif