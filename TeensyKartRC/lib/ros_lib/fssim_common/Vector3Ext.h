#ifndef _ROS_fssim_common_Vector3Ext_h
#define _ROS_fssim_common_Vector3Ext_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"

namespace fssim_common
{

  class Vector3Ext : public ros::Msg
  {
    public:
      typedef geometry_msgs::Vector3 _vec_type;
      _vec_type vec;
      typedef float _mag_type;
      _mag_type mag;

    Vector3Ext():
      vec(),
      mag(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->vec.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->mag);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->vec.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->mag));
     return offset;
    }

    const char * getType(){ return "fssim_common/Vector3Ext"; };
    const char * getMD5(){ return "9ce7c2ac065b8145af6ba1b53af0b0bd"; };

  };

}
#endif