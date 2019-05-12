#ifndef _ROS_fsd_common_msgs_Cone_h
#define _ROS_fsd_common_msgs_Cone_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/String.h"

namespace fsd_common_msgs
{

  class Cone : public ros::Msg
  {
    public:
      typedef geometry_msgs::Point _position_type;
      _position_type position;
      typedef std_msgs::String _color_type;
      _color_type color;

    Cone():
      position(),
      color()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->position.serialize(outbuffer + offset);
      offset += this->color.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->position.deserialize(inbuffer + offset);
      offset += this->color.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "fsd_common_msgs/Cone"; };
    const char * getMD5(){ return "a503fe5c66b468cc3dd8166c52d2bed4"; };

  };

}
#endif