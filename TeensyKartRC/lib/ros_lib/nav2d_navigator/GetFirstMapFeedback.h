#ifndef _ROS_nav2d_navigator_GetFirstMapFeedback_h
#define _ROS_nav2d_navigator_GetFirstMapFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace nav2d_navigator
{

  class GetFirstMapFeedback : public ros::Msg
  {
    public:

    GetFirstMapFeedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return "nav2d_navigator/GetFirstMapFeedback"; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

}
#endif