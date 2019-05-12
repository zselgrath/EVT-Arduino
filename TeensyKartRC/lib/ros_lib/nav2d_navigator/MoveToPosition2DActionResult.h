#ifndef _ROS_nav2d_navigator_MoveToPosition2DActionResult_h
#define _ROS_nav2d_navigator_MoveToPosition2DActionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "nav2d_navigator/MoveToPosition2DResult.h"

namespace nav2d_navigator
{

  class MoveToPosition2DActionResult : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef nav2d_navigator::MoveToPosition2DResult _result_type;
      _result_type result;

    MoveToPosition2DActionResult():
      header(),
      status(),
      result()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->result.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->result.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "nav2d_navigator/MoveToPosition2DActionResult"; };
    const char * getMD5(){ return "453591a1a02181261647f316326ee90c"; };

  };

}
#endif