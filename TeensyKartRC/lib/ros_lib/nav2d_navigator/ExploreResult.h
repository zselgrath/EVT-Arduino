#ifndef _ROS_nav2d_navigator_ExploreResult_h
#define _ROS_nav2d_navigator_ExploreResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose2D.h"

namespace nav2d_navigator
{

  class ExploreResult : public ros::Msg
  {
    public:
      typedef geometry_msgs::Pose2D _final_pose_type;
      _final_pose_type final_pose;

    ExploreResult():
      final_pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->final_pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->final_pose.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "nav2d_navigator/ExploreResult"; };
    const char * getMD5(){ return "9b03b05e2f5c62e96e4cec4715bf432f"; };

  };

}
#endif