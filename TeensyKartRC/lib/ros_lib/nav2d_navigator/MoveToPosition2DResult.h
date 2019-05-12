#ifndef _ROS_nav2d_navigator_MoveToPosition2DResult_h
#define _ROS_nav2d_navigator_MoveToPosition2DResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose2D.h"

namespace nav2d_navigator
{

  class MoveToPosition2DResult : public ros::Msg
  {
    public:
      typedef geometry_msgs::Pose2D _final_pose_type;
      _final_pose_type final_pose;
      typedef float _final_distance_type;
      _final_distance_type final_distance;

    MoveToPosition2DResult():
      final_pose(),
      final_distance(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->final_pose.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_final_distance;
      u_final_distance.real = this->final_distance;
      *(outbuffer + offset + 0) = (u_final_distance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_final_distance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_final_distance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_final_distance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->final_distance);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->final_pose.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_final_distance;
      u_final_distance.base = 0;
      u_final_distance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_final_distance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_final_distance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_final_distance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->final_distance = u_final_distance.real;
      offset += sizeof(this->final_distance);
     return offset;
    }

    const char * getType(){ return "nav2d_navigator/MoveToPosition2DResult"; };
    const char * getMD5(){ return "1494b1c9041b641e97cee161a63a1b7b"; };

  };

}
#endif