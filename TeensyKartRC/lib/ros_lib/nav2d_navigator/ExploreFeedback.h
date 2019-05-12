#ifndef _ROS_nav2d_navigator_ExploreFeedback_h
#define _ROS_nav2d_navigator_ExploreFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose2D.h"

namespace nav2d_navigator
{

  class ExploreFeedback : public ros::Msg
  {
    public:
      typedef geometry_msgs::Pose2D _robot_pose_type;
      _robot_pose_type robot_pose;
      typedef geometry_msgs::Pose2D _target_pose_type;
      _target_pose_type target_pose;
      typedef float _distance_type;
      _distance_type distance;

    ExploreFeedback():
      robot_pose(),
      target_pose(),
      distance(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->robot_pose.serialize(outbuffer + offset);
      offset += this->target_pose.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_distance;
      u_distance.real = this->distance;
      *(outbuffer + offset + 0) = (u_distance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_distance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_distance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_distance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->distance);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->robot_pose.deserialize(inbuffer + offset);
      offset += this->target_pose.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_distance;
      u_distance.base = 0;
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->distance = u_distance.real;
      offset += sizeof(this->distance);
     return offset;
    }

    const char * getType(){ return "nav2d_navigator/ExploreFeedback"; };
    const char * getMD5(){ return "e64a606b3357bbb098996ab9c2799a9f"; };

  };

}
#endif