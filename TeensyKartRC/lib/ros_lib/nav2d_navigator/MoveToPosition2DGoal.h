#ifndef _ROS_nav2d_navigator_MoveToPosition2DGoal_h
#define _ROS_nav2d_navigator_MoveToPosition2DGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose2D.h"

namespace nav2d_navigator
{

  class MoveToPosition2DGoal : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Pose2D _target_pose_type;
      _target_pose_type target_pose;
      typedef float _target_distance_type;
      _target_distance_type target_distance;
      typedef float _target_angle_type;
      _target_angle_type target_angle;

    MoveToPosition2DGoal():
      header(),
      target_pose(),
      target_distance(0),
      target_angle(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->target_pose.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_target_distance;
      u_target_distance.real = this->target_distance;
      *(outbuffer + offset + 0) = (u_target_distance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_target_distance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_target_distance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_target_distance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->target_distance);
      union {
        float real;
        uint32_t base;
      } u_target_angle;
      u_target_angle.real = this->target_angle;
      *(outbuffer + offset + 0) = (u_target_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_target_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_target_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_target_angle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->target_angle);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->target_pose.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_target_distance;
      u_target_distance.base = 0;
      u_target_distance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_target_distance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_target_distance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_target_distance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->target_distance = u_target_distance.real;
      offset += sizeof(this->target_distance);
      union {
        float real;
        uint32_t base;
      } u_target_angle;
      u_target_angle.base = 0;
      u_target_angle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_target_angle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_target_angle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_target_angle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->target_angle = u_target_angle.real;
      offset += sizeof(this->target_angle);
     return offset;
    }

    const char * getType(){ return "nav2d_navigator/MoveToPosition2DGoal"; };
    const char * getMD5(){ return "471d15abce131270d05238fa2475ae64"; };

  };

}
#endif