#ifndef _ROS_fssim_common_WheelSpeeds_h
#define _ROS_fssim_common_WheelSpeeds_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace fssim_common
{

  class WheelSpeeds : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint8_t _version_type;
      _version_type version;
      typedef int16_t _rpm_front_left_type;
      _rpm_front_left_type rpm_front_left;
      typedef int16_t _rpm_front_right_type;
      _rpm_front_right_type rpm_front_right;
      typedef int16_t _rpm_rear_left_type;
      _rpm_rear_left_type rpm_rear_left;
      typedef int16_t _rpm_rear_right_type;
      _rpm_rear_right_type rpm_rear_right;

    WheelSpeeds():
      header(),
      version(0),
      rpm_front_left(0),
      rpm_front_right(0),
      rpm_rear_left(0),
      rpm_rear_right(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->version >> (8 * 0)) & 0xFF;
      offset += sizeof(this->version);
      union {
        int16_t real;
        uint16_t base;
      } u_rpm_front_left;
      u_rpm_front_left.real = this->rpm_front_left;
      *(outbuffer + offset + 0) = (u_rpm_front_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rpm_front_left.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rpm_front_left);
      union {
        int16_t real;
        uint16_t base;
      } u_rpm_front_right;
      u_rpm_front_right.real = this->rpm_front_right;
      *(outbuffer + offset + 0) = (u_rpm_front_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rpm_front_right.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rpm_front_right);
      union {
        int16_t real;
        uint16_t base;
      } u_rpm_rear_left;
      u_rpm_rear_left.real = this->rpm_rear_left;
      *(outbuffer + offset + 0) = (u_rpm_rear_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rpm_rear_left.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rpm_rear_left);
      union {
        int16_t real;
        uint16_t base;
      } u_rpm_rear_right;
      u_rpm_rear_right.real = this->rpm_rear_right;
      *(outbuffer + offset + 0) = (u_rpm_rear_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rpm_rear_right.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rpm_rear_right);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->version =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->version);
      union {
        int16_t real;
        uint16_t base;
      } u_rpm_front_left;
      u_rpm_front_left.base = 0;
      u_rpm_front_left.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rpm_front_left.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->rpm_front_left = u_rpm_front_left.real;
      offset += sizeof(this->rpm_front_left);
      union {
        int16_t real;
        uint16_t base;
      } u_rpm_front_right;
      u_rpm_front_right.base = 0;
      u_rpm_front_right.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rpm_front_right.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->rpm_front_right = u_rpm_front_right.real;
      offset += sizeof(this->rpm_front_right);
      union {
        int16_t real;
        uint16_t base;
      } u_rpm_rear_left;
      u_rpm_rear_left.base = 0;
      u_rpm_rear_left.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rpm_rear_left.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->rpm_rear_left = u_rpm_rear_left.real;
      offset += sizeof(this->rpm_rear_left);
      union {
        int16_t real;
        uint16_t base;
      } u_rpm_rear_right;
      u_rpm_rear_right.base = 0;
      u_rpm_rear_right.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rpm_rear_right.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->rpm_rear_right = u_rpm_rear_right.real;
      offset += sizeof(this->rpm_rear_right);
     return offset;
    }

    const char * getType(){ return "fssim_common/WheelSpeeds"; };
    const char * getMD5(){ return "de6e76c895b1095f899172fc46f64a60"; };

  };

}
#endif