#ifndef _ROS_fssim_common_Track_h
#define _ROS_fssim_common_Track_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Point.h"

namespace fssim_common
{

  class Track : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t cones_left_length;
      typedef geometry_msgs::Point _cones_left_type;
      _cones_left_type st_cones_left;
      _cones_left_type * cones_left;
      uint32_t cones_right_length;
      typedef geometry_msgs::Point _cones_right_type;
      _cones_right_type st_cones_right;
      _cones_right_type * cones_right;
      uint32_t cones_orange_length;
      typedef geometry_msgs::Point _cones_orange_type;
      _cones_orange_type st_cones_orange;
      _cones_orange_type * cones_orange;
      uint32_t cones_orange_big_length;
      typedef geometry_msgs::Point _cones_orange_big_type;
      _cones_orange_big_type st_cones_orange_big;
      _cones_orange_big_type * cones_orange_big;
      uint32_t tk_device_start_length;
      typedef geometry_msgs::Point _tk_device_start_type;
      _tk_device_start_type st_tk_device_start;
      _tk_device_start_type * tk_device_start;
      uint32_t tk_device_end_length;
      typedef geometry_msgs::Point _tk_device_end_type;
      _tk_device_end_type st_tk_device_end;
      _tk_device_end_type * tk_device_end;

    Track():
      header(),
      cones_left_length(0), cones_left(NULL),
      cones_right_length(0), cones_right(NULL),
      cones_orange_length(0), cones_orange(NULL),
      cones_orange_big_length(0), cones_orange_big(NULL),
      tk_device_start_length(0), tk_device_start(NULL),
      tk_device_end_length(0), tk_device_end(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->cones_left_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cones_left_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cones_left_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cones_left_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cones_left_length);
      for( uint32_t i = 0; i < cones_left_length; i++){
      offset += this->cones_left[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->cones_right_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cones_right_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cones_right_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cones_right_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cones_right_length);
      for( uint32_t i = 0; i < cones_right_length; i++){
      offset += this->cones_right[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->cones_orange_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cones_orange_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cones_orange_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cones_orange_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cones_orange_length);
      for( uint32_t i = 0; i < cones_orange_length; i++){
      offset += this->cones_orange[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->cones_orange_big_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cones_orange_big_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cones_orange_big_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cones_orange_big_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cones_orange_big_length);
      for( uint32_t i = 0; i < cones_orange_big_length; i++){
      offset += this->cones_orange_big[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->tk_device_start_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tk_device_start_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tk_device_start_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tk_device_start_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tk_device_start_length);
      for( uint32_t i = 0; i < tk_device_start_length; i++){
      offset += this->tk_device_start[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->tk_device_end_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tk_device_end_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tk_device_end_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tk_device_end_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tk_device_end_length);
      for( uint32_t i = 0; i < tk_device_end_length; i++){
      offset += this->tk_device_end[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t cones_left_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      cones_left_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      cones_left_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      cones_left_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->cones_left_length);
      if(cones_left_lengthT > cones_left_length)
        this->cones_left = (geometry_msgs::Point*)realloc(this->cones_left, cones_left_lengthT * sizeof(geometry_msgs::Point));
      cones_left_length = cones_left_lengthT;
      for( uint32_t i = 0; i < cones_left_length; i++){
      offset += this->st_cones_left.deserialize(inbuffer + offset);
        memcpy( &(this->cones_left[i]), &(this->st_cones_left), sizeof(geometry_msgs::Point));
      }
      uint32_t cones_right_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      cones_right_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      cones_right_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      cones_right_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->cones_right_length);
      if(cones_right_lengthT > cones_right_length)
        this->cones_right = (geometry_msgs::Point*)realloc(this->cones_right, cones_right_lengthT * sizeof(geometry_msgs::Point));
      cones_right_length = cones_right_lengthT;
      for( uint32_t i = 0; i < cones_right_length; i++){
      offset += this->st_cones_right.deserialize(inbuffer + offset);
        memcpy( &(this->cones_right[i]), &(this->st_cones_right), sizeof(geometry_msgs::Point));
      }
      uint32_t cones_orange_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      cones_orange_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      cones_orange_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      cones_orange_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->cones_orange_length);
      if(cones_orange_lengthT > cones_orange_length)
        this->cones_orange = (geometry_msgs::Point*)realloc(this->cones_orange, cones_orange_lengthT * sizeof(geometry_msgs::Point));
      cones_orange_length = cones_orange_lengthT;
      for( uint32_t i = 0; i < cones_orange_length; i++){
      offset += this->st_cones_orange.deserialize(inbuffer + offset);
        memcpy( &(this->cones_orange[i]), &(this->st_cones_orange), sizeof(geometry_msgs::Point));
      }
      uint32_t cones_orange_big_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      cones_orange_big_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      cones_orange_big_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      cones_orange_big_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->cones_orange_big_length);
      if(cones_orange_big_lengthT > cones_orange_big_length)
        this->cones_orange_big = (geometry_msgs::Point*)realloc(this->cones_orange_big, cones_orange_big_lengthT * sizeof(geometry_msgs::Point));
      cones_orange_big_length = cones_orange_big_lengthT;
      for( uint32_t i = 0; i < cones_orange_big_length; i++){
      offset += this->st_cones_orange_big.deserialize(inbuffer + offset);
        memcpy( &(this->cones_orange_big[i]), &(this->st_cones_orange_big), sizeof(geometry_msgs::Point));
      }
      uint32_t tk_device_start_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      tk_device_start_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      tk_device_start_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      tk_device_start_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->tk_device_start_length);
      if(tk_device_start_lengthT > tk_device_start_length)
        this->tk_device_start = (geometry_msgs::Point*)realloc(this->tk_device_start, tk_device_start_lengthT * sizeof(geometry_msgs::Point));
      tk_device_start_length = tk_device_start_lengthT;
      for( uint32_t i = 0; i < tk_device_start_length; i++){
      offset += this->st_tk_device_start.deserialize(inbuffer + offset);
        memcpy( &(this->tk_device_start[i]), &(this->st_tk_device_start), sizeof(geometry_msgs::Point));
      }
      uint32_t tk_device_end_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      tk_device_end_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      tk_device_end_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      tk_device_end_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->tk_device_end_length);
      if(tk_device_end_lengthT > tk_device_end_length)
        this->tk_device_end = (geometry_msgs::Point*)realloc(this->tk_device_end, tk_device_end_lengthT * sizeof(geometry_msgs::Point));
      tk_device_end_length = tk_device_end_lengthT;
      for( uint32_t i = 0; i < tk_device_end_length; i++){
      offset += this->st_tk_device_end.deserialize(inbuffer + offset);
        memcpy( &(this->tk_device_end[i]), &(this->st_tk_device_end), sizeof(geometry_msgs::Point));
      }
     return offset;
    }

    const char * getType(){ return "fssim_common/Track"; };
    const char * getMD5(){ return "b3e47dce05b0e0c04dc61079e1408cdf"; };

  };

}
#endif