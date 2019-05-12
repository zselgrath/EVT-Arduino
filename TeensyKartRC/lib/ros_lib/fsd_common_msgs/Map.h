#ifndef _ROS_fsd_common_msgs_Map_h
#define _ROS_fsd_common_msgs_Map_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "fsd_common_msgs/Cone.h"

namespace fsd_common_msgs
{

  class Map : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t cone_yellow_length;
      typedef fsd_common_msgs::Cone _cone_yellow_type;
      _cone_yellow_type st_cone_yellow;
      _cone_yellow_type * cone_yellow;
      uint32_t cone_blue_length;
      typedef fsd_common_msgs::Cone _cone_blue_type;
      _cone_blue_type st_cone_blue;
      _cone_blue_type * cone_blue;
      uint32_t cone_orange_length;
      typedef fsd_common_msgs::Cone _cone_orange_type;
      _cone_orange_type st_cone_orange;
      _cone_orange_type * cone_orange;

    Map():
      header(),
      cone_yellow_length(0), cone_yellow(NULL),
      cone_blue_length(0), cone_blue(NULL),
      cone_orange_length(0), cone_orange(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->cone_yellow_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cone_yellow_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cone_yellow_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cone_yellow_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cone_yellow_length);
      for( uint32_t i = 0; i < cone_yellow_length; i++){
      offset += this->cone_yellow[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->cone_blue_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cone_blue_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cone_blue_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cone_blue_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cone_blue_length);
      for( uint32_t i = 0; i < cone_blue_length; i++){
      offset += this->cone_blue[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->cone_orange_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cone_orange_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cone_orange_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cone_orange_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cone_orange_length);
      for( uint32_t i = 0; i < cone_orange_length; i++){
      offset += this->cone_orange[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t cone_yellow_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      cone_yellow_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      cone_yellow_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      cone_yellow_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->cone_yellow_length);
      if(cone_yellow_lengthT > cone_yellow_length)
        this->cone_yellow = (fsd_common_msgs::Cone*)realloc(this->cone_yellow, cone_yellow_lengthT * sizeof(fsd_common_msgs::Cone));
      cone_yellow_length = cone_yellow_lengthT;
      for( uint32_t i = 0; i < cone_yellow_length; i++){
      offset += this->st_cone_yellow.deserialize(inbuffer + offset);
        memcpy( &(this->cone_yellow[i]), &(this->st_cone_yellow), sizeof(fsd_common_msgs::Cone));
      }
      uint32_t cone_blue_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      cone_blue_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      cone_blue_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      cone_blue_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->cone_blue_length);
      if(cone_blue_lengthT > cone_blue_length)
        this->cone_blue = (fsd_common_msgs::Cone*)realloc(this->cone_blue, cone_blue_lengthT * sizeof(fsd_common_msgs::Cone));
      cone_blue_length = cone_blue_lengthT;
      for( uint32_t i = 0; i < cone_blue_length; i++){
      offset += this->st_cone_blue.deserialize(inbuffer + offset);
        memcpy( &(this->cone_blue[i]), &(this->st_cone_blue), sizeof(fsd_common_msgs::Cone));
      }
      uint32_t cone_orange_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      cone_orange_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      cone_orange_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      cone_orange_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->cone_orange_length);
      if(cone_orange_lengthT > cone_orange_length)
        this->cone_orange = (fsd_common_msgs::Cone*)realloc(this->cone_orange, cone_orange_lengthT * sizeof(fsd_common_msgs::Cone));
      cone_orange_length = cone_orange_lengthT;
      for( uint32_t i = 0; i < cone_orange_length; i++){
      offset += this->st_cone_orange.deserialize(inbuffer + offset);
        memcpy( &(this->cone_orange[i]), &(this->st_cone_orange), sizeof(fsd_common_msgs::Cone));
      }
     return offset;
    }

    const char * getType(){ return "fsd_common_msgs/Map"; };
    const char * getMD5(){ return "8ddb8bd148fb7730a132faec3d415fe0"; };

  };

}
#endif