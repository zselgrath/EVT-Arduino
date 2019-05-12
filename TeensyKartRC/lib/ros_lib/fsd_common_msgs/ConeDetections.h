#ifndef _ROS_fsd_common_msgs_ConeDetections_h
#define _ROS_fsd_common_msgs_ConeDetections_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "fsd_common_msgs/Cone.h"

namespace fsd_common_msgs
{

  class ConeDetections : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t cone_detections_length;
      typedef fsd_common_msgs::Cone _cone_detections_type;
      _cone_detections_type st_cone_detections;
      _cone_detections_type * cone_detections;

    ConeDetections():
      header(),
      cone_detections_length(0), cone_detections(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->cone_detections_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cone_detections_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cone_detections_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cone_detections_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cone_detections_length);
      for( uint32_t i = 0; i < cone_detections_length; i++){
      offset += this->cone_detections[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t cone_detections_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      cone_detections_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      cone_detections_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      cone_detections_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->cone_detections_length);
      if(cone_detections_lengthT > cone_detections_length)
        this->cone_detections = (fsd_common_msgs::Cone*)realloc(this->cone_detections, cone_detections_lengthT * sizeof(fsd_common_msgs::Cone));
      cone_detections_length = cone_detections_lengthT;
      for( uint32_t i = 0; i < cone_detections_length; i++){
      offset += this->st_cone_detections.deserialize(inbuffer + offset);
        memcpy( &(this->cone_detections[i]), &(this->st_cone_detections), sizeof(fsd_common_msgs::Cone));
      }
     return offset;
    }

    const char * getType(){ return "fsd_common_msgs/ConeDetections"; };
    const char * getMD5(){ return "905dd9eb6d37422f728e412e2f15b0eb"; };

  };

}
#endif