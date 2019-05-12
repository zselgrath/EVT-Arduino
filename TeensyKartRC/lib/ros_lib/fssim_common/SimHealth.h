#ifndef _ROS_fssim_common_SimHealth_h
#define _ROS_fssim_common_SimHealth_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "fssim_common/TopicsHealth.h"

namespace fssim_common
{

  class SimHealth : public ros::Msg
  {
    public:
      typedef bool _request_shutdown_type;
      _request_shutdown_type request_shutdown;
      typedef bool _vehicle_started_type;
      _vehicle_started_type vehicle_started;
      typedef fssim_common::TopicsHealth _topics_health_type;
      _topics_health_type topics_health;

    SimHealth():
      request_shutdown(0),
      vehicle_started(0),
      topics_health()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_request_shutdown;
      u_request_shutdown.real = this->request_shutdown;
      *(outbuffer + offset + 0) = (u_request_shutdown.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->request_shutdown);
      union {
        bool real;
        uint8_t base;
      } u_vehicle_started;
      u_vehicle_started.real = this->vehicle_started;
      *(outbuffer + offset + 0) = (u_vehicle_started.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->vehicle_started);
      offset += this->topics_health.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_request_shutdown;
      u_request_shutdown.base = 0;
      u_request_shutdown.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->request_shutdown = u_request_shutdown.real;
      offset += sizeof(this->request_shutdown);
      union {
        bool real;
        uint8_t base;
      } u_vehicle_started;
      u_vehicle_started.base = 0;
      u_vehicle_started.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->vehicle_started = u_vehicle_started.real;
      offset += sizeof(this->vehicle_started);
      offset += this->topics_health.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "fssim_common/SimHealth"; };
    const char * getMD5(){ return "2d6de42391271dba371094f7524b84b6"; };

  };

}
#endif