#ifndef _ROS_fssim_common_CarInfo_h
#define _ROS_fssim_common_CarInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "fssim_common/Vector3Ext.h"

namespace fssim_common
{

  class CarInfo : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef fssim_common::Vector3Ext _drag_force_type;
      _drag_force_type drag_force;
      typedef float _delta_type;
      _delta_type delta;
      typedef float _dc_type;
      _dc_type dc;
      typedef float _front_left_steering_angle_type;
      _front_left_steering_angle_type front_left_steering_angle;
      typedef float _front_right_steering_angle_type;
      _front_right_steering_angle_type front_right_steering_angle;
      typedef float _delta_measured_type;
      _delta_measured_type delta_measured;
      typedef float _vx_type;
      _vx_type vx;
      typedef float _vy_type;
      _vy_type vy;
      typedef float _r_type;
      _r_type r;
      typedef bool _torque_ok_type;
      _torque_ok_type torque_ok;
      typedef float _alpha_f_type;
      _alpha_f_type alpha_f;
      typedef float _alpha_f_l_type;
      _alpha_f_l_type alpha_f_l;
      typedef float _alpha_f_r_type;
      _alpha_f_r_type alpha_f_r;
      typedef float _alpha_r_l_type;
      _alpha_r_l_type alpha_r_l;
      typedef float _alpha_r_type;
      _alpha_r_type alpha_r;
      typedef float _alpha_r_r_type;
      _alpha_r_r_type alpha_r_r;
      typedef float _Fy_f_type;
      _Fy_f_type Fy_f;
      typedef float _Fy_f_l_type;
      _Fy_f_l_type Fy_f_l;
      typedef float _Fy_f_r_type;
      _Fy_f_r_type Fy_f_r;
      typedef float _Fy_r_type;
      _Fy_r_type Fy_r;
      typedef float _Fy_r_l_type;
      _Fy_r_l_type Fy_r_l;
      typedef float _Fy_r_r_type;
      _Fy_r_r_type Fy_r_r;
      typedef float _Fx_type;
      _Fx_type Fx;

    CarInfo():
      header(),
      drag_force(),
      delta(0),
      dc(0),
      front_left_steering_angle(0),
      front_right_steering_angle(0),
      delta_measured(0),
      vx(0),
      vy(0),
      r(0),
      torque_ok(0),
      alpha_f(0),
      alpha_f_l(0),
      alpha_f_r(0),
      alpha_r_l(0),
      alpha_r(0),
      alpha_r_r(0),
      Fy_f(0),
      Fy_f_l(0),
      Fy_f_r(0),
      Fy_r(0),
      Fy_r_l(0),
      Fy_r_r(0),
      Fx(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->drag_force.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->delta);
      offset += serializeAvrFloat64(outbuffer + offset, this->dc);
      offset += serializeAvrFloat64(outbuffer + offset, this->front_left_steering_angle);
      offset += serializeAvrFloat64(outbuffer + offset, this->front_right_steering_angle);
      offset += serializeAvrFloat64(outbuffer + offset, this->delta_measured);
      offset += serializeAvrFloat64(outbuffer + offset, this->vx);
      offset += serializeAvrFloat64(outbuffer + offset, this->vy);
      offset += serializeAvrFloat64(outbuffer + offset, this->r);
      union {
        bool real;
        uint8_t base;
      } u_torque_ok;
      u_torque_ok.real = this->torque_ok;
      *(outbuffer + offset + 0) = (u_torque_ok.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->torque_ok);
      offset += serializeAvrFloat64(outbuffer + offset, this->alpha_f);
      offset += serializeAvrFloat64(outbuffer + offset, this->alpha_f_l);
      offset += serializeAvrFloat64(outbuffer + offset, this->alpha_f_r);
      offset += serializeAvrFloat64(outbuffer + offset, this->alpha_r_l);
      offset += serializeAvrFloat64(outbuffer + offset, this->alpha_r);
      offset += serializeAvrFloat64(outbuffer + offset, this->alpha_r_r);
      offset += serializeAvrFloat64(outbuffer + offset, this->Fy_f);
      offset += serializeAvrFloat64(outbuffer + offset, this->Fy_f_l);
      offset += serializeAvrFloat64(outbuffer + offset, this->Fy_f_r);
      offset += serializeAvrFloat64(outbuffer + offset, this->Fy_r);
      offset += serializeAvrFloat64(outbuffer + offset, this->Fy_r_l);
      offset += serializeAvrFloat64(outbuffer + offset, this->Fy_r_r);
      offset += serializeAvrFloat64(outbuffer + offset, this->Fx);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->drag_force.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->delta));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->dc));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->front_left_steering_angle));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->front_right_steering_angle));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->delta_measured));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->vx));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->vy));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->r));
      union {
        bool real;
        uint8_t base;
      } u_torque_ok;
      u_torque_ok.base = 0;
      u_torque_ok.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->torque_ok = u_torque_ok.real;
      offset += sizeof(this->torque_ok);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->alpha_f));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->alpha_f_l));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->alpha_f_r));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->alpha_r_l));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->alpha_r));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->alpha_r_r));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->Fy_f));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->Fy_f_l));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->Fy_f_r));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->Fy_r));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->Fy_r_l));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->Fy_r_r));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->Fx));
     return offset;
    }

    const char * getType(){ return "fssim_common/CarInfo"; };
    const char * getMD5(){ return "d1d7b96c5e9f10a89a35df8f3f330a2c"; };

  };

}
#endif