#ifndef _ROS_nav2d_navigator_MoveToPosition2DActionFeedback_h
#define _ROS_nav2d_navigator_MoveToPosition2DActionFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "nav2d_navigator/MoveToPosition2DFeedback.h"

namespace nav2d_navigator
{

  class MoveToPosition2DActionFeedback : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef nav2d_navigator::MoveToPosition2DFeedback _feedback_type;
      _feedback_type feedback;

    MoveToPosition2DActionFeedback():
      header(),
      status(),
      feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->feedback.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "nav2d_navigator/MoveToPosition2DActionFeedback"; };
    const char * getMD5(){ return "e63058c3827d4bb2ecc233eb98442965"; };

  };

}
#endif