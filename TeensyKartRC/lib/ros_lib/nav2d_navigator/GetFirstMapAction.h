#ifndef _ROS_nav2d_navigator_GetFirstMapAction_h
#define _ROS_nav2d_navigator_GetFirstMapAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "nav2d_navigator/GetFirstMapActionGoal.h"
#include "nav2d_navigator/GetFirstMapActionResult.h"
#include "nav2d_navigator/GetFirstMapActionFeedback.h"

namespace nav2d_navigator
{

  class GetFirstMapAction : public ros::Msg
  {
    public:
      typedef nav2d_navigator::GetFirstMapActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef nav2d_navigator::GetFirstMapActionResult _action_result_type;
      _action_result_type action_result;
      typedef nav2d_navigator::GetFirstMapActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    GetFirstMapAction():
      action_goal(),
      action_result(),
      action_feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->action_goal.serialize(outbuffer + offset);
      offset += this->action_result.serialize(outbuffer + offset);
      offset += this->action_feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->action_goal.deserialize(inbuffer + offset);
      offset += this->action_result.deserialize(inbuffer + offset);
      offset += this->action_feedback.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "nav2d_navigator/GetFirstMapAction"; };
    const char * getMD5(){ return "d5a016b49f278075666fbc901debbd08"; };

  };

}
#endif