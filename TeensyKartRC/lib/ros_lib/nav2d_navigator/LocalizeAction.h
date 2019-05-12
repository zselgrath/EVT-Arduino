#ifndef _ROS_nav2d_navigator_LocalizeAction_h
#define _ROS_nav2d_navigator_LocalizeAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "nav2d_navigator/LocalizeActionGoal.h"
#include "nav2d_navigator/LocalizeActionResult.h"
#include "nav2d_navigator/LocalizeActionFeedback.h"

namespace nav2d_navigator
{

  class LocalizeAction : public ros::Msg
  {
    public:
      typedef nav2d_navigator::LocalizeActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef nav2d_navigator::LocalizeActionResult _action_result_type;
      _action_result_type action_result;
      typedef nav2d_navigator::LocalizeActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    LocalizeAction():
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

    const char * getType(){ return "nav2d_navigator/LocalizeAction"; };
    const char * getMD5(){ return "c6b44763ef96b288abc7dd265b2d5353"; };

  };

}
#endif