#ifndef _ROS_nav2d_navigator_ExploreAction_h
#define _ROS_nav2d_navigator_ExploreAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "nav2d_navigator/ExploreActionGoal.h"
#include "nav2d_navigator/ExploreActionResult.h"
#include "nav2d_navigator/ExploreActionFeedback.h"

namespace nav2d_navigator
{

  class ExploreAction : public ros::Msg
  {
    public:
      typedef nav2d_navigator::ExploreActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef nav2d_navigator::ExploreActionResult _action_result_type;
      _action_result_type action_result;
      typedef nav2d_navigator::ExploreActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    ExploreAction():
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

    const char * getType(){ return "nav2d_navigator/ExploreAction"; };
    const char * getMD5(){ return "5844eb91ed4a31b07ad7476dced45180"; };

  };

}
#endif