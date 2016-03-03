#include "shakey_planning_actions/actionExecutorPushObject.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(shakey_planning_actions, action_executor_push_object,
		shakey_planning_actions::ActionExecutorPushObject,
        continual_planning_executive::ActionExecutorInterface)

namespace shakey_planning_actions
{
    bool ActionExecutorPushObject::fillGoal(shakey_actionlib::PushGoal & goal,
            const DurativeAction & a, const SymbolicState & current)
    {
    	goal.push_distance = 1.5 + 0.75;
        return true;
    }

    void ActionExecutorPushObject::updateState(const actionlib::SimpleClientGoalState & actionReturnState,
    		const shakey_actionlib::PushResult & result,
            const DurativeAction & a, SymbolicState & current)
    {
    	std::string object_name = a.parameters[0];
    	current.setBooleanPredicate("pushed", object_name, true);
    	// TODO: set doorway state to unkown

    }

};
