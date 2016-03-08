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
    	Predicate p;
    	double push_distance;
    	p.parameters.push_back(a.parameters[0]);
    	p.name = "push-distance";
    	current.hasNumericalFluent(p, &push_distance);
    	float base_width = 0.65;

    	goal.push_distance = push_distance + 0.75 - base_width / 2;
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
