#include "shakey_planning_actions/goalCreatorObjects.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <set>
#include <std_srvs/Empty.h>

PLUGINLIB_DECLARE_CLASS(shakey_planning_actions, goal_creator_objects,
        shakey_planning_actions::GoalCreatorObjects, continual_planning_executive::GoalCreator)

namespace shakey_planning_actions
{

	GoalCreatorObjects::GoalCreatorObjects()
    {
    }

	GoalCreatorObjects::~GoalCreatorObjects()
    {
    }

    bool GoalCreatorObjects::fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal)
    {
    	goal.setBooleanPredicate("at-base", "loc1", "true");
        //goal.setForEachGoalStatement("movable_object", "pushed", true);
        return true;
    }

};
