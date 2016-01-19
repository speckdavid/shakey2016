#include "shakey_planning_actions/actionExecutorDriveBase.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(shakey_planning_actions, action_executor_drive_base,
        shakey_planning_actions::ActionExecutorDriveBase,
        continual_planning_executive::ActionExecutorInterface)

namespace shakey_planning_actions
{

    void ActionExecutorDriveBase::initialize(const std::deque<std::string> & arguments)
    {
        ActionExecutorActionlib<move_base_msgs::MoveBaseAction, move_base_msgs::MoveBaseGoal,
            move_base_msgs::MoveBaseResult>::initialize(arguments);

        if(arguments.size() < 3)
            return;
    }

    bool ActionExecutorDriveBase::fillGoal(move_base_msgs::MoveBaseGoal & goal,
            const DurativeAction & a, const SymbolicState & current)
    {

        // FIXME: don't get from state (very old), but the newest.
        // The frame_id should be a fixed frame anyways
        goal.target_pose.header.stamp = ros::Time::now();

        ROS_ASSERT(a.parameters.size() == 2);
        string targetName = a.parameters[1];

        // extract nicer + warn.
        Predicate p;
        p.parameters.push_back(targetName);
        p.name = "frame-id";
        if(!current.hasObjectFluent(p, &goal.target_pose.header.frame_id))
            return false;
        p.name = "x";
        if(!current.hasNumericalFluent(p, &goal.target_pose.pose.position.x))
            return false;
        p.name = "y";
        if(!current.hasNumericalFluent(p, &goal.target_pose.pose.position.y))
            return false;
        p.name = "z";
        if(!current.hasNumericalFluent(p, &goal.target_pose.pose.position.z))
            return false;
        p.name = "qx";
        if(!current.hasNumericalFluent(p, &goal.target_pose.pose.orientation.x))
            return false;
        p.name = "qy";
        if(!current.hasNumericalFluent(p, &goal.target_pose.pose.orientation.y))
            return false;
        p.name = "qz";
        if(!current.hasNumericalFluent(p, &goal.target_pose.pose.orientation.z))
            return false;
        p.name = "qw";
        if(!current.hasNumericalFluent(p, &goal.target_pose.pose.orientation.w))
            return false;

        ROS_INFO_STREAM("Created goal for ActionExecutorDriveBase as: " << goal);

        return true;
    }

    void ActionExecutorDriveBase::updateState(const actionlib::SimpleClientGoalState & actionReturnState,
            const move_base_msgs::MoveBaseResult & result,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_ASSERT(a.parameters.size() == 2);
        string startName = a.parameters[0];
        string targetName = a.parameters[1];

        // start predicates are always applied independent of success
        for(std::vector<std::pair<std::string, bool> >::iterator it = _startPredicates.begin();
                it != _startPredicates.end(); it++) {
            current.setBooleanPredicate(it->first, startName, it->second);
        }

        if(actionReturnState == actionlib::SimpleClientGoalState::SUCCEEDED) {
            for(std::vector<std::pair<std::string, bool> >::iterator it = _goalPredicates.begin();
                    it != _goalPredicates.end(); it++) {
                current.setBooleanPredicate(it->first, targetName, it->second);
            }
        }
    }

};
