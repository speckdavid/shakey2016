#ifndef ACTION_PUSH_OBJECT_H
#define ACTION_PUSH_OBJECT_H

#include "continual_planning_executive/actionExecutorActionlib.hpp"
#include "continual_planning_executive/symbolicState.h"
#include "shakey_actionlib/PushAction.h"
#include "geometry_msgs/Pose.h"
#include "shakey_utils/MapHelper.h"
#include "tf/transform_listener.h"

namespace shakey_planning_actions {

class ActionExecutorPushObject: public ActionExecutorActionlib<
		shakey_actionlib::PushAction, shakey_actionlib::PushGoal,
		shakey_actionlib::PushResult> {
public:
	virtual bool fillGoal(shakey_actionlib::PushGoal & goal,
			const DurativeAction & a, const SymbolicState & current);

	virtual void updateState(
			const actionlib::SimpleClientGoalState & actionReturnState,
			const shakey_actionlib::PushResult & result,
			const DurativeAction & a, SymbolicState & current);

private:
	geometry_msgs::Pose getLocation(const DurativeAction & a,
			const SymbolicState & current, std::string param);
	VectorXf _push_vector;
	tf::TransformListener _tf;
};

}
;

#endif
