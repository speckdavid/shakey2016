#include "shakey_planning_actions/actionExecutorPushObject.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(shakey_planning_actions, action_executor_push_object,
		shakey_planning_actions::ActionExecutorPushObject,
		continual_planning_executive::ActionExecutorInterface)

namespace shakey_planning_actions {
bool ActionExecutorPushObject::fillGoal(shakey_actionlib::PushGoal & goal,
		const DurativeAction & a, const SymbolicState & current) {
	Predicate p;
	p.parameters.push_back(a.parameters[1]);
	// get push distance
	double push_distance;
	p.name = "push-distance";
	current.hasNumericalFluent(p, &push_distance);
	float base_size = 0.65;
	goal.push_distance.push_back(push_distance + 0.75 - base_size / 2 );
	geometry_msgs::Pose push1 = getLocation(a, current, a.parameters[1]);
	goal.push_poses.push_back(push1);

	// Do the same with second one if double push action
	if (a.name == "push-object-to-pos") {
		// Get push and destination poses
		geometry_msgs::Pose push2 = getLocation(a, current, a.parameters[2]);
		goal.push_poses.push_back(push2);
		geometry_msgs::Pose dest = getLocation(a, current, a.parameters[3]);
		MapHelper mh;
		// Compute necessary push distances
		mh.getPushDistances(push1, push2, dest, &_push_vector);
		std::cout << "DEST:\n[" << dest.position.x << ", " << dest.position.y
				<< ", " << dest.position.z << "]" << std::endl;
		std::cout << "\n[" << push1.position.x << ", " << push1.position.y
				<< ", " << push1.position.z << "]  with " << _push_vector.x() << "meters" << std::endl;
		std::cout << "[" << push2.position.x << ", " << push2.position.y << ", "
				<< push2.position.z << "] with " << _push_vector.y() << "meters" << std::endl;

		// Set push distances
		goal.push_distance.at(0) = _push_vector.x() + 0.75;
		goal.push_distance.push_back(_push_vector.y() + 0.75);
	}
	return true;
}

void ActionExecutorPushObject::updateState(
		const actionlib::SimpleClientGoalState & actionReturnState,
		const shakey_actionlib::PushResult & result, const DurativeAction & a,
		SymbolicState & current) {
	std::string object_name = a.parameters[0];
	current.setBooleanPredicate("pushed", object_name, true);
	Predicate p;
	p.parameters.push_back(object_name);
	p.name = "belongs-to-doorway";
	std::string doorway;
	if (current.hasObjectFluent(p, &doorway)) {
		current.setBooleanPredicate("doorway-state-known", doorway, false);
	}
	if (a.name == "push-object-to-pos") {
		current.setBooleanPredicate("object_at", a.parameters[3], true);
		current.setBooleanPredicate("object_occupied", a.parameters[0], true);
	}
	//current.removeObject(object_name, "true");

}

geometry_msgs::Pose ActionExecutorPushObject::getLocation(
		const DurativeAction & a, const SymbolicState & current,
		std::string param) {
	geometry_msgs::Pose result;
	Predicate p;
	p.parameters.push_back(param);
	double qX, qY, qZ, qW, x, y, z;
	p.name = "qx";
	current.hasNumericalFluent(p, &qX);
	p.name = "qy";
	current.hasNumericalFluent(p, &qY);
	p.name = "qz";
	current.hasNumericalFluent(p, &qZ);
	p.name = "qw";
	current.hasNumericalFluent(p, &qW);
	p.name = "x";
	current.hasNumericalFluent(p, &x);
	p.name = "y";
	current.hasNumericalFluent(p, &y);
	p.name = "z";
	current.hasNumericalFluent(p, &z);
	result.orientation.x = qX;
	result.orientation.y = qY;
	result.orientation.z = qZ;
	result.orientation.w = qW;
	result.position.x = x;
	result.position.y = y;
	result.position.z = z;
	return result;
}

}
;
