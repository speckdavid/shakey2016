#include "shakey_planning_actions/actionExecutorDetectObjects.h"
#include "shakey_object_recognition/PushableObject.h"
#include "shakey_object_recognition/ObjectVisualisation.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <sstream>

PLUGINLIB_DECLARE_CLASS(shakey_planning_actions, action_executor_detect_objects,
		shakey_planning_actions::ActionExecutorDetectObjects,
		continual_planning_executive::ActionExecutorInterface)

namespace shakey_planning_actions {
void ActionExecutorDetectObjects::initialize(
		const std::deque<std::string> & arguments) {
	// Create Template for the service
	ActionExecutorService<shakey_object_recognition::DetectObjects>::initialize(
			arguments);
}

bool ActionExecutorDetectObjects::fillGoal(
		shakey_object_recognition::DetectObjects::Request & goal,
		const DurativeAction & a, const SymbolicState & current) {
	return true;
}

void ActionExecutorDetectObjects::updateState(bool& success,
		shakey_object_recognition::DetectObjects::Response & response,
		const DurativeAction & a, SymbolicState & current) {
	if (!success)
		return;
	for (int i = 0; i < response.objects.size(); i++) {
		shakey_object_recognition::PushableObject object = response.objects.at(
				i);
		if (object.obj_type == Box || object.obj_type == Wedge) {
			std::string obj_name;
			obj_name = (object.obj_type == Box) ? "box" : "wedge";
			std::ostringstream os;
			os << i << "_";
			obj_name += os.str();
			current.setObjectFluent("belongs-to-search-location", obj_name,
					a.name == "detect-objects" ?
							a.parameters[0] : a.parameters[1]);
			current.addObject(obj_name, "movable_object");
			current.setBooleanPredicate("pushed", obj_name, false);
			// TODO: Some magic for only one push_location!!!
			//-> normal dection: Push direction to wall + add distance to wall to object
			//-> detect doorway: One Puslocation which is at least 30° different
			int j = 0;
			geometry_msgs::Pose cur = object.push_poses.at(j);
			// FIXME: Don't take a location directly at the doorway in -> only take some which
			// push box away
			if (a.name == "detect-doorway-state") {
				Predicate p;
				p.parameters.push_back(a.parameters[0]);
				double qX, qY, qZ, qW;
				p.name = "qx";
				current.hasNumericalFluent(p, &qX);
				p.name = "qy";
				current.hasNumericalFluent(p, &qZ);
				p.name = "qz";
				current.hasNumericalFluent(p, &qW);
				p.name = "qw";
				current.hasNumericalFluent(p, &qY);
				for (int k = 0; k < object.push_poses.size(); i++) {
					geometry_msgs::Pose cur1 = object.push_poses.at(k);
					float dot = cur1.orientation.x * qX
							+ cur1.orientation.y * qY + cur1.orientation.z * qZ
							+ cur1.orientation.w * qW;
					float ang = acos(dot) / M_PI * 180;
					if (std::abs(ang - 90) < 10) {
						cur = cur1;
						break;
					}
				}
			}
			std::ostringstream os1;
			os1 << j;
			std::string push_loc_name = obj_name + "_push_loc_" + os1.str();
			current.addObject(push_loc_name, "pushable_location");
			current.setObjectFluent("frame-id", push_loc_name, "/map");
			current.setNumericalFluent("x", push_loc_name, cur.position.x);
			current.setNumericalFluent("y", push_loc_name, cur.position.y);
			current.setNumericalFluent("z", push_loc_name, cur.position.z);
			current.setNumericalFluent("qx", push_loc_name, cur.orientation.x);
			current.setNumericalFluent("qy", push_loc_name, cur.orientation.y);
			current.setNumericalFluent("qz", push_loc_name, cur.orientation.z);
			current.setNumericalFluent("qw", push_loc_name, cur.orientation.w);
			current.setNumericalFluent("timestamp", push_loc_name,
					ros::Time::now().toSec());
			current.setObjectFluent("belongs-to-movable-object", push_loc_name,
					obj_name);

			// Set object location to current position of detection loation
			Predicate p;
			std::string det_loc = a.parameters[0];
			p.parameters.push_back(det_loc);
			p.name = "location-in-room";
			std::string new_room;
			if (!current.hasObjectFluent(p, &new_room)) {
				ROS_ERROR("%s: det_loc: %s - no location-in-room in state.",
						__func__, det_loc.c_str());
			}
			current.setObjectFluent("location-in-room", push_loc_name,
					new_room);


		}
	}
	if (a.name == "detect-objects")
		current.setBooleanPredicate("searched", a.parameters[0], true);
	if (a.name == "detect-doorway-state")
		current.setBooleanPredicate("doorway-state-known", a.parameters[1],
				true);
	ROS_INFO("DetectObjects returned result");

}

void ActionExecutorDetectObjects::getBestPushPose(
		shakey_object_recognition::PushableObject) {

}

}
