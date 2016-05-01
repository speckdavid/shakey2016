#include "shakey_planning_actions/actionExecutorDetectObjects.h"
#include "shakey_object_recognition/PushableObject.h"
#include "shakey_object_recognition/ObjectVisualisation.h"
#include <actionlib/client/simple_action_client.h>
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
	_objVis.initialise("Destination_Objects", "/map");
	_map_client = _nh.serviceClient<nav_msgs::GetMap>("/static_map");
	if (!_map_client.call(_mapResponse))
		ROS_ERROR("No static map found.");
	_plan_client = _nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
	if (arguments.at(0) == "detect-doorway-state") {
		_nh.getParam(
				"continual_planning_executive/push_distance_object_doorway",
				_push_distance);
		if (_push_distance <= 0) {
			ROS_WARN("Push distance is smaller than 0. Set to 2");
			_push_distance = 2;
		}
	}
	if (arguments.at(0) == "detect-objects") {
		_nh.getParam("continual_planning_executive/push_distance_object_normal",
				_push_distance);
		if (_push_distance < 0) {
			_push_distance = std::numeric_limits<float>::infinity();
		}
	}
}

bool ActionExecutorDetectObjects::fillGoal(
		shakey_object_recognition::DetectObjects::Request & goal,
		const DurativeAction & a, const SymbolicState & current) {
	return true;
}

void ActionExecutorDetectObjects::updateState(bool& success,
		shakey_object_recognition::DetectObjects::Response & response,
		const DurativeAction & a, SymbolicState & current) {
	if (!success || _mapResponse.response.map.data.empty())
		return;
	_objVis.resetMarker();
	for (int i = 0; i < response.objects.size(); i++) {
		shakey_object_recognition::PushableObject object = response.objects.at(
				i);
		if (object.obj_type == Box || object.obj_type == Wedge) {
			geometry_msgs::Pose detection_position = getCurrentLocation(a,
					current);
			// Create name
			std::string obj_name;
			obj_name = (object.obj_type == Box) ? "box" : "wedge";
			std::ostringstream os;
			os << i;
			obj_name += os.str() + "_" + a.parameters[0];

			// Iterate over all push poses
			for (int j = 0; j < object.push_poses.size(); j++) {
				geometry_msgs::Pose cur = object.push_poses.at(j);
				double distance = getWallDistance(cur.position, object);
				distance = std::min((double) distance,
						_push_distance);
				double push_cost = distance;

				// Detected Object at Search location -> push to wall or push distance
				if (a.name == "detect-objects") {
					current.setObjectFluent("belongs-to-search-location",
							obj_name, a.parameters[0]);
				}
				// Detect Objects for doorways
				if (a.name == "detect-doorway-state") {
					current.setObjectFluent("belongs-to-doorway", obj_name,
							a.parameters[1]);

					float dot = cur.orientation.x
							* detection_position.orientation.x
							+ cur.orientation.y
									* detection_position.orientation.y
							+ cur.orientation.z
									* detection_position.orientation.z
							+ cur.orientation.w
									* detection_position.orientation.w;
					float ang = 2 * (acos(dot) / M_PI * 180);
					// Minima at 90...wierd can not be 0 or planner bugs
					// a driving action +1000 to be more expensive then
					push_cost = ((90 - ang) * (90 - ang) / 810 + 1000) - distance;
				}
				// Add ererything to state
				current.addObject(obj_name, "movable_object");
				bool object_pushed = distance < 0.15;
				current.setBooleanPredicate("pushed", obj_name, object_pushed);
				std::ostringstream os1;
				os1 << j;
				std::string push_loc_name = obj_name + "_push_loc_" + os1.str();
				ROS_INFO("%s cost: %f", push_loc_name.c_str(), push_cost);
				current.addObject(push_loc_name, "pushable_location");
				current.setObjectFluent("frame-id", push_loc_name, "/map");
				current.setNumericalFluent("x", push_loc_name, cur.position.x);
				current.setNumericalFluent("y", push_loc_name, cur.position.y);
				current.setNumericalFluent("z", push_loc_name, cur.position.z);
				current.setNumericalFluent("qx", push_loc_name,
						cur.orientation.x);
				current.setNumericalFluent("qy", push_loc_name,
						cur.orientation.y);
				current.setNumericalFluent("qz", push_loc_name,
						cur.orientation.z);
				current.setNumericalFluent("qw", push_loc_name,
						cur.orientation.w);
				current.setNumericalFluent("timestamp", push_loc_name,
						ros::Time::now().toSec());
				current.setObjectFluent("belongs-to-movable-object",
						push_loc_name, obj_name);
				current.setNumericalFluent("push-distance", push_loc_name,
						distance);
				current.setNumericalFluent("push-cost", push_loc_name,
						(int) (push_cost * 100));
				// Set object to occupied
				if (a.name == "detect-doorway-state")
					current.setBooleanPredicate("object_occupied", obj_name, true);

				// Set object location to current position of detection location
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
				current.setObjectFluent("in-room", obj_name, new_room);
				//Visualize the destination point
				visualize(object, cur, distance);
			}
		}
	}
	_objVis.publish();
	if (a.name == "detect-objects")
		current.setBooleanPredicate("searched", a.parameters[0], true);
	if (a.name == "detect-doorway-state")
		current.setBooleanPredicate("doorway-state-known", a.parameters[1],
				true);
	ROS_INFO("DetectObjects returned result");

}

void ActionExecutorDetectObjects::visualize(
		shakey_object_recognition::PushableObject object,
		geometry_msgs::Pose push_pose, double dist) {
	shakey_object_recognition::PushableObject new_object = object;
	new_object.push_poses.clear();
	Eigen::Vector3f direction = Eigen::Vector3f(object.mean.position.x,
			object.mean.position.y, 0)
			- Eigen::Vector3f(push_pose.position.x, push_pose.position.y, 0);
	float transX = direction.normalized().x() * dist;
	float transY = direction.normalized().y() * dist;
	new_object.mean.position.x = object.mean.position.x + transX;
	new_object.mean.position.y = object.mean.position.y + transY;
	new_object.corner_points = object.corner_points;
	for (int b = 0; b < new_object.corner_points.size(); b++) {
		new_object.corner_points.at(b).x += transX;
		new_object.corner_points.at(b).y += transY;
	}
	std_msgs::ColorRGBA color;
	color.b = color.a = 1;
	_objVis.addObjectMarker(new_object, color);

}

float ActionExecutorDetectObjects::getOccValue(Eigen::Vector3f pos) {
	nav_msgs::OccupancyGrid map = _mapResponse.response.map;
	unsigned int grid_x = (unsigned int) ((pos.x() - map.info.origin.position.x)
			/ map.info.resolution);
	unsigned int grid_y = (unsigned int) ((pos.y() - map.info.origin.position.y)
			/ map.info.resolution);
	unsigned int cell = grid_y * map.info.width + grid_x;
	if (grid_x >= map.info.width || grid_y >= map.info.height) {
		std::ostringstream os;
		os << pos.x() << ", " << pos.y() << ", " << pos.z();
		ROS_WARN("Position [%s] is not in static map.", os.str().c_str());
		return -1;
	}
	return map.data[cell];
}

float ActionExecutorDetectObjects::getWallDistance(geometry_msgs::Point pos,
		shakey_object_recognition::PushableObject obj) {
	Eigen::Vector3f cur_pos = Eigen::Vector3f(pos.x, pos.y, 0);
	Eigen::Vector3f mean = Eigen::Vector3f(obj.mean.position.x,
			obj.mean.position.y, 0);
	Eigen::Vector3f direction = (mean - cur_pos).normalized();
	double distance = 0;
	Eigen::Vector3f add_vector = direction
			* _mapResponse.response.map.info.resolution;
	while (true) {
		for (int i = 0; i < obj.corner_points.size(); i++) {
			Eigen::Vector3f cur_cell(obj.corner_points.at(i).x,
					obj.corner_points.at(i).y, 0);
			cur_cell += add_vector;
			double occ_value = getOccValue(cur_cell);
			if (occ_value == -1 || occ_value == 100)
				return distance - _mapResponse.response.map.info.resolution;
		}
		add_vector += direction * _mapResponse.response.map.info.resolution;
		distance += _mapResponse.response.map.info.resolution;
	}
}

geometry_msgs::Pose ActionExecutorDetectObjects::getCurrentLocation(
		const DurativeAction & a, SymbolicState & current) {
	geometry_msgs::Pose result;
	Predicate p;
	p.parameters.push_back(a.parameters[0]);
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
