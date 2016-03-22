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
	_pushToWall = false;
	_onlyDetection = false;
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
		if (_push_distance < 0)
			_pushToWall = true;
		if (_push_distance == 0)
			_onlyDetection = true;
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
			vector<geometry_msgs::Pose> reachable_poses;
			for (int j = 0; j < object.push_poses.size(); j++) {
				if (planToPos(detection_position, object.push_poses.at(j)))
					reachable_poses.push_back(object.push_poses.at(j));
			}
			object.push_poses = reachable_poses;
			std::cerr << object.push_poses.size() << std::endl;
			if (object.push_poses.size() == 0)
				ROS_WARN("No path to any push pose. Set Object to pushed.");

			// Create name
			std::string obj_name;
			obj_name = (object.obj_type == Box) ? "box" : "wedge";
			std::ostringstream os;
			os << i;
			obj_name += os.str();

			geometry_msgs::Pose cur;
			if (a.name == "detect-objects") {
				current.setObjectFluent("belongs-to-search-location", obj_name,
						a.parameters[0]);
				std::vector<geometry_msgs::Pose> minMax;
				std::vector<float> distance;
				if (!getMinMaxWallPushPose(object, &minMax, &distance)) {
					ROS_ERROR("No push location for pushing to a wall found.");
					continue;
				}
				cur = minMax.at(0);
				if (_pushToWall)
					_push_distance = distance.at(0);
				else
					_push_distance = std::min((double) distance.at(0),
							_push_distance);
			}

			if (a.name == "detect-doorway-state") {
				current.setObjectFluent("belongs-to-doorway", obj_name,
						a.parameters[1]);

				bool found_push_loc = false;
				for (int k = 0; k < object.push_poses.size(); k++) {
					geometry_msgs::Pose cur1 = object.push_poses.at(k);
					float dot = cur1.orientation.x
							* detection_position.orientation.x
							+ cur1.orientation.y
									* detection_position.orientation.y
							+ cur1.orientation.z
									* detection_position.orientation.z
							+ cur1.orientation.w
									* detection_position.orientation.w;
					float ang = 2 * (acos(dot) / M_PI * 180);
					if (std::abs(ang - 90) < 10) {
						if (getWallDistance(cur1.position, object)
								< _push_distance)
							continue;
						cur = cur1;
						found_push_loc = true;
						break;
					}
				}
				// If no 90 degree push location is possible -> push max location
				if (!found_push_loc) {
					std::vector<geometry_msgs::Pose> minMax;
					std::vector<float> distance;
					if (!getMinMaxWallPushPose(object, &minMax, &distance)) {
						ROS_ERROR(
								"No push location for pushing to a wall found.");
						continue;
					}
					cur = minMax.at(1);
					_push_distance = std::min((double) distance.at(1),
							_push_distance);
				}
			}
			current.addObject(obj_name, "movable_object");
			bool object_pushed = _onlyDetection || _push_distance < 0.25
					|| object.push_poses.size() == 0;
			current.setBooleanPredicate("pushed", obj_name, object_pushed);
			std::string push_loc_name = obj_name + "_push_loc";
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
			current.setNumericalFluent("push-distance", obj_name,
					_push_distance);

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

			//Visualize the destination point
			if (!_onlyDetection)
				visualize(object, cur);

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
		geometry_msgs::Pose push_pose) {
	shakey_object_recognition::PushableObject new_object = object;
	new_object.push_poses.clear();
	Eigen::Vector3f direction = Eigen::Vector3f(object.mean.position.x,
			object.mean.position.y, 0)
			- Eigen::Vector3f(push_pose.position.x, push_pose.position.y, 0);
	float transX = direction.normalized().x() * _push_distance;
	float transY = direction.normalized().y() * _push_distance;
	new_object.mean.position.x += transX;
	new_object.mean.position.y += transY;
	new_object.corner_points = object.corner_points;
	for (int b = 0; b < new_object.corner_points.size(); b++) {
		new_object.corner_points.at(b).x += transX;
		new_object.corner_points.at(b).y += transY;
	}
	std_msgs::ColorRGBA color;
	color.b = color.a = 1;
	_objVis.addObjectMarker(new_object, color);

}

bool ActionExecutorDetectObjects::getMinMaxWallPushPose(
		shakey_object_recognition::PushableObject obj,
		std::vector<geometry_msgs::Pose> *best_pos,
		std::vector<float> *distance) {
	distance->push_back(std::numeric_limits<float>::infinity());
	distance->push_back(-std::numeric_limits<float>::infinity());
	best_pos->push_back(geometry_msgs::Pose());
	best_pos->push_back(geometry_msgs::Pose());
	for (int i = 0; i < obj.push_poses.size(); i++) {
		geometry_msgs::Pose pos = obj.push_poses.at(i);
		Eigen::Vector3f cur_pos = Eigen::Vector3f(pos.position.x,
				pos.position.y, 0);
		float dist = getWallDistance(pos.position, obj);
		if (distance->at(0) > dist) {
			distance->at(0) = dist;
			best_pos->at(0) = pos;
		}
		if (distance->at(1) < dist) {
			distance->at(1) = dist;
			best_pos->at(1) = pos;
		}
	}
	// minus max size and a small offset
	distance->at(0) -= (std::max(obj.length, obj.width)
			+ _mapResponse.response.map.info.resolution * 1.5);
	distance->at(1) -= (std::max(obj.length, obj.width)
			+ _mapResponse.response.map.info.resolution * 1.5);
	return distance->at(0) != std::numeric_limits<float>::infinity()
			&& distance->at(1) != -std::numeric_limits<float>::infinity();
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
	float distance = 0;
	while (true) {
		float occ_value = getOccValue(mean);
		if (occ_value == -1 || occ_value == 100)
			return distance;
		mean += direction * _mapResponse.response.map.info.resolution;
		distance += _mapResponse.response.map.info.resolution;
	}
}

// TODO: Finish it
bool ActionExecutorDetectObjects::planToPos(geometry_msgs::Pose start,
		geometry_msgs::Pose goal) {
	nav_msgs::GetPlan srv;
	srv.request.start.pose = start;
	srv.request.goal.pose = goal;
	srv.request.start.header.frame_id = srv.request.goal.header.frame_id =
			"map";
	srv.request.start.header.stamp = srv.request.goal.header.stamp =
			ros::Time::now();
	_plan_client.call(srv);
	std::cerr << srv.response.plan.poses.size() << std::endl;
	return srv.response.plan.poses.size() != 0;
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
