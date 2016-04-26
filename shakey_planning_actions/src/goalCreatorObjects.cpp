#include "shakey_planning_actions/goalCreatorObjects.h"
#include "shakey_utils/geometryPoses.h"
#include "shakey_utils/stringutil.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <set>
#include <std_srvs/Empty.h>

PLUGINLIB_DECLARE_CLASS(shakey_planning_actions, goal_creator_objects,
		shakey_planning_actions::GoalCreatorObjects,
		continual_planning_executive::GoalCreator)

namespace shakey_planning_actions {

GoalCreatorObjects::GoalCreatorObjects() {
	_objViz.initialise("Objects_Location", "map");
	_objViz.publish();
}

GoalCreatorObjects::~GoalCreatorObjects() {
}

bool GoalCreatorObjects::fillStateAndGoal(SymbolicState & currentState,
		SymbolicState & goal) {
	currentState.addSuperType("pose", "pose");
	currentState.addSuperType("frameid", "frameid");
	currentState.addSuperType("location", "pose");
	currentState.addSuperType("search_location", "location");
	currentState.addSuperType("pushable_location", "location");
	currentState.addSuperType("object_location", "location");
	currentState.addSuperType("doorway_location", "location");
	currentState.addSuperType("doorway_in_location", "doorway_location");
	currentState.addSuperType("doorway_out_location", "doorway_location");
	currentState.addSuperType("room", "room");
	currentState.addSuperType("doorway", "doorway");
	currentState.addSuperType("movable_object", "pose");
	goal.addSuperType("pose", "pose");
	goal.addSuperType("frameid", "frameid");
	goal.addSuperType("location", "pose");
	goal.addSuperType("search_location", "location");
	goal.addSuperType("pushable_location", "location");
	goal.addSuperType("object_location", "location");
	goal.addSuperType("doorway_location", "location");
	goal.addSuperType("doorway_in_location", "doorway_location");
	goal.addSuperType("doorway_out_location", "doorway_location");
	goal.addSuperType("room", "room");
	goal.addSuperType("doorway", "doorway");
	goal.addSuperType("movable_object", "pose");

	// Dummy initial location to set the room initial correct room assignment
	std::string loc = "init_loc";
	currentState.addObject(loc, "location");
	goal.addObject(loc, "search_location");
	currentState.setNumericalFluent("x", loc, 0);
	currentState.setNumericalFluent("y", loc, 0);
	currentState.setNumericalFluent("z", loc, 0);
	currentState.setNumericalFluent("qx", loc, 0);
	currentState.setNumericalFluent("qy", loc, 0);
	currentState.setNumericalFluent("qz", loc, 0);
	currentState.setNumericalFluent("qw", loc, 1);
	currentState.setNumericalFluent("timestamp", loc, ros::Time::now().toSec());
	currentState.setObjectFluent("frame-id", loc, "/map");
	currentState.setObjectFluent("location-in-room", loc, "room0");

	ros::NodeHandle nhPriv("~");
	// load object_locations
	std::string locationsFile;
	// load grasp_locations
	if (!nhPriv.getParam("locations", locationsFile)) {
		ROS_ERROR("Could not get ~locations parameter.");
		return false;
	}

	GeometryPoses locations = GeometryPoses();
	if (!locations.load(locationsFile)) {
		ROS_ERROR("Could not load locations from \"%s\".",
				locationsFile.c_str());
		return false;
	}
	std::set<string> rooms;
	std::set<string> doorways;
	forEach(const GeometryPoses::NamedPose & np, locations.getPoses()){
	const string& location = np.first;
	currentState.setNumericalFluent("timestamp", location, np.second.header.stamp.toSec());
	currentState.addObject(np.second.header.frame_id, "frameid");
	currentState.setObjectFluent("frame-id", location, np.second.header.frame_id);
	currentState.setNumericalFluent("x", location, np.second.pose.position.x);
	currentState.setNumericalFluent("y", location, np.second.pose.position.y);
	currentState.setNumericalFluent("z", location, np.second.pose.position.z);
	currentState.setNumericalFluent("qx", location, np.second.pose.orientation.x);
	currentState.setNumericalFluent("qy", location, np.second.pose.orientation.y);
	currentState.setNumericalFluent("qz", location, np.second.pose.orientation.z);
	currentState.setNumericalFluent("qw", location, np.second.pose.orientation.w);

	// additional fluents
	// location name scheme: <type>typeName_AdditionalName_<room>roomName
	const vector<string>& nameParts = StringUtil::split(location, "_");
	const string& room = nameParts[nameParts.size()-1];
	const string& type = nameParts[0];
	string name = nameParts[0];
	rooms.insert(room);
	currentState.addObject(room, "room");
	currentState.setObjectFluent("location-in-room", location, room);

	if (StringUtil::startsWith(type, "doorway"))
	{
		doorways.insert(name);
		currentState.addObject(name, "doorway");

		// door_in_location
		currentState.setObjectFluent("belongs-to-doorway", location, name);
		currentState.addObject(location, "doorway_in_location");
		goal.addObject(location, "doorway_in_location");

		// also create the matching door_out_location as rotZ by 180 deg
		geometry_msgs::PoseStamped outPose = np.second;// copy everything, only switch orientation
		tf::Quaternion rot180 = tf::createQuaternionFromYaw(M_PI);
		tf::Quaternion locRot;
		tf::quaternionMsgToTF(outPose.pose.orientation, locRot);
		tf::quaternionTFToMsg(locRot * rot180, outPose.pose.orientation);

		string outLocation = location + "_out";
		currentState.setObjectFluent("belongs-to-doorway", outLocation, name);
		currentState.addObject(outLocation, "doorway_out_location");
		goal.addObject(outLocation, "doorway_out_location");

		currentState.setNumericalFluent("timestamp", outLocation, outPose.header.stamp.toSec());
		currentState.setObjectFluent("frame-id", outLocation, outPose.header.frame_id);
		currentState.setNumericalFluent("x", outLocation, outPose.pose.position.x);
		currentState.setNumericalFluent("y", outLocation, outPose.pose.position.y);
		currentState.setNumericalFluent("z", outLocation, outPose.pose.position.z);
		currentState.setNumericalFluent("qx", outLocation, outPose.pose.orientation.x);
		currentState.setNumericalFluent("qy", outLocation, outPose.pose.orientation.y);
		currentState.setNumericalFluent("qz", outLocation, outPose.pose.orientation.z);
		currentState.setNumericalFluent("qw", outLocation, outPose.pose.orientation.w);
		currentState.setObjectFluent("location-in-room", outLocation, room);
	}
	else if (StringUtil::startsWith(type, "object")) {
		currentState.addObject(location, "object_location");
		goal.addObject(location, "object_location");
		// Visualize the pose
		shakey_object_recognition::PushableObject obj;
		obj.obj_type = Box;
		obj.frame_id = np.second.header.frame_id;
		obj.mean.position.x = np.second.pose.position.x;
		obj.mean.position.y = np.second.pose.position.y;
		obj.mean.position.z = np.second.pose.position.z;
		obj.mean.orientation.x = np.second.pose.orientation.x;
		obj.mean.orientation.y = np.second.pose.orientation.y;
		obj.mean.orientation.z = np.second.pose.orientation.z;
		obj.mean.orientation.w = np.second.pose.orientation.w;
		obj.height = 0.01;
		obj.length = obj.width = 0.25;
		std_msgs::ColorRGBA color;
		color.g = color.r = 1;
		_objViz.addObjectMarker(obj, color);
		_objViz.publish();
	}
	else
	{
		currentState.addObject(location, "search_location");
		goal.addObject(location, "search_location");
	}
}

	goal.setForEachGoalStatement("search_location", "searched", true);
	goal.setForEachGoalStatement("movable_object", "pushed", true);
	goal.setForEachGoalStatement("object_location", "object-at-location", true);
	return true;
}

}
;
