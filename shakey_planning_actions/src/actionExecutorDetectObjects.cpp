#include "shakey_planning_actions/actionExecutorDetectObjects.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <sstream>

PLUGINLIB_DECLARE_CLASS(shakey_planning_actions, action_executor_detect_objects,
        shakey_planning_actions::ActionExecutorDetectObjects,
        continual_planning_executive::ActionExecutorInterface)

namespace shakey_planning_actions
{
	void ActionExecutorDetectObjects::initialize(const std::deque<std::string> & arguments)
	    {
			// Create Template for the service
			ActionExecutorService<shakey_object_recognition::DetectObjects>::initialize(arguments);
	    }

	bool ActionExecutorDetectObjects::fillGoal(shakey_object_recognition::DetectObjects::Request & goal,
	            const DurativeAction & a, const SymbolicState & current)
	    {
			// fill the srv message
    		// FIXME: Wait some seconds to buffer cloud
    		ros::Duration(1.0).sleep();
	        return true;
	    }

	void ActionExecutorDetectObjects::updateState(bool& success,
			shakey_object_recognition::DetectObjects::Response & response,
	            const DurativeAction & a, SymbolicState & current)
	    {
			if (!success) return;
			if (response.obj_type <= 1) {
				std::string obj_name;
				obj_name = (response.obj_type == 0) ? "box_" : "wedge_";
				if (a.name == "detect-objects") {
					obj_name += a.parameters[0];
					current.setObjectFluent("belongs-to-search-location", obj_name, a.parameters[0]);
				}
				if (a.name == "detect-doorway-state") {
					obj_name = "box_" + a.parameters[1];
					current.setObjectFluent("belongs-to-doorway", obj_name, a.parameters[1]);
				}
				current.addObject(obj_name, "movable_object");
				current.setBooleanPredicate("pushed", obj_name, false);
				for (int i = 0; i < response.push_locations.size(); i++) {
					geometry_msgs::Pose cur = response.push_locations.at(i);
					// FIXME: Don't take a location directly at the doorway in -> only take some which
					// push box away
					if (a.name == "detect-doorway-state") {
						Predicate p;
						p.parameters.push_back(a.parameters[0]);
						double valueX, valueY;
						p.name = "x";
						current.hasNumericalFluent(p, &valueX);
						p.name = "y";
						current.hasNumericalFluent(p, &valueY);
						if (hypot(valueX - cur.position.x, valueY - cur.position.y) < 1.5)
							continue;
					}
					std::ostringstream os;
					os << i;
					std::string push_loc_name = obj_name + "_push_loc_" + os.str();
					current.addObject(push_loc_name, "pushable_location");
					current.setObjectFluent("frame-id", push_loc_name, "/map");
					current.setNumericalFluent("x", push_loc_name, cur.position.x);
					current.setNumericalFluent("y", push_loc_name, cur.position.y);
					current.setNumericalFluent("z", push_loc_name, cur.position.z);
					current.setNumericalFluent("qx", push_loc_name, cur.orientation.x);
					current.setNumericalFluent("qy", push_loc_name, cur.orientation.y);
					current.setNumericalFluent("qz", push_loc_name, cur.orientation.z);
					current.setNumericalFluent("qw", push_loc_name, cur.orientation.w);
					current.setNumericalFluent("timestamp", push_loc_name, ros::Time::now().toSec());
					current.setObjectFluent("belongs-to-movable-object", push_loc_name, obj_name);

					// Determine Room
					// get all locations and search for the nearest that has location-in-room set
					pair<SymbolicState::TypedObjectConstIterator, SymbolicState::TypedObjectConstIterator> locations =
					current.getTypedObjects().equal_range("location");
					double minDist = HUGE_VAL;
					std::string locRoom = "";
					Predicate p;
					p.name = "location-in-room";
					p.parameters.push_back(push_loc_name);
					for(SymbolicState::TypedObjectConstIterator it = locations.first; it != locations.second; it++) {
						string location = it->second;
						p.parameters[0] = location;
						// first check if location-in-room is set
						p.name = "location-in-room";
						string room;
						if(!current.hasObjectFluent(p, &room)) {
							continue;
						}
						p.name = "x";
						double valueX;
						if(!current.hasNumericalFluent(p, &valueX)) {
							ROS_ERROR("%s: location: %s - no x-location.", __func__, location.c_str());
							continue;
						}
						double valueY;
						p.name = "y";
						if(!current.hasNumericalFluent(p, &valueY)) {
							ROS_ERROR("%s: location: %s - no y-location.", __func__, location.c_str());
							continue;
						}
						double dist = hypot(valueX - cur.position.x, valueY - cur.position.y);
						if(dist < minDist) {
							minDist = dist;
							locRoom = room;
						}
					}
					// search done, now enter robotRoom if found
					if(locRoom.empty()) {
						ROS_WARN("Could not infer location-in-room for robot_location as no nearest location with location-in-room set was found.");
					} else {
						ROS_INFO("Determined (location-in-room %s) = %s", push_loc_name.c_str(), locRoom.c_str());
						current.setObjectFluent("location-in-room", push_loc_name, locRoom);
					}
				}
			}
			if (a.name == "detect-objects")
				current.setBooleanPredicate("searched", a.parameters[0], true);
			if (a.name == "detect-doorway-state")
				current.setBooleanPredicate("doorway-state-known", a.parameters[1], true);
			ROS_INFO("DetectObjects returned result");
	    }

}
