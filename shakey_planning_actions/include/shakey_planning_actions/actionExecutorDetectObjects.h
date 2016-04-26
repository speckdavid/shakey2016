#ifndef ACTION_EXECUTOR_DETECT_OBJECTS_H
#define ACTION_EXECUTOR_DETECT_OBJECTS_H

#include <actionlib/client/simple_action_client.h>
#include "continual_planning_executive/actionExecutorService.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <actionlib/client/simple_action_client.h>
#include <shakey_object_recognition/DetectObjects.h>
#include "shakey_object_recognition/ObjectVisualisation.h"
#include <nav_msgs/GetMap.h>
#include <nav_msgs/GetPlan.h>

namespace shakey_planning_actions {

class ActionExecutorDetectObjects: public ActionExecutorService<
		shakey_object_recognition::DetectObjects> {
public:
	virtual void initialize(const std::deque<std::string> & arguments);

	virtual bool fillGoal(
			shakey_object_recognition::DetectObjects::Request & goal,
			const DurativeAction & a, const SymbolicState & current);

	virtual void updateState(bool & success,
			shakey_object_recognition::DetectObjects::Response & response,
			const DurativeAction & a, SymbolicState & current);

protected:

	void visualize(shakey_object_recognition::PushableObject object,
			geometry_msgs::Pose push_pose);

	float getOccValue(Eigen::Vector3f pos);

	float getWallDistance(geometry_msgs::Point pos,
			shakey_object_recognition::PushableObject obj);

	geometry_msgs::Pose getCurrentLocation(const DurativeAction & a, SymbolicState & current);

	ObjectVisualisation _objVis;
	ros::NodeHandle _nh;
	ros::ServiceClient _map_client;
	ros::ServiceClient _plan_client;
	nav_msgs::GetMap _mapResponse;
	double _push_distance;
	bool _pushToWall;
};

}
;

#endif
