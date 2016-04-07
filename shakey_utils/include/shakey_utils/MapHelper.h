#ifndef MAP_HELPER_H
#define MAP_HELPER_H

#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include <iostream>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

using namespace Eigen;

class MapHelper {

public:
	bool getPushDistances(MatrixXf A, VectorXf b, VectorXf * x);
	bool getPushDistances(geometry_msgs::Pose push1, geometry_msgs::Pose push2,
			geometry_msgs::Pose dest, VectorXf * x);
	float getOccValue(geometry_msgs::Point p);
	bool freeSpace(geometry_msgs::Point p);
	bool canPushDistance(geometry_msgs::Point mean, Vector3f direction,
			Vector3f distance, float obj_size);

	MapHelper() {
		ros::ServiceClient map_client = _nh.serviceClient<nav_msgs::GetMap>(
				"/global_costmap");
		if (!map_client.call(_map))
			ROS_ERROR("No global costmap found.");
	}
	;

private:
	ros::NodeHandle _nh;
	nav_msgs::GetMap _map;

};

#endif
