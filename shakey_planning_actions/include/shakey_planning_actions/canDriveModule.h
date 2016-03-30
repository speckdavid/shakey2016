#ifndef CAN_DRIVE_MODULE_H
#define CAN_DRIVE_MODULE_H

#include "tfd_modules/module_api/pddlModuleTypes.h"
#include <ros/ros.h>
#include "pcl/common/eigen.h"
#include <nav_msgs/GetMap.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

using namespace modules;

#ifdef __cplusplus
extern "C" {
#endif

ros::NodeHandle _nh;
nav_msgs::GetMap _map;
tf::TransformListener _listener;
bool latestMap = false;

/// Checks if robot can drive to a certain location in 2 actions
double canDriveToPos(const ParameterList & parameterList, predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback, int relaxed);

VERIFY_CONDITIONCHECKER_DEF(canDriveToPos);

// Additional Functions for modules
float getOccValue(nav_msgs::GetMap map, Eigen::Vector3f pos);
bool freeSpace(nav_msgs::GetMap map, geometry_msgs::Pose pose, float width, float depth);

#ifdef __cplusplus
}
#endif

#endif

