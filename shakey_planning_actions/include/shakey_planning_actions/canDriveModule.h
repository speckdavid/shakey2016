#ifndef CAN_DRIVE_MODULE_H
#define CAN_DRIVE_MODULE_H

#include "tfd_modules/module_api/pddlModuleTypes.h"
#include <ros/ros.h>
#include "pcl/common/eigen.h"
#include <nav_msgs/GetMap.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <shakey_utils/MapHelper.h>

using namespace modules;

#ifdef __cplusplus
extern "C" {
#endif

MapHelper _mapHelper;
bool _setMapHelper = false;

/// Checks if robot can drive to a certain location in 2 actions
double canDriveToPos(const ParameterList & parameterList,
		predicateCallbackType predicateCallback,
		numericalFluentCallbackType numericalFluentCallback, int relaxed);

double canPushDistance(const ParameterList & parameterList,
		predicateCallbackType predicateCallback,
		numericalFluentCallbackType numericalFluentCallback, int relaxed);

double canPushToPos(const ParameterList & parameterList,
		predicateCallbackType predicateCallback,
		numericalFluentCallbackType numericalFluentCallback, int relaxed);

double costDriveToPos(const ParameterList & parameterList,
		predicateCallbackType predicateCallback,
		numericalFluentCallbackType numericalFluentCallback, int relaxed);

double costPushDistance(const ParameterList & parameterList,
		predicateCallbackType predicateCallback,
		numericalFluentCallbackType numericalFluentCallback, int relaxed);

VERIFY_CONDITIONCHECKER_DEF(canDriveToPos);
VERIFY_CONDITIONCHECKER_DEF(canPushDistance);
VERIFY_CONDITIONCHECKER_DEF(canPushToPos);

// Additional Functions
geometry_msgs::Pose getPose(const ParameterList & pl, numericalFluentCallbackType numericalFluentCallback);

#ifdef __cplusplus
}
#endif

#endif

