#include "shakey_planning_actions/canDriveModule.h"
#include <stdlib.h>
#include <sys/time.h>
using namespace std;
using namespace Eigen;

double canDriveToPos(const ParameterList & parameterList,
		predicateCallbackType predicateCallback,
		numericalFluentCallbackType numericalFluentCallback, int relaxed) {
	geometry_msgs::Point dest =
			getPose(parameterList, numericalFluentCallback).position;
	if (!_mapHelper.freeSpace(dest)) {
		ROS_WARN("Not possible to drive to position -> [%f, %f, %f]", dest.x,
				dest.y, dest.z);
		return INFINITE_COST;
	}
	return 0;
}

double canPushDistance(const ParameterList & parameterList,
		predicateCallbackType predicateCallback,
		numericalFluentCallbackType numericalFluentCallback, int relaxed) {
	return 0;
}

double costDriveToPos(const ParameterList & parameterList,
		predicateCallbackType predicateCallback,
		numericalFluentCallbackType numericalFluentCallback, int relaxed) {
	NumericalFluentList* nlf = new NumericalFluentList();
	ParameterList* pl = new ParameterList();
	pl->push_back(parameterList.at(0));
	geometry_msgs::Pose pos1 = getPose(*pl, numericalFluentCallback);
	pl->at(0) = parameterList.at(1);
	geometry_msgs::Pose pos2 = getPose(*pl, numericalFluentCallback);
	tf::Point p, g;
	tf::pointMsgToTF(pos1.position, p);
	tf::pointMsgToTF(pos2.position, g);
	g = g - p;
	return std::abs(g.x()) + std::abs(g.y());
}

double costPushDistance(const ParameterList & parameterList,
		predicateCallbackType predicateCallback,
		numericalFluentCallbackType numericalFluentCallback, int relaxed) {
	NumericalFluentList* nlf = new NumericalFluentList();
	nlf->push_back(NumericalFluent("push-cost", parameterList));
	numericalFluentCallback(nlf);
	return nlf->at(0).value;
}

double canPushToPos(const ParameterList & parameterList,
		predicateCallbackType predicateCallback,
		numericalFluentCallbackType numericalFluentCallback, int relaxed) {
	ParameterList* pl = new ParameterList();
	pl->push_back(parameterList.at(0));
	geometry_msgs::Pose push1 = getPose(*pl, numericalFluentCallback);
	pl->at(0) = parameterList.at(1);
	geometry_msgs::Pose push2 = getPose(*pl, numericalFluentCallback);
	pl->at(0) = parameterList.at(2);
	geometry_msgs::Pose dest = getPose(*pl, numericalFluentCallback);
	MapHelper mh;
	VectorXf s;
	double result =
			mh.getPushDistances(push1, push2, dest, &s) ? 0 : INFINITE_COST;
	if (result == INFINITE_COST)
		return result;
	// Check if max push distance (limited by  wall) is smaller then the mandatory push distance
	NumericalFluentList* nlf = new NumericalFluentList();
	pl->at(0) = parameterList.at(0);
	nlf->push_back(NumericalFluent("push-distance", *pl));
	numericalFluentCallback(nlf);
	float dist1 = nlf->at(0).value;
	pl->at(0) = parameterList.at(1);
	nlf->push_back(NumericalFluent("push-distance", *pl));
	numericalFluentCallback(nlf);
	float dist2 = nlf->at(0).value;
	if (dist1 < s.x() || dist2 < s.y()) {
		result = INFINITE_COST;
		ROS_WARN(
				"Max push-distance smaller then push-distance to pos: %f <? %f , %f <? %f",
				dist1, s.x(), dist2, s.y());
	}
	return result;
}

// Additional functions
geometry_msgs::Pose getPose(const ParameterList & pl,
		numericalFluentCallbackType numericalFluentCallback) {
	NumericalFluentList* nlf = new NumericalFluentList();
	nlf->push_back(NumericalFluent("x", pl));
	nlf->push_back(NumericalFluent("y", pl));
	nlf->push_back(NumericalFluent("z", pl));
	nlf->push_back(NumericalFluent("qx", pl));
	nlf->push_back(NumericalFluent("qy", pl));
	nlf->push_back(NumericalFluent("qz", pl));
	nlf->push_back(NumericalFluent("qw", pl));
	numericalFluentCallback(nlf);
	geometry_msgs::Pose result;
	result.position.x = nlf->at(0).value;
	result.position.y = nlf->at(1).value;
	result.position.z = nlf->at(2).value;
	result.orientation.x = nlf->at(3).value;
	result.orientation.y = nlf->at(4).value;
	result.orientation.z = nlf->at(5).value;
	result.orientation.w = nlf->at(6).value;
	return result;
}

