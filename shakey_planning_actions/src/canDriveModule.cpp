#include "shakey_planning_actions/canDriveModule.h"
#include <stdlib.h>
#include <sys/time.h>
using namespace std;

double canDriveToPos(const ParameterList & parameterList,
		predicateCallbackType predicateCallback,
		numericalFluentCallbackType numericalFluentCallback, int relaxed) {
	if (!latestMap) {
		ros::ServiceClient map_client = _nh.serviceClient<nav_msgs::GetMap>(
				"/static_map");
		if (!map_client.call(_map)) {
			ROS_ERROR("No static map found.");
			return INFINITE_COST;
		}
		latestMap = true;
	}

	NumericalFluentList* nlf = new NumericalFluentList();
	nlf->push_back(NumericalFluent("x", parameterList));
	nlf->push_back(NumericalFluent("y", parameterList));
	nlf->push_back(NumericalFluent("z", parameterList));
	nlf->push_back(NumericalFluent("qx", parameterList));
	nlf->push_back(NumericalFluent("qy", parameterList));
	nlf->push_back(NumericalFluent("qz", parameterList));
	nlf->push_back(NumericalFluent("qw", parameterList));
	numericalFluentCallback(nlf);
	/*for (int i = 0; i < nlf->size(); i++) {
		std::cerr << nlf->at(i) << std::endl;
	}*/
	geometry_msgs::Pose dest;
	dest.position.x = nlf->at(0).value;
	dest.position.y = nlf->at(1).value;
	dest.position.z = nlf->at(2).value;
	dest.orientation.x = nlf->at(3).value;
	dest.orientation.y = nlf->at(4).value;
	dest.orientation.z = nlf->at(5).value;
	dest.orientation.w = nlf->at(6).value;
	// Footprint 0.65x0.65 of PR2
	if (!freeSpace(_map, dest, 0.65, 0.65)) {
		ROS_ERROR("Not possible to drive to position -> [%f, %f, %f]",
				dest.position.x, dest.position.y, dest.position.z);
		return INFINITE_COST;
	}
	return 0;
}

// Additional Functions
float getOccValue(nav_msgs::GetMap map, Eigen::Vector3f pos) {
	nav_msgs::OccupancyGrid occGrid = map.response.map;
	unsigned int grid_x = (unsigned int) ((pos.x()
			- occGrid.info.origin.position.x) / occGrid.info.resolution);
	unsigned int grid_y = (unsigned int) ((pos.y()
			- occGrid.info.origin.position.y) / occGrid.info.resolution);
	unsigned int cell = grid_y * occGrid.info.width + grid_x;
	if (grid_x >= occGrid.info.width || grid_y >= occGrid.info.height) {
		std::ostringstream os;
		os << pos.x() << ", " << pos.y() << ", " << pos.z();
		ROS_WARN("Position [%s] is not in static map.", os.str().c_str());
		return -1;
	}
	return occGrid.data[cell];
}

bool freeSpace(nav_msgs::GetMap map, geometry_msgs::Pose pose, float width,
		float depth) {
	nav_msgs::OccupancyGrid occGrid = map.response.map;
	// starting at left top point
	pose.position.x -= width / 2;
	pose.position.y -= depth / 2;
	// Create tranformation
	tf::Transform tranform;
	tranform.setOrigin(
			tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
	tranform.setRotation(
			tf::Quaternion(pose.orientation.x, pose.orientation.y,
					pose.orientation.z, pose.orientation.w));
	// Check every occ cell
	for (int i = 0; i < depth / occGrid.info.resolution; i++) {
		for (int j = 0; j < width / occGrid.info.resolution; j++) {
			tf::Vector3 cur_point(i * occGrid.info.resolution,
					j * occGrid.info.resolution, 0);
			cur_point = tranform.operator*(cur_point);
			std::cerr << cur_point.x() << ", " << cur_point.y() << ", "
					<< cur_point.z() << std::endl;
			float occ_value = getOccValue(map,
					Eigen::Vector3f(cur_point.x(), cur_point.y(),
							cur_point.z()));
			if (occ_value == -1 || occ_value == 100)
				return false;
		}
	}
	return true;
}

