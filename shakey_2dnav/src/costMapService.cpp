#include "ros/ros.h"
#include "nav_msgs/GetMap.h"

nav_msgs::OccupancyGrid cur_grid;
bool map_set = false;

void saveMap(const nav_msgs::OccupancyGrid& grid)
{
		cur_grid = grid;
		map_set = true;
}

bool getMap (nav_msgs::GetMap::Request &req,
		nav_msgs::GetMap::Response &res) {
	res.map = cur_grid;
	return map_set;
}

int main (int argc, char **argv) {
	ros::init(argc, argv, "costmap_service");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/move_base/global_costmap/costmap", 10, saveMap);
	ros::ServiceServer service = nh.advertiseService("global_costmap", getMap);
	ros::spin();
	return 0;
}
