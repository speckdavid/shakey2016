#ifndef OBJECT_DETECTION_VISUALISATION_H
#define OBJECT_DETECTION_VISUALISATION_H

#include <ros/ros.h>
#include <Eigen/Geometry>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class ObjectVisualisation {
	
protected:
	visualization_msgs::MarkerArray _markerArray;
	ros::Publisher _vis_pub;
	ros::NodeHandle _nh;
	int _num_marker;
	int _cur_objects;
	std::string _worldFrame;
	
	visualization_msgs::Marker dummyMarker(int id);
	
	geometry_msgs::Point getRosPoint(Eigen::Vector3f point);
	
public:
	ObjectVisualisation();
	
	void setNodeHandle(ros::NodeHandle nh);

	void setWorldFrame(std::string worldFrame);

	void resetMarker();

	void addBoxMarkerFromTopPlane(std::vector<Eigen::Vector3f> minBox, 
			Eigen::Matrix3f eigenVectors, float width_p, float length_p);
			
	void addWedgeMarkerFromCrookedPlane(std::vector<Eigen::Vector3f> minBox, 
			Eigen::Matrix3f eigenVectors, float width_p, float length_p);
	
	void publish();
	
};
#endif
