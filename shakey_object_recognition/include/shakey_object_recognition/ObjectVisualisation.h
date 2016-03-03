#ifndef OBJECT_DETECTION_VISUALISATION_H
#define OBJECT_DETECTION_VISUALISATION_H

#include <ros/ros.h>
#include <Eigen/Geometry>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <shakey_object_recognition/PushableObject.h>

enum Object_class {Box, Wedge, Ground, Unkown};

class ObjectVisualisation {
	
protected:
	visualization_msgs::MarkerArray _markerArray;
	ros::Publisher _vis_pub;
	ros::NodeHandle _nh;
	int _num_marker;
	int _cur_objects;
	std::string _worldFrame;
	std::string _name;

	visualization_msgs::Marker dummyMarker(int id);
	
	geometry_msgs::Point getRosPoint(Eigen::Vector3f point);
	
	void addBoxMarker(shakey_object_recognition::PushableObject, std_msgs::ColorRGBA color);

	void addWedgeMarker(shakey_object_recognition::PushableObject, std_msgs::ColorRGBA color);

	void addPushPosesMarker(shakey_object_recognition::PushableObject, std_msgs::ColorRGBA color);

public:
	ObjectVisualisation(){};
	
	void initialise(ros::NodeHandle nh, std::string name, std::string worldFrame) {
		_nh = nh;
		_name = name;
		_worldFrame = worldFrame;
		_vis_pub = _nh.advertise<visualization_msgs::MarkerArray>(
				_name, 0);
		_num_marker = 25;
		_markerArray.markers.resize(_num_marker);
		_cur_objects = 0;
		resetMarker();
	}

	void resetMarker();
	
	void addObjectMarker(shakey_object_recognition::PushableObject, std_msgs::ColorRGBA color);

	void publish();
	
};
#endif
