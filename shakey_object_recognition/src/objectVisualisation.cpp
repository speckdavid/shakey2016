#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Geometry>
#include "shakey_object_recognition/objectVisualisation.h"

ObjectVisualisation::ObjectVisualisation() {
	_vis_pub = _nh.advertise<visualization_msgs::MarkerArray>(
			"visualization_markers", 0);
	_num_marker = 5;
	_markerArray.markers.resize(5);
	_cur_objects = 0;
	resetMarker();
}

void ObjectVisualisation::setNodeHandle(ros::NodeHandle nh) {
	_nh = nh;
}

void ObjectVisualisation::setWorldFrame(std::string worldFrame) {
	_worldFrame = worldFrame;
}

void ObjectVisualisation::resetMarker() {
	_cur_objects = 0;
	for (int i = 0; i < _num_marker; i++)
		_markerArray.markers[i] = dummyMarker(i);
}

void ObjectVisualisation::publish() {
	_vis_pub.publish(_markerArray);
}

void ObjectVisualisation::addBoxMarkerFromTopPlane(
		std::vector<Eigen::Vector3f> minBox, Eigen::Matrix3f eigenVectors,
		float width_p, float length_p) {
	visualization_msgs::Marker marker = dummyMarker(_cur_objects);
	marker.type = visualization_msgs::Marker::CUBE;
	// Translation of points to the mean
	marker.pose.position.x = minBox[4](0);
	marker.pose.position.y = minBox[4](1);
	marker.pose.position.z = minBox[4](2) / 2;
	// Rotation given by EigenVectors as Quaternion
	Eigen::Matrix3d m = eigenVectors.cast<double>();
	Eigen::Quaterniond q(m);
	q.normalize();
	geometry_msgs::Quaternion plane_orientation;
	tf::Quaternion qt;
	tf::quaternionEigenToTF(q, qt);
	tf::quaternionTFToMsg(qt, plane_orientation);
	marker.pose.orientation = plane_orientation;
	// Construct scale from max and min values (Bounding box)
	marker.scale.x = std::abs(width_p);
	marker.scale.y = std::abs(length_p);
	std::cerr << width_p << ", " << length_p << std::endl;
	marker.scale.z = minBox[0](2);
	marker.color.a = 1.0;
	_markerArray.markers[_cur_objects] = marker;
	_cur_objects++;
}

void ObjectVisualisation::addWedgeMarkerFromCrookedPlane(
		std::vector<Eigen::Vector3f> minBox, Eigen::Matrix3f eigenVectors,
		float width_p, float length_p) {
	visualization_msgs::Marker marker = dummyMarker(_cur_objects);
	marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
	marker.color.a = 1.0;
	std::vector<geometry_msgs::Point> ros_p;
	for (int i = 0; i < 4; i++) {
		ros_p.push_back(getRosPoint(minBox[i]));
	}
	int upperPointsInd[2] = { -1, -1 };
	for (int i = 0; i < 4; i++) {
		if (minBox[i](2) > 0.2) {
			upperPointsInd[1] = upperPointsInd[0];
			upperPointsInd[0] = i;
		}
	}
	geometry_msgs::Point p0 = getRosPoint(minBox[upperPointsInd[0]]);
	geometry_msgs::Point p1 = getRosPoint(minBox[upperPointsInd[1]]);
	p0.z = 0;
	p1.z = 0;
	ros_p.push_back(p0);
	ros_p.push_back(p1);
	// TODO: connect correct points and find all bottoms points in partial view
	for (int i = 0; i < 6; i++) {
		for (int j = i + 1; j < 6; j++) {
			for (int k = j + 1; k < 6; k++) {
				marker.points.push_back(ros_p[i]);
				marker.points.push_back(ros_p[j]);
				marker.points.push_back(ros_p[k]);
			}
		}
	}
	_markerArray.markers[_cur_objects] = marker;
	_cur_objects++;
}

visualization_msgs::Marker ObjectVisualisation::dummyMarker(int id) {
	visualization_msgs::Marker marker;
	marker.header.frame_id = _worldFrame;
	marker.header.stamp = ros::Time();
	marker.ns = "segmentation3d";
	marker.id = id;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 1;
	marker.color.a = 0.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	return marker;
}

geometry_msgs::Point ObjectVisualisation::getRosPoint(Eigen::Vector3f point)
  {
	  geometry_msgs::Point p;
	  p.x = point(0);
	  p.y = point(1);
	  p.z = point(2);
	  return p;
  }
