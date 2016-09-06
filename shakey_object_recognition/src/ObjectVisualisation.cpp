#include <ros/ros.h>
#include <shakey_object_recognition/ObjectVisualisation.h>
#include <shakey_object_recognition/PushableObject.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Geometry>

void ObjectVisualisation::resetMarker() {
	_cur_objects = 0;
	for (int i = 0; i < _num_marker; i++)
		_markerArray.markers[i] = dummyMarker(i);
}

void ObjectVisualisation::publish() {
	_vis_pub.publish(_markerArray);
}

void ObjectVisualisation::addObjectMarker(shakey_object_recognition::PushableObject obj, std_msgs::ColorRGBA color) {
	if (obj.obj_type == Box) addBoxMarker(obj, color);
	if (obj.obj_type == Wedge) addWedgeMarker(obj, color);
	addPushPosesMarker(obj, color);
}

void ObjectVisualisation::addBoxMarker(shakey_object_recognition::PushableObject obj, std_msgs::ColorRGBA color) {
	visualization_msgs::Marker marker = dummyMarker(_cur_objects);
	marker.header.frame_id = obj.frame_id;
	marker.type = visualization_msgs::Marker::CUBE;
	// Translation of points to the mean
	marker.pose = obj.mean;

	// Construct scale from max and min values (Bounding box)
	marker.scale.x = std::abs(obj.width);
	marker.scale.y = std::abs(obj.length);
	marker.scale.z = std::abs(obj.height);
	marker.color = color;
	marker.color.a = 1.0;
	_markerArray.markers[_cur_objects] = marker;
	_cur_objects++;
}

void ObjectVisualisation::addWedgeMarker(shakey_object_recognition::PushableObject obj, std_msgs::ColorRGBA color) {
	visualization_msgs::Marker marker = dummyMarker(_cur_objects);
	marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
	marker.header.frame_id = obj.frame_id;
	std::vector<geometry_msgs::Point> ros_p;
	for (int i = 0; i < 4; i++) {
		ros_p.push_back(obj.corner_points.at(i));
	}
	int upperPointsInd[2] = { -1, -1 };
	for (int i = 0; i < 4; i++) {
		if (obj.corner_points.at(i).z > 0.2) {
			upperPointsInd[1] = upperPointsInd[0];
			upperPointsInd[0] = i;
		}
	}
	geometry_msgs::Point p0 = obj.corner_points.at(upperPointsInd[0]);
	geometry_msgs::Point p1 = obj.corner_points.at(upperPointsInd[1]);
	p0.z = 0;
	p1.z = 0;
	ros_p.push_back(p0);
	ros_p.push_back(p1);
	for (int i = 0; i < 6; i++) {
		for (int j = i + 1; j < 6; j++) {
			for (int k = j + 1; k < 6; k++) {
				marker.points.push_back(ros_p[i]);
				marker.points.push_back(ros_p[j]);
				marker.points.push_back(ros_p[k]);
			}
		}
	}
	marker.color = color;
	marker.color.a = 1.0;
	_markerArray.markers[_cur_objects] = marker;
	_cur_objects++;
}

void ObjectVisualisation::addPushPosesMarker(shakey_object_recognition::PushableObject obj, std_msgs::ColorRGBA color) {
	for (int i = 0; i < obj.push_poses.size(); i++) {
		visualization_msgs::Marker marker = dummyMarker(_cur_objects);
		marker.type = visualization_msgs::Marker::ARROW;
		marker.header.frame_id = obj.frame_id;
		marker.pose = obj.push_poses.at(i);
		marker.scale.x = 0.3;
		marker.scale.y = 0.05;
		marker.scale.z = 0.05;
		marker.color = color;
		marker.color.a = 1.0;
		_markerArray.markers[_cur_objects] = marker;
		_cur_objects++;
	}
}

visualization_msgs::Marker ObjectVisualisation::dummyMarker(int id) {
	visualization_msgs::Marker marker;
	marker.header.frame_id = _worldFrame;
	marker.header.stamp = ros::Time();
	marker.ns = "obj/" + _name;
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
