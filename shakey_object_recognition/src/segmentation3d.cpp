#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <ros/callback_queue.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>
// PCL specific includes
#include "pcl/common/eigen.h"
#include "pcl/common/angles.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>
// Local files
#include "./classifier.h"


class Segmentation3d
{
  ros::NodeHandle nh;
  ros::Publisher vis_pub;
  ros::Subscriber sub;
  tf::TransformListener listener;
  Classifier cl;

public:  
  Segmentation3d()
  {
    // Create a ROS subscriber for the input point cloud
    sub = nh.subscribe ("/head_mount_kinect/depth/points", 1, 
      &Segmentation3d::cloud_cb, this);
    
    // Create a ROS publisher for the output point cloud
    vis_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_markers", 0);
  }  

  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
  {
	// Create the output cloud
	pcl::PointCloud<pcl::PointXYZ> cloud_out_ob = transformed_cloud(input);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out = cloud_out_ob.makeShared();
    
    // Create some cloud for filtering
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>); 
    
    // Create visualization marker array.
    visualization_msgs::MarkerArray marker_array_msg;
    int number_markers = 10;
    marker_array_msg.markers.resize(number_markers);

    // Create plane representation
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    std::vector<pcl::ModelCoefficients> coeffs;
    
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);
    
    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    int i = 0, nr_points = (int) cloud_out->points.size ();
    // While 15% of the original cloud is still there
    while (cloud_out->points.size () > 0.15 * nr_points)
    {
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud (cloud_out);
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0)
      {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        // Replace all markers which are not needed
        while (i < number_markers) {
		  	marker_array_msg.markers[i] = dummy_marker(i);
		  	i++;
		}
        break;
      }
      coeffs.push_back(*coefficients);

      // Add plane to marker array
      marker_array_msg.markers[i] = marker(cloud_out, i, inliers, coefficients);
      
      // Extract the inliers
      extract.setInputCloud (cloud_out);
      extract.setIndices (inliers);
      extract.setNegative (false);
      extract.filter (*cloud_p);

      // Create the filtering object
      extract.setNegative (true);
      extract.filter (*cloud_f);
      cloud_out.swap (cloud_f);
      
      // Dumb to console
      dump_console(inliers, coefficients, i);
	  // Convert to ROS data type
      sensor_msgs::PointCloud2 output;
      pcl::toROSMsg (*cloud_f, output);
      i++;
    }
    Object_class obj = cl.classify(coeffs);
    if (obj == Box) std::cerr << "BOX" << std::endl;
    if (obj == Wedge) std::cerr << "WEDGE" << std::endl;
    if (obj == Ground) std::cerr << "GROUND" << std::endl;
    std::cerr << "--------------------------------------------------------------" << std::endl;
    vis_pub.publish(marker_array_msg);
  }
  
private:
  pcl::PointCloud<pcl::PointXYZ> transformed_cloud(const sensor_msgs::PointCloud2ConstPtr& input)  
  {
	pcl::PointCloud<pcl::PointXYZ> cloud_in;
	pcl::PointCloud<pcl::PointXYZ> cloud_out;
    pcl::fromROSMsg (*input, cloud_in);
	tf::StampedTransform transform;
    try
    {
	  listener.waitForTransform("odom_combined", input->header.frame_id, 
	    input->header.stamp , ros::Duration(3.0));
      listener.lookupTransform("odom_combined", input->header.frame_id, 
        input->header.stamp, transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
    }
    pcl_ros::transformPointCloud(cloud_in, cloud_out, transform);
    cloud_out.header.frame_id = "odom_combined";
    return cloud_out;
  }
  
  void dump_console(pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients, int planeNumber)
  {
    // Print data
    std::cerr << planeNumber << ". Model coefficients: " 
                             << coefficients->values[0] << " " 
                             << coefficients->values[1] << " "
                             << coefficients->values[2] << " " 
                             << coefficients->values[3] << std::endl;
    std::cerr << "with " << inliers->indices.size () << " Points\n" << std::endl;     
  }
  
  pcl::PointXYZ middle_point(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointIndices::Ptr inliers)
  {
	  pcl::PointXYZ middlePoint;
	  for(int i = 0; i <= inliers->indices.size(); i++)
	  { 
		int indice = inliers->indices[i];
		if (std::isnan(cloud_in->points[indice].x)) continue;
		middlePoint.x += (double) cloud_in->points[indice].x;
		middlePoint.y += (double) cloud_in->points[indice].y;
		middlePoint.z += (double) cloud_in->points[indice].z;
	  } 
	  middlePoint.x /= inliers->indices.size();
	  middlePoint.y /= inliers->indices.size();
	  middlePoint.z /= inliers->indices.size();
	  return middlePoint;
  }
  
    visualization_msgs::Marker marker(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, int index,  
      pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients)
    {
	  visualization_msgs::Marker marker;
      pcl::PointXYZ middlePoint = middle_point(cloud_in, inliers);
      marker.header.frame_id = "odom_combined";
      marker.header.stamp = ros::Time();
      marker.ns = "segmentation3d";
      marker.id = index;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = middlePoint.x;
      marker.pose.position.y = middlePoint.y;
      marker.pose.position.z = middlePoint.z;      
      marker.scale.x = 0.01;
      marker.scale.y = 1;
      marker.scale.z = 1;
      marker.color.a = 1;
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      
      // Set correct orientation
      tf::Quaternion qt = tf::shortestArcQuat (tf::Vector3(1,0,0), 
        tf::Vector3(coefficients->values[0], coefficients->values[1], coefficients->values[2]));
      geometry_msgs::Quaternion plane_orientation;
      tf::quaternionTFToMsg(qt, plane_orientation);
      marker.pose.orientation = plane_orientation;
      
      return marker;
  }
  visualization_msgs::Marker dummy_marker(int index)
  {
    visualization_msgs::Marker dummy_marker;
    dummy_marker.header.frame_id = "odom_combined";
    dummy_marker.header.stamp = ros::Time();
    dummy_marker.ns = "segmentation3d";
    dummy_marker.id = index;
    dummy_marker.type = visualization_msgs::Marker::SPHERE;
    dummy_marker.action = visualization_msgs::Marker::ADD;
    dummy_marker.pose.position.x = 1;
    dummy_marker.pose.position.y = 1;
    dummy_marker.pose.position.z = 1;
    dummy_marker.pose.orientation.x = 0.0;
    dummy_marker.pose.orientation.y = 0.0;
    dummy_marker.pose.orientation.z = 0.0;
    dummy_marker.pose.orientation.w = 1.0;
    dummy_marker.scale.x = 1;
    dummy_marker.scale.y = 0.1;
    dummy_marker.scale.z = 0.1;
    dummy_marker.color.a = 0.0;
    dummy_marker.color.r = 0.0;
    dummy_marker.color.g = 1.0;
    dummy_marker.color.b = 0.0;
    return dummy_marker;
  }
};

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "segmentation3d");
  Segmentation3d seg3d;

  // Spin
  ros::spin ();
  
  return 0;
}


