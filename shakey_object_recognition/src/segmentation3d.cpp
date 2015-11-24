#include <ros/ros.h>
#include <tf/transform_listener.h>
// PCL specific includes
#include "pcl/common/eigen.h"
#include "pcl/common/angles.h"
#include <sensor_msgs/PointCloud2.h>
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

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud_in;
  pcl::PointCloud<pcl::PointXYZ> cloud_out;
  pcl::fromROSMsg (*input, cloud_in);
  
  tf::TransformListener listener;
  tf::StampedTransform transform;
  try
  {
	listener.waitForTransform("odom_combined", input->header.frame_id, 
	ros::Time(0), ros::Duration(1));
    listener.lookupTransform("odom_combined", input->header.frame_id, 
    ros::Time(0), transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
  }
  pcl_ros::transformPointCloud(cloud_in, cloud_out, transform);
  cloud_out.header.frame_id = "odom_combined";
  
  //--------------------------------------------------------------------
  
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  // Optional
  seg.setOptimizeCoefficients (true);
  // seg.setAxis (Eigen::Vector3f (0.0, -0.9, -0.45));
  // seg.setEpsAngle (15.0f * (M_PI/180.0f));

  // Compute segmentation
  seg.setInputCloud (cloud_out.makeShared());
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  seg.segment (*inliers, *coefficients);

  // Publish the model coefficients
  if (inliers->indices.size() != 0)
  {  
  // Create the filtering object
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud_out.makeShared());
  proj.setModelCoefficients (coefficients);
  pcl::PointCloud<pcl::PointXYZ> cloud_projected;
  proj.filter (cloud_projected);
  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg (cloud_projected, output);

  // Publish the data
  pub.publish (output);
  
  // Print data
  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;
  }  else {
    std::cerr << "Not enough inliers!" << std::endl;
  }
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "segmentation3d");
  ros::NodeHandle nh;
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/head_mount_kinect/depth/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  // pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1);
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}


