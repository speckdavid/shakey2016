#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <ros/callback_queue.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>
#include <Eigen/Geometry>
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
#include <pcl/common/pca.h>
// Local files
#include "./classifier.h"


class Segmentation3d
{
  ros::NodeHandle nh;
  ros::Publisher vis_pub;
  ros::Subscriber sub;
  tf::TransformListener listener;
  Classifier cl;
  bool tf_error;

public:  
  Segmentation3d()
  {
    sub = nh.subscribe ("/head_mount_kinect/depth/points", 1, 
      &Segmentation3d::cloud_cb, this);
    vis_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    tf_error = false;
  }  

  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
  {
	tf_error = false;
	// Create some clouds
	pcl::PointCloud<pcl::PointXYZ> cloud_out_ob = transformed_cloud(input);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out = cloud_out_ob.makeShared();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

	// Abort if tf_error occurs
	if (tf_error) return;

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
        break;
      }
      coeffs.push_back(*coefficients);

      // Extract the inlier
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
      i++;

      //-----------------------------------------------------------
      if (std::abs(coefficients->values[0]) < 0.05 && std::abs(coefficients->values[1]) < 0.05
      	    && std::abs(coefficients->values[2] - 1) < 0.05  && coefficients->values[3] > -0.1)
    	  continue;

      pcl::PCA<pcl::PointXYZ> pca;
      pca.setInputCloud(cloud_p);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
      pca.project(*cloud_p, *cloudPCAprojection);
      pcl::PointXYZ minPoint, maxPoint;
      pcl::getMinMax3D(*cloudPCAprojection, minPoint, maxPoint);
      Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

      visualization_msgs::Marker marker;
      marker.header.frame_id = "odom_combined";
      marker.header.stamp = ros::Time();
      marker.ns = "segmentation3d";
      marker.id = i;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;

      // Rotation by EigenVectors around the middle point of the bounding box
      // Translation of points to the mean
      Eigen::Vector3f position = pca.getEigenVectors() * meanDiagonal + pca.getMean().head<3>();
      marker.pose.position.x = position(0);
      marker.pose.position.y = position(1);
      marker.pose.position.z = position(2);

      // Rotation given by EigenVectors as Quaternion
      Eigen::Matrix3d m = pca.getEigenVectors().cast<double>();
      Eigen::Quaterniond q(m);
      q.normalize();
      geometry_msgs::Quaternion plane_orientation;
      tf::Quaternion qt;
      tf::quaternionEigenToTF(q, qt);
      tf::quaternionTFToMsg(qt, plane_orientation);
      marker.pose.orientation = plane_orientation;

      // Construct scale from max and min values (Bounding box)
      Eigen::Vector3f scale = maxPoint.getVector3fMap() - minPoint.getVector3fMap();
      marker.scale.x = std::abs((float)scale(0));
      marker.scale.y = std::abs((float)scale(1));
      marker.scale.z = std::abs((float)scale(2));
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      //------------------------------------------------
      vis_pub.publish(marker);
    }
    Object_class obj = cl.classify(coeffs);
    if (obj == Box) std::cerr << "BOX" << std::endl;
    if (obj == Wedge) std::cerr << "WEDGE" << std::endl;
    if (obj == Ground) std::cerr << "GROUND" << std::endl;
    std::cerr << "--------------------------------------------------------------" << std::endl;
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
	    input->header.stamp , ros::Duration(0.5));
      listener.lookupTransform("odom_combined", input->header.frame_id, 
        input->header.stamp, transform);
    }
    catch (tf::TransformException &ex) {
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
          tf_error = true;
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
};

int main (int argc, char** argv)
{
  // Initialise ROS
  ros::init (argc, argv, "segmentation3d");
  Segmentation3d seg3d;

  // Spin
  ros::spin ();
  
  return 0;
}


