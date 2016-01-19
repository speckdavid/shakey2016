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
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/filter.h>
// Opencv specific includes
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
// Local files
#include "./classifier.h"

class Segmentation3d
{
  ros::NodeHandle nh;
  ros::Publisher vis_pub;
  // ros::Publisher cloud_pub;
  ros::Subscriber sub;
  tf::TransformListener listener;
  Classifier cl;
  bool tf_error;
  Object_class obj;

public:  
  Segmentation3d()
  {
    sub = nh.subscribe ("/head_mount_kinect/depth/points", 1,
      &Segmentation3d::cloud_cb, this);
    // sub = nh.subscribe ("/head_mount_kinect/depth_registered/points", 1,
    //  &Segmentation3d::cloud_cb, this);
    vis_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_markers", 0);
    // cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("out_cloud", 0);
    tf_error = false;
    obj = Ground;
    std::cerr << "Start object recognition:" << std::endl;
  }  

  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
  {
	visualization_msgs::MarkerArray markerArray;
	int number_marker = 5, cur_number_markers = 0;
	markerArray.markers.resize(5);
	tf_error = false;
	obj = Ground;
	// Create some clouds
	pcl::PointCloud<pcl::PointXYZ> cloud_out_ob = transformed_cloud(input);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out = cloud_out_ob.makeShared();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

	// Abort if tf_error occurs
	if (tf_error) return;

    // Create plane representation
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    std::vector<pcl::ModelCoefficients> all_coeffs;
    std::vector<pcl::PointIndices> all_inliers;
    
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
    // While 5% of the original cloud is still there
    while (cloud_out->points.size () > 0.1 * nr_points)
    {
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud (cloud_out);
      seg.segment (*inliers, *coefficients);

      if (inliers->indices.size () == 0)
      {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
      }

      // Extract the inlier
      extract.setInputCloud (cloud_out);
      extract.setIndices (inliers);
      extract.setNegative (false);
      extract.filter (*cloud_p);

      // Create the filtering object
      extract.setNegative (true);
      extract.filter (*cloud_f);
      cloud_out.swap (cloud_f);

      // debug for cloud
      sensor_msgs::PointCloud2 out;
      pcl::toROSMsg(*cloud_p, out);
      // cloud_pub.publish(out);

      // Check for ground plane and "small" surfaces
      if ((std::abs(coefficients->values[0]) < 0.05 && std::abs(coefficients->values[1]) < 0.05
      	    && std::abs(coefficients->values[2])-1 < 0.05
			&& std::abs(coefficients->values[3]) < 0.1) || inliers->indices.size() < 1000)
    	  continue;
      // save relevant coefficiencs
      all_coeffs.push_back(*coefficients);
      all_inliers.push_back(*inliers);

      // Remove all points which are far from centroid and therefore "possibly" not on surface
      Eigen::Vector4f pcaCentroid;
      float distance = 0.75f;
      pcl::compute3DCentroid(*cloud_p, pcaCentroid);
      pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new
            pcl::ConditionAnd<pcl::PointXYZ> ());
      range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
          pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, pcaCentroid(0) - distance)));
      range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
          pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, (pcaCentroid(0) + distance))));
      range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
          pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, pcaCentroid(1) - distance)));
      range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
          pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, (pcaCentroid(1) + distance))));
      range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
          pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, pcaCentroid(2) - distance)));
      range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
          pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, (pcaCentroid(2) + distance))));
      pcl::ConditionalRemoval<pcl::PointXYZ> condrem (range_cond);
      condrem.setInputCloud (cloud_p);
      condrem.filter (*cloud_p);

      // parallel plane to ground plane (top plane of box)
      if (std::abs(coefficients->values[0]) < 0.05 && std::abs(coefficients->values[1]) < 0.05
            	    && std::abs(coefficients->values[2])-1 < 0.05  && std::abs(coefficients->values[3]) > 0.1) {
    	obj = Box;
    	visualization_msgs::Marker marker = boxMarkerFromTopPlane(cloud_p, coefficients, cur_number_markers);
    	markerArray.markers[cur_number_markers] = marker;
    	cur_number_markers++;
      }
      // crooked plane (wedge)
      if ( (1 - std::abs(coefficients->values[0]) < 0.75 || 1 - std::abs(coefficients->values[1])  < 0.75)
    		  && 1 - std::abs(coefficients->values[2]) < 0.75 && std::abs(coefficients->values[2]) < 0.9) {
        obj = Wedge;
        visualization_msgs::Marker marker = wedgeMarkerFromCrookedPlane(cloud_p, coefficients, cur_number_markers);
    	markerArray.markers[cur_number_markers] = marker;
    	cur_number_markers++;
      }
      // Dumb to console
      dump_console(inliers, coefficients, i);
      i++;
    }
    for (int i = cur_number_markers; i < number_marker; i++)
    	markerArray.markers[i] = dummyMarker(cur_number_markers);
    vis_pub.publish(markerArray);
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
	  listener.waitForTransform("map", input->header.frame_id,
	    input->header.stamp , ros::Duration(0.5));
      listener.lookupTransform("map", input->header.frame_id,
        input->header.stamp, transform);
    }
    catch (tf::TransformException &ex) {
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
          tf_error = true;
    }
    pcl_ros::transformPointCloud(cloud_in, cloud_out, transform);
    cloud_out.header.frame_id = "map";
    return cloud_out;
  }
  
  visualization_msgs::Marker boxMarkerFromTopPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p,
  		  pcl::ModelCoefficients::Ptr coefficients, int id) {
	  std::vector<Eigen::Vector3f> minBox = minimal2DBoundingBox(cloud_p, coefficients);
	  printPushData(minBox);
	  float width_p, length_p;
	  Eigen::Matrix3f eigenVectors = eigenVectorsAndSize(minBox, &width_p, &length_p);
	  visualization_msgs::Marker marker = dummyMarker(id);
	  marker.type = visualization_msgs::Marker::CUBE;
	  // Translation of points to the mean
	  marker.pose.position.x = minBox[4](0);
	  marker.pose.position.y = minBox[4](1);
	  marker.pose.position.z = minBox[4](2)/2;
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
	  marker.scale.z = minBox[0](2);
	  marker.color.a = 1.0;
	  return marker;
  }

  visualization_msgs::Marker wedgeMarkerFromCrookedPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p,
		  pcl::ModelCoefficients::Ptr coefficients, int id) {
      std::vector<Eigen::Vector3f> minBox = minimal2DBoundingBox(cloud_p, coefficients);
      float width_p, length_p;
      Eigen::Matrix3f eigenVectors = eigenVectorsAndSize(minBox, &width_p, &length_p);
      visualization_msgs::Marker marker = dummyMarker(id);
      marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
	  marker.color.a = 1.0;
      std::vector<geometry_msgs::Point> ros_p;
      for (int i = 0; i < 4; i++) {
        ros_p.push_back(getRosPoint(minBox[i]));
      }
      int upperPointsInd[2] = {-1, -1};
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
        for (int j=i+1; j < 6; j++) {
          for (int k=j+1; k < 6; k++) {
            marker.points.push_back(ros_p[i]);
            marker.points.push_back(ros_p[j]);
            marker.points.push_back(ros_p[k]);
          }
        }
      }
      return marker;
  }

  visualization_msgs::Marker dummyMarker(int id) {
	  visualization_msgs::Marker marker;
	  marker.header.frame_id = "map";
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

  void dump_console(pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients, int planeNumber)
  {
    // Print data
	pcl_msgs::ModelCoefficients ros_coefficients;
	pcl_conversions::fromPCL(*coefficients, ros_coefficients);
    std::cerr << planeNumber << ". Model coefficients: " 
                             << ros_coefficients.values[0] << " "
                             << ros_coefficients.values[1] << " "
                             << ros_coefficients.values[2] << " "
                             << ros_coefficients.values[3] << std::endl;
    std::cerr << "with " << inliers->indices.size () << " Points\n" << std::endl;     
  }

  std::vector<Eigen::Vector3f> minimal2DBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p,
		  pcl::ModelCoefficients::Ptr coefficients)
  {
    std::vector<Eigen::Vector3f> plane_bbx;

    // store the table top plane parameters
    Eigen::Vector3f plane_normal;
    plane_normal.x() = coefficients->values[0];
    plane_normal.y() = coefficients->values[1];
    plane_normal.z() = coefficients->values[2];
    // compute an orthogonal normal to the plane normal
    Eigen::Vector3f v = plane_normal.unitOrthogonal();
    // take the cross product of the two normals to get
    // a thirds normal, on the plane
    Eigen::Vector3f u = plane_normal.cross(v);

    // project the 3D point onto a 2D plane
    std::vector<cv::Point2f> points;
    // choose a point on the plane
    Eigen::Vector3f p0(cloud_p->points[0].x,
    				   cloud_p->points[0].y,
                       cloud_p->points[0].z);
    for(unsigned int i = 0; i < cloud_p->points.size(); i++)
    {
      Eigen::Vector3f p3d(cloud_p->points[i].x,
    		              cloud_p->points[i].y,
			              cloud_p->points[i].z);

      // subtract all 3D points with a point in the plane
      // this will move the origin of the 3D coordinate system
      // onto the plane
      p3d = p3d - p0;

      cv::Point2f p2d;
      p2d.x = p3d.dot(u);
      p2d.y = p3d.dot(v);
      points.push_back(p2d);
    }

    cv::Mat points_mat(points);
    cv::RotatedRect rrect = cv::minAreaRect(points_mat);
    cv::Point2f rrPts[4];
    rrect.points(rrPts);

    //store the table top bounding points in a vector
    for(unsigned int i=0; i<4; i++)
    {
      Eigen::Vector3f pbbx(rrPts[i].x*u + rrPts[i].y*v + p0);
      plane_bbx.push_back(pbbx);
    }
    Eigen::Vector3f center(rrect.center.x*u + rrect.center.y*v + p0);
    plane_bbx.push_back(center);

    // 0-3: 4 corner points, 4: mean
    return plane_bbx;
  }

  Eigen::Matrix3f eigenVectorsAndSize(std::vector<Eigen::Vector3f> plane_bbx, float *width, float *length) {
	  Eigen::Matrix3f eigenVectors;;
	  Eigen::MatrixXf::Index index;
	  Eigen::MatrixXf points(3,3);
	  points << plane_bbx[0], plane_bbx[1], plane_bbx[2];
	  // find nearest neighbour
	  Eigen::VectorXf dist = (points.colwise() - plane_bbx[3]).colwise().squaredNorm();
	  dist.minCoeff(&index);
	  *width = dist(index);
	  Eigen::Vector3f e0 = (plane_bbx[3] - plane_bbx[index]).normalized();
	  // set the distance of e0 to infinity
	  dist(index) = std::numeric_limits<float>::infinity();
	  dist.minCoeff(&index);
	  *length= dist(index);
	  Eigen::Vector3f e1 = (plane_bbx[3] - plane_bbx[index]).normalized();
	  Eigen::Vector3f e2 = e0.cross(e1);
	  eigenVectors << e0, e1, e2;
	  return eigenVectors;
  }

  // TODO: Verify order of points
  void printPushData(std::vector<Eigen::Vector3f> plane_bbx) {
	  std::cerr << plane_bbx[0]<< plane_bbx[1]<< plane_bbx[2]<< plane_bbx[3]<< std::endl;
	  Eigen::Vector3f push_p1 = (plane_bbx[0] + plane_bbx[1]) * 0.5f;
	  Eigen::Vector3f push_p2 = (plane_bbx[0] + plane_bbx[3]) * 0.5f;
	  Eigen::Vector3f push_p3 = (plane_bbx[2] + plane_bbx[1]) * 0.5f;
	  Eigen::Vector3f push_p4 = (plane_bbx[2] + plane_bbx[3]) * 0.5f;
	  Eigen::Vector3f push_v1 = plane_bbx[4] - push_p1;
	  Eigen::Vector3f push_v2 = plane_bbx[4] - push_p2;
	  Eigen::Vector3f push_v3 = plane_bbx[4] - push_p3;
	  Eigen::Vector3f push_v4 = plane_bbx[4] - push_p4;
	  push_p1 -= 1.2 * push_v1;
	  push_p2 -= 1.2 * push_v2;
	  push_p3 -= 1.2 * push_v3;
	  push_p4 -= 1.2 * push_v4;
	  std::cerr << "[(" << push_p1.x() << ", " << push_p1.y() << ", 0), (" <<
			  push_v1.x() << ", " << push_v1.y() << ", 0)]" << std::endl;
	  std::cerr << "[(" << push_p2.x() << ", " << push_p2.y() << ", 0), (" <<
			  push_v2.x() << ", " << push_v2.y() << ", 0)]" << std::endl;
	  std::cerr << "[(" << push_p3.x() << ", " << push_p3.y() << ", 0), (" <<
			  push_v3.x() << ", " << push_v3.y() << ", 0)]" << std::endl;
	  std::cerr << "[(" << push_p4.x() << ", " << push_p4.y() << ", 0), (" <<
			  push_v4.x() << ", " << push_v4.y() << ", 0)]" << std::endl;
  }

  geometry_msgs::Point getRosPoint(Eigen::Vector3f point)
  {
	  geometry_msgs::Point p;
	  p.x = point(0);
	  p.y = point(1);
	  p.z = point(2);
	  return p;
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
