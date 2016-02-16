#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/PointCloud2.h>
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
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
// Opencv specific includes
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
// Local files
#include "shakey_object_recognition/DetectObjects.h"
#include "shakey_object_recognition/objectVisualisation.h"


enum Object_class {Box, Wedge, Ground, Unkown};

class ObjectDetection
{
  int _maxIteratrions;
  double _distanceThreshold;
  double _remainingPoints;
  float _maxDistanceToCentroid;
  double _minXDistance;
  double _minYDistance;
  float _groundAngleDelta;
  float _boxAngleDelta;
  float _wedgeAngleDelta;
  float _groundDistDelta;
  float _boxDistDelta;
  float _wedgeDistDelta;
  int _minClusterPoints;

  ros::NodeHandle nh;
  ros::Publisher vis_pub;
  ros::Publisher cloud_pub;
  ros::Subscriber sub;
  tf::TransformListener listener;
  Object_class obj;
  ros::ServiceServer service;
  sensor_msgs::PointCloud2ConstPtr cur_cloud;
  std::vector<geometry_msgs::Pose> push_poses;
  bool tf_error;
  ObjectVisualisation visObjs;

  std::string _worldFrame;

public:  
  ObjectDetection()
  {
    load_params();
    sub = nh.subscribe ("/head_mount_kinect/depth/points", 1,
      &ObjectDetection::normal_spin, this);
    service = nh.advertiseService("/detectObject", &ObjectDetection::detect, this);
    // sub = nh.subscribe ("/head_mount_kinect/depth_registered/points", 1,
    //  &Segmentation3d::cloud_cb, this);;
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("seg_cloud", 0);
    obj = Ground;
    std::cerr << "Start object recognition:\nWaiting for service call..." << std::endl;
    //cur_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    tf_error = true;
    cur_cloud = sensor_msgs::PointCloud2ConstPtr();
    visObjs.setNodeHandle(nh);
    visObjs.setWorldFrame(_worldFrame);
  }  

  bool detect(shakey_object_recognition::DetectObjects::Request &req,
		  	  	  	   shakey_object_recognition::DetectObjects::Response &res) {
     visObjs.resetMarker();
	 tf_error = true;
	 pcl::PointCloud<pcl::PointXYZ> cloud_in;
	 int num_tf_error = 0;
	 while (tf_error) {
		 if (num_tf_error == 2) {
			 ROS_INFO("Canceled due to tf error!");
			 return false;
		 }
		 cloud_in = transformed_cloud(cur_cloud);
		 if (!tf_error) cloud_cb(cloud_in);
		 num_tf_error++;
	  }
	  res.push_locations = push_poses;
	  res.obj_type = obj;
	  return true;
  }

  void load_params()
  {
	  std::cerr << "--------------------------------------------------------------" << std::endl;
	  std::cerr << "--------------------------------------------------------------" << std::endl;
	  ROS_INFO("CURRENT PARAMETERS:");
	  ros::NodeHandle nh("~");
	  std::string nspace = nh.getNamespace();;

	  nh.getParam("ObjectDetection/maxIterations", _maxIteratrions);
	  std::ostringstream temp;
	  temp << _maxIteratrions;
	  ROS_INFO("Maximal Iterations: %s", temp.str().c_str());

	  nh.getParam("ObjectDetection/minClusterPoints", _minClusterPoints);
	  temp.str("");
	  temp.clear();
	  temp << _minClusterPoints;
	  ROS_INFO("Minimal Points in a Cluster: %s", temp.str().c_str());

	  nh.getParam("ObjectDetection/distanceThreshold", _distanceThreshold);
	  temp.str("");
	  temp.clear();
	  temp << _distanceThreshold;
	  ROS_INFO("Distance Threshold: %s", temp.str().c_str());

	  double d_tmp;
	  nh.getParam("ObjectDetection/maxDistanceToCentroid", d_tmp);
	  _maxDistanceToCentroid = d_tmp + 0.0f;
	  temp.str("");
	  temp.clear();
	  temp << _maxDistanceToCentroid;
	  ROS_INFO("Max Distance to Centroid: %s", temp.str().c_str());

	  nh.getParam("ObjectDetection/remainingPoints", _remainingPoints);
	  temp.str("");
	  temp.clear();
	  temp << _remainingPoints;
	  ROS_INFO("Remaining Points: %s", temp.str().c_str());

	  nh.getParam("ObjectDetection/minXDistance", _minXDistance);
	  temp.str("");
	  temp.clear();
	  temp << _minXDistance;
	  ROS_INFO("Minimal X Distance: %s", temp.str().c_str());

	  nh.getParam("ObjectDetection/minYDistance", _minYDistance);
	  temp.str("");
	  temp.clear();
	  temp << _minYDistance;
	  ROS_INFO("Minimal Y Distance: %s", temp.str().c_str());

	  nh.getParam("ObjectDetection/groundAngleDelta", d_tmp);
	  _groundAngleDelta = d_tmp + 0.0f;
	  nh.getParam("ObjectDetection/groundDistDelta", d_tmp);
	  _groundDistDelta = d_tmp + 0.0f;
	  temp.str("");
	  temp.clear();
	  temp << _groundAngleDelta << "deg and w < " << _groundDistDelta;
	  ROS_INFO("Ground Angle Delta: %s", temp.str().c_str());

	  nh.getParam("ObjectDetection/boxAngleDelta", d_tmp);
	  _boxAngleDelta = d_tmp + 0.0f;
	  nh.getParam("ObjectDetection/boxDistDelta", d_tmp);
	  _boxDistDelta = d_tmp + 0.0f;
	  temp.str("");
	  temp.clear();
	  temp << _boxAngleDelta << "deg and " << "w > " << _boxDistDelta;
	  ROS_INFO("Box Angle Delta: %s", temp.str().c_str());

	  nh.getParam("ObjectDetection/wedgeAngleDelta", d_tmp);
	  _wedgeAngleDelta = d_tmp + 0.0f;
	  nh.getParam("ObjectDetection/wedgeDistDelta", d_tmp);
	  _wedgeDistDelta = d_tmp + 0.0f;
	  temp.str("");
	  temp.clear();
	  temp << _wedgeAngleDelta << "deg and w > " << _wedgeDistDelta;
	  ROS_INFO("Wedge Delta: %s", temp.str().c_str());

      nh.getParam("ObjectDetection/world_frame", _worldFrame);
      ROS_INFO("Using world_frame = %s", _worldFrame.c_str());
	  std::cerr << "--------------------------------------------------------------" << std::endl;
  }

  void normal_spin(const sensor_msgs::PointCloud2ConstPtr& input) {
	 cur_cloud = input;
	 visObjs.publish();
  }

  void cloud_cb (pcl::PointCloud<pcl::PointXYZ> cloud_in)
  {
	obj = Ground;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out = cloud_in.makeShared();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    // Create plane representation
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (_maxIteratrions);
    seg.setDistanceThreshold (_distanceThreshold);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    int i = 0, nr_points = (int) cloud_out->points.size ();
    // While 5% of the original cloud is still there
    while (cloud_out->points.size () > _remainingPoints * nr_points)
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

      // Create Vectors for angle checks
      Eigen::Vector3f normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
      float w = coefficients->values[3] * normal.norm();
      normal.normalize();
      float ang = angle(normal, Eigen::Vector3f (0,0,1));
      // Direction does not matter.
      if (ang > 90) ang = std::abs(180 - ang);

      // Dump console
      std::cerr << "\n" << std::endl;
      std::ostringstream temp;
      temp << i << "-th " << "PLANE: ";
      temp << "Coefficients: " << "[" << normal[0] << ", " << normal[1]
		   << ", " << normal[2] << ", " << w << "] with " << inliers->indices.size () << " Points and ";
      temp << "Angle Deviation from [0, 0, 1]: " << ang;
      ROS_INFO("%s", temp.str().c_str());

      // debug for cloud
      sensor_msgs::PointCloud2 out;
      pcl::toROSMsg(*cloud_p, out);
      cloud_pub.publish(out);
      //ros::Duration(5).sleep();


      // Check for ground plane
      if (ang < _groundAngleDelta && std::abs(w) < _groundDistDelta) {
    	  ROS_INFO("=> GROUND PLANE");
    	  i++;
    	  continue;
      }

      // Creating the KdTree object for the search method of the extraction
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud (cloud_p);

      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      ec.setClusterTolerance (_distanceThreshold);
      ec.setMinClusterSize (_minClusterPoints);
      ec.setMaxClusterSize (nr_points);
      ec.setSearchMethod (tree);
      ec.setInputCloud (cloud_p);
      ec.extract (cluster_indices);
      temp.str("");
      temp.clear();
      temp << "Number of clusters: " << cluster_indices.size();
      ROS_INFO("%s", temp.str().c_str());
      // Extract the inlier
      if (cluster_indices.size() < 1) {
    	  ROS_INFO("No Clusters found. Continue with next plane.\n");
    	  i++;
    	  continue;
      }

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for (int k = 0; k < cluster_indices.size(); k++) {
    	  pcl::PointIndices::Ptr cur_ind(new  pcl::PointIndices(cluster_indices[k]));
    	  extract.setInputCloud (cloud_p);
    	  extract.setIndices (cur_ind);
    	  extract.setNegative (false);
    	  extract.filter (*cloud_cluster);

    	  temp.str("");
    	  temp.clear();
    	  temp << " " << k << "-th Cluster with "  << cluster_indices[k].indices.size() << " Points.";
    	  ROS_INFO("%s", temp.str().c_str());

          // debug for cloud
          pcl::toROSMsg(*cloud_cluster, out);
          cloud_pub.publish(out);
          //ros::Duration(5).sleep();

          // FIXME: hack - Check if x and y is very small -> no object
          Eigen::Vector4f min, max;
          pcl::getMinMax3D(*cloud_cluster, min, max);
          if (max.x() - min.x() < _minXDistance || max.y() - min.y() < _minYDistance) {
        	  ROS_INFO(" => Not required X or Y distance.");
        	  i++;
        	  continue;
          }
          // parallel plane to ground plane (top plane of box)
          if (ang < _boxAngleDelta && std::abs(w) > _boxDistDelta) {
        	  ROS_INFO(" => BOX TOP PLANE");
        	  obj = Box;
        	  std::vector<Eigen::Vector3f> minBox = minimal2DBoundingBox(cloud_cluster, coefficients);
        	  push_poses = getPushData(minBox);
        	  float width_p, length_p;
        	  Eigen::Matrix3f eigenVectors = eigenVectorsAndSize(minBox, &width_p, &length_p);
        	  visObjs.addBoxMarkerFromTopPlane(minBox, eigenVectors, width_p, length_p);
          }
          // crooked plane (wedge)
          if ( std::abs(45 - ang) < _wedgeAngleDelta && std::abs(w) > _wedgeDistDelta) {
        	  ROS_INFO(" => WEDGE CROOKED PLANE");
        	  obj = Wedge;
        	  std::vector<Eigen::Vector3f> minBox = minimal2DBoundingBox(cloud_cluster, coefficients);
        	  push_poses = getPushData(minBox);
        	  float width_p, length_p;
        	  Eigen::Matrix3f eigenVectors = eigenVectorsAndSize(minBox, &width_p, &length_p);
        	  visObjs.addWedgeMarkerFromCrookedPlane(minBox, eigenVectors, width_p, length_p);
          }
      }
      i++;
    }
    std::cerr << "Last detection was: " << std::endl;
    if (obj == Box) std::cerr << "BOX" << std::endl;
    if (obj == Wedge) std::cerr << "WEDGE" << std::endl;
    if (obj == Ground) std::cerr << "GROUND" << std::endl;
    std::cerr << "--------------------------------------------------------------" << std::endl;
    std::cerr << "--------------------------------------------------------------" << std::endl;
  }
  
private:
  pcl::PointCloud<pcl::PointXYZ> transformed_cloud(const sensor_msgs::PointCloud2ConstPtr& input)  
  {
	tf_error = false;
	pcl::PointCloud<pcl::PointXYZ> cloud_in;
	pcl::PointCloud<pcl::PointXYZ> cloud_out;
    pcl::fromROSMsg (*input, cloud_in);
	tf::StampedTransform transform;
    try
    {
	  listener.waitForTransform(_worldFrame, input->header.frame_id,
	    input->header.stamp , ros::Duration(0.5));
      listener.lookupTransform(_worldFrame, input->header.frame_id,
        input->header.stamp, transform);
    }
    catch (tf::TransformException &ex) {
          tf_error = true;
          ros::Duration(0.5).sleep();
          ROS_ERROR("%s",ex.what());
    }
    pcl_ros::transformPointCloud(cloud_in, cloud_out, transform);
    cloud_out.header.frame_id = _worldFrame;
    return cloud_out;
  }

  float angle(Eigen::Vector3f v1, Eigen::Vector3f v2) {
	  float cos_sim = v1.dot(v2) / v1.norm() * v2.norm();
	  return acos(cos_sim) / M_PI * 180;
  }

  std::vector<Eigen::Vector3f> minimal2DBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p,
		  pcl::ModelCoefficients::Ptr coefficients)
  {
    std::vector<Eigen::Vector3f> plane_bbx;

    // store the top plane parameters
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

  // TODO: Verify order of points
  Eigen::Matrix3f eigenVectorsAndSize(std::vector<Eigen::Vector3f> plane_bbx, float *width, float *length) {
	  Eigen::Matrix3f eigenVectors;
	  Eigen::Vector3f e0 = plane_bbx[0] - plane_bbx[1];
	  *width = e0.norm();
	  std::cerr << *width << std::endl;
	  e0.normalize();
	  // set the distance of e0 to infinity
	  Eigen::Vector3f e1 = plane_bbx[0] - plane_bbx[3];
	  *length= e1.norm();
	  e1.normalize();
	  Eigen::Vector3f e2 = e0.cross(e1);
	  eigenVectors << e0, e1, e2;
	  return eigenVectors;
  }

  // TODO: Verify order of points
  std::vector<geometry_msgs::Pose> getPushData(std::vector<Eigen::Vector3f> plane_bbx) {
	  //std::cerr << plane_bbx[0]<< "\n\n" << plane_bbx[1] << "\n\n" << plane_bbx[2] << "\n\n"<< plane_bbx[3]<< std::endl;
	  std::vector<Eigen::Vector3f> points;
	  std::vector<Eigen::Vector3f> vecs;
	  // Add all points if not both are on the floor (flat side on wedge)
	  if (plane_bbx[0](2) > 0.15 || plane_bbx[1](2) > 0.15) {
		  Eigen::Vector3f p0 = (plane_bbx[0] + plane_bbx[1]) * 0.5f;
		  points.push_back(p0);
	  }
	  if (plane_bbx[0](2) > 0.15 || plane_bbx[3](2) > 0.15) {
		  Eigen::Vector3f p1 = (plane_bbx[0] + plane_bbx[3]) * 0.5f;
		  points.push_back(p1);
	  }
	  if (plane_bbx[2](2) > 0.15 || plane_bbx[1](2) > 0.15) {
		  Eigen::Vector3f p2 = (plane_bbx[2] + plane_bbx[1]) * 0.5f;
		  points.push_back(p2);
	  }
	  if (plane_bbx[0](2) > 0.15 || plane_bbx[3](2) > 0.15) {
		  Eigen::Vector3f p3 = (plane_bbx[2] + plane_bbx[3]) * 0.5f;
		  points.push_back(p3);
	  }

	  // std::cerr << "Push locations:" << std::endl;
	  for (int i = 0; i < points.size(); i++) {
		  Eigen::Vector3f v = plane_bbx[4] - points.at(i);
		  points.at(i) -= 1.75 * v;
		  v(2) = 0;
		  vecs.push_back(v);
		  /*std::cerr << "[(" << points.at(i).x() << ", " << points.at(i).y() << ", " << points.at(i).z()
				  << "), (" << vecs.at(i).x() << ", " << vecs.at(i).y() << ", " << vecs.at(i).z() << ")]"
				  << std::endl;*/
	  }
	  std::vector<geometry_msgs::Pose> poses;
	  for (int i = 0; i < points.size(); i++) {
		  geometry_msgs::Pose pose;
		  geometry_msgs::Point position = getRosPoint(points.at(i));
		  position.z = 0;
		  pose.position = position;
		  tf::Vector3 plane_norm(vecs.at(i).x(), vecs.at(i).y(), vecs.at(i).z());
		  plane_norm = plane_norm.normalized();
		  tf::Quaternion qt = tf::shortestArcQuat (tf::Vector3(1,0,0), plane_norm);
		  geometry_msgs::Quaternion robot_orientation;
		  tf::quaternionTFToMsg(qt, robot_orientation);
		  pose.orientation = robot_orientation;
		  poses.push_back(pose);
	  }
	  return poses;
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
  ObjectDetection objDet;

  // Spin
  ros::spin ();
  
  return 0;
}
