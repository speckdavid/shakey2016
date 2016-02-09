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
  std::vector<double> _groundCoeffs;
  std::vector<double> _boxCoeffs;
  std::vector<double> _wedgeCoeffs;
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

public:  
  ObjectDetection()
  {
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
  }  

  bool detect(shakey_object_recognition::DetectObjects::Request &req,
		  	  	  	   shakey_object_recognition::DetectObjects::Response &res) {
     visObjs.resetMarker();
	 tf_error = true;
	 pcl::PointCloud<pcl::PointXYZ> cloud_in;
	 int num_tf_error = 0;
	 while (tf_error) {
		 if (num_tf_error == 2) return false;
		 std::cerr << "tf error" << std::endl;
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
	  nh.getParam("ObjectDetection/distanceThreshold", _distanceThreshold);
	  temp.str("");
	  temp.clear();
	  temp << _distanceThreshold;
	  ROS_INFO("Distance Threshold: %s", temp.str().c_str());
	  double dist;
	  nh.getParam("ObjectDetection/maxDistanceToCentroid", dist);
	  _maxDistanceToCentroid = dist + 0.0f;
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
	  nh.getParam("ObjectDetection/groundCoeffs", _groundCoeffs);
	  temp.str("");
	  temp.clear();
	  temp << "[" << _groundCoeffs[0] << ", " << _groundCoeffs[1] << ", " <<
			  _groundCoeffs[2] << ", " << _groundCoeffs[3] << "]";
	  ROS_INFO("Ground Coefficients: %s", temp.str().c_str());
	  nh.getParam("ObjectDetection/boxCoeffs", _boxCoeffs);
	  temp.str("");
	  temp.clear();
	  temp << "[" << _boxCoeffs[0] << ", " << _boxCoeffs[1] << ", " <<
			  _boxCoeffs[2] << ", " << _boxCoeffs[3] << "]";
	  ROS_INFO("Box Coefficients: %s", temp.str().c_str());
	  nh.getParam("ObjectDetection/wedgeCoeffs", _wedgeCoeffs);
	  temp.str("");
	  temp.clear();
	  temp << "[" << _wedgeCoeffs[0] << ", " << _wedgeCoeffs[1] << ", " <<
	  		  _wedgeCoeffs[2] << ", " << _wedgeCoeffs[3] << "]";
	  ROS_INFO("Wedge Coefficients: %s", temp.str().c_str());
	  std::cerr << "--------------------------------------------------------------" << std::endl;
  }

  void normal_spin(const sensor_msgs::PointCloud2ConstPtr& input) {
	 cur_cloud = input;
	 visObjs.publish();
  }

  void cloud_cb (pcl::PointCloud<pcl::PointXYZ> cloud_in)
  {
	load_params();
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

      // debug for cloud
      sensor_msgs::PointCloud2 out;
      pcl::toROSMsg(*cloud_p, out);
      cloud_pub.publish(out);

      dump_console(inliers, coefficients, i);

      // Check for ground plane
      if ((std::abs(coefficients->values[0]) < _groundCoeffs[0]
			 && std::abs(coefficients->values[1]) < _groundCoeffs[1]
      	    && std::abs(coefficients->values[2]) > _groundCoeffs[2]
			&& std::abs(coefficients->values[3]) < _groundCoeffs[3])) {
    	  ROS_INFO("=> GROUND PLANE\n");
    	  i++;
    	  continue;
      }

      // Remove all points which are far from centroid and therefore "possibly" not on surface
      Eigen::Vector4f pcaCentroid;
      pcl::compute3DCentroid(*cloud_p, pcaCentroid);
      pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new
            pcl::ConditionAnd<pcl::PointXYZ> ());
      range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
          pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, pcaCentroid(0) - _maxDistanceToCentroid)));
      range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
          pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, (pcaCentroid(0) + _maxDistanceToCentroid))));
      range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
          pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, pcaCentroid(1) - _maxDistanceToCentroid)));
      range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
          pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, (pcaCentroid(1) + _maxDistanceToCentroid))));
      range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
          pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, pcaCentroid(2) - _maxDistanceToCentroid)));
      range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
          pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, (pcaCentroid(2) + _maxDistanceToCentroid))));
      pcl::ConditionalRemoval<pcl::PointXYZ> condrem (range_cond);
      condrem.setInputCloud (cloud_p);
      condrem.filter (*cloud_p);

      // FIXME: hack - Check if x and y is very small -> no object
      Eigen::Vector4f min, max;
      pcl::getMinMax3D(*cloud_p, min, max);
      if (max.x() - min.x() < _minXDistance || max.y() - min.y() < _minYDistance) {
    	  ROS_INFO("=> Not required X or Y Distance\n");
    	  i++;
          continue;
      }


      // parallel plane to ground plane (top plane of box)
      if (std::abs(coefficients->values[0]) < _boxCoeffs[0]
    		  && std::abs(coefficients->values[1]) < _boxCoeffs[1]
              && std::abs(coefficients->values[2]) > _boxCoeffs[2]
			  && std::abs(coefficients->values[3]) > _boxCoeffs[3]) {
        ROS_INFO("=> BOX TOP PLANE\n");
    	obj = Box;
  	    std::vector<Eigen::Vector3f> minBox = minimal2DBoundingBox(cloud_p, coefficients);
  	    push_poses = getPushData(minBox);
  	    float width_p, length_p;
  	    Eigen::Matrix3f eigenVectors = eigenVectorsAndSize(minBox, &width_p, &length_p);
  	    visObjs.addBoxMarkerFromTopPlane(minBox, eigenVectors, width_p, length_p);

      }
      // crooked plane (wedge)
      if ( (std::abs(coefficients->values[0]) > _wedgeCoeffs[0]
    		  || std::abs(coefficients->values[1])  < _wedgeCoeffs[1])
    		  && std::abs(coefficients->values[2]) > _wedgeCoeffs[2]
			  && std::abs(coefficients->values[2]) < _wedgeCoeffs[3]) {
    	ROS_INFO("=> WEDGE CROOKED PLANE\n");
        obj = Wedge;
        std::vector<Eigen::Vector3f> minBox = minimal2DBoundingBox(cloud_p, coefficients);
        push_poses = getPushData(minBox);
        float width_p, length_p;
        Eigen::Matrix3f eigenVectors = eigenVectorsAndSize(minBox, &width_p, &length_p);
        visObjs.addWedgeMarkerFromCrookedPlane(minBox, eigenVectors, width_p, length_p);
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
	  listener.waitForTransform("map", input->header.frame_id,
	    input->header.stamp , ros::Duration(0.5));
      listener.lookupTransform("map", input->header.frame_id,
        input->header.stamp, transform);
    }
    catch (tf::TransformException &ex) {
          tf_error = true;
          ros::Duration(0.5).sleep();
          ROS_ERROR("%s",ex.what());
    }
    pcl_ros::transformPointCloud(cloud_in, cloud_out, transform);
    cloud_out.header.frame_id = "map";
    return cloud_out;
  }

  void dump_console(pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients, int planeNumber)
  {
    // Print data
	pcl_msgs::ModelCoefficients ros_coefficients;
	pcl_conversions::fromPCL(*coefficients, ros_coefficients);
    std::cerr << planeNumber << ". Coeffs: ["
                             << ros_coefficients.values[0] << " "
                             << ros_coefficients.values[1] << " "
                             << ros_coefficients.values[2] << " "
                             << ros_coefficients.values[3] <<  "] with " <<
							 inliers->indices.size () << " Points." << std::endl;
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
