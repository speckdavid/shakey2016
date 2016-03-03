#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <shakey_actionlib/PushAction.h>
#include <topic_tools/MuxSelect.h>
#include <move_base/move_base.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class PushAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<shakey_actionlib::PushAction> as_;
  ros::ServiceClient mux_client;
  ros::ServiceClient move_base_client;
  std::string action_name_;
  // Odom drive (mainly for backward drive)
  ros::Publisher cmd_vel_pub_;
  tf::TransformListener listener_;

  bool dirveStraightOdom(float velocity, float distance) {
	  //wait for the listener to get the first message
	      listener_.waitForTransform("base_footprint", "odom_combined",
	                                 ros::Time(0), ros::Duration(1.0));

	      //we will record transforms here
	      tf::StampedTransform start_transform;
	      tf::StampedTransform current_transform;

	      //record the starting transform from the odometry to the base frame
	      listener_.lookupTransform("base_footprint", "odom_combined",
	                                ros::Time(0), start_transform);

	      //we will be sending commands of type "twist"
	      geometry_msgs::Twist base_cmd;
	      //the command will be to go forward at 0.25 m/s
	      base_cmd.linear.y = base_cmd.linear.z = 0;
	      base_cmd.linear.x = velocity;

	      ros::Rate rate(10.0);
	      bool done = false;
	      while (!done && nh_.ok())
	      {
	        //send the drive command
	        cmd_vel_pub_.publish(base_cmd);
	        rate.sleep();
	        //get the current transform
	        try
	        {
	          listener_.lookupTransform("base_footprint", "odom_combined",
	                                    ros::Time(0), current_transform);
	        }
	        catch (tf::TransformException &ex)
	        {
	          ROS_ERROR("%s",ex.what());
	          break;
	        }
	        //see how far we've traveled
	        tf::Transform relative_transform =
	          start_transform.inverse() * current_transform;
	        double dist_moved = relative_transform.getOrigin().length();

	        if(std::abs(dist_moved) > std::abs(distance)) done = true;
	      }
	      if (done) return true;
	      return false;
  }

public:

  PushAction(std::string name) :
    as_(nh_, name, boost::bind(&PushAction::executeCB, this, _1), false),
    action_name_(name)
  {
	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/base_controller/command", 1);
    as_.start();
  }

  ~PushAction(void)
  {
  }

  void executeCB(const shakey_actionlib::PushGoalConstPtr &goal)
  {
	// publish info to the console for the user
    /*std::cerr << "Push position: [" << goal->target_point.x << ", "
    		<< goal->target_point.y << ", "<< goal->target_point.z << "]"<< std::endl;
    std::cerr << "Push direction: [" << goal->push_direction.x << ", "
    		<< goal->push_direction.y << ", "<< goal->push_direction.z << "]" << std::endl;*/
    std::cerr << "Push distance: " << goal->push_distance << std::endl;

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);
    //wait for the move_base action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    /*// Move in front of objection
    move_base_msgs::MoveBaseGoal goalMB;
    goalMB.target_pose.header.frame_id = "map";
    goalMB.target_pose.header.stamp = ros::Time::now();
    // stop with some offset before the object
    goalMB.target_pose.pose.position.x = goal->target_point.x - offset * goal->push_direction.x;
    goalMB.target_pose.pose.position.y = goal->target_point.y - offset * goal->push_direction.y;
    goalMB.target_pose.pose.position.z = goal->target_point.z - offset * goal->push_direction.z;

    // Set correct orientation
    tf::Vector3 plane_norm(goal->push_direction.x, goal->push_direction.y, goal->push_direction.z);
    plane_norm = plane_norm.normalized();
    tf::Quaternion qt = tf::shortestArcQuat (tf::Vector3(1,0,0), plane_norm);
    geometry_msgs::Quaternion robot_orientation;
    tf::quaternionTFToMsg(qt, robot_orientation);
    goalMB.target_pose.pose.orientation = robot_orientation;

    ROS_INFO("Sending goal to move_base.");
    ROS_INFO("Moving to push position...");
    ac.sendGoal(goalMB);
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Moving to push position completed.");
    else {
      ROS_INFO("Not able to move to push location.");
      as_.setPreempted();
      return;
    }*/

    // Swap to base_scan_filtered
    mux_client = nh_.serviceClient<topic_tools::MuxSelect>("base_scan_mux_select");
    topic_tools::MuxSelect sel;
    sel.request.topic = "base_scan_filtered";
    if (mux_client.call(sel))
      ROS_INFO("Swap to base_scan_filtered completed.");
    else {
      ROS_INFO("Not able to swap to base_scan_filtered.");
      as_.setPreempted();
      return;
    }

    // Clear cost map
    move_base_client = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    std_srvs::Empty e;
    if (move_base_client.call(e))
      ROS_INFO("Costmap cleared completed");
    else {
      ROS_INFO("Not able to clear costmap.");
      as_.setPreempted();
      return;
    }

    ros::Duration(0.25).sleep();
    // Move forward and push object
    move_base_msgs::MoveBaseGoal goalMB2;
    goalMB2.target_pose.header.frame_id = "base_link";
    goalMB2.target_pose.header.stamp = ros::Time::now();
    goalMB2.target_pose.pose.position.x = goal->push_distance;
    goalMB2.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal to move_base.");
    ROS_INFO("Pushing object...");
    ac.sendGoal(goalMB2);
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Pushing object completed.");
    else {
      ROS_INFO("Not able to push the object.");
      as_.setPreempted();
      return;
    }

    // Move back
    ROS_INFO("Moving back via cmd command...");
    ROS_INFO("Moving back...");
    if(this->dirveStraightOdom(-0.25, 0.75))
      ROS_INFO("Moving back completed.");
    else {
      ROS_INFO("Not able to move back.");
      as_.setPreempted();
      return;
    }

    // Swap back to base_scan
    topic_tools::MuxSelect sel2;
    sel2.request.topic = "base_scan";
    if (mux_client.call(sel2))
      ROS_INFO("Swap to base_scan completed.");
    else {
      ROS_INFO("Not able to swap to base_scan.");
      as_.setPreempted();
      return;
    }

    ros::Duration(0.25).sleep();
    ROS_INFO("Push action completed.");
    as_.setSucceeded();
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "push_action_server");

  PushAction push(ros::this_node::getName());
  ros::spin();

  return 0;
}
