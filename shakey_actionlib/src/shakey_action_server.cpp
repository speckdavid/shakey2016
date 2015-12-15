#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <shakey_actionlib/PushAction.h>
#include <topic_tools/MuxSelect.h>
#include <move_base/move_base.h>

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
  float offset;

public:

  PushAction(std::string name) :
    as_(nh_, name, boost::bind(&PushAction::executeCB, this, _1), false),
    action_name_(name)
  {
    offset = 2;
    as_.start();
  }

  ~PushAction(void)
  {
  }

  void executeCB(const shakey_actionlib::PushGoalConstPtr &goal)
  {
	// publish info to the console for the user
    std::cerr << "Push position: [" << goal->target_point.x << ", "
    		<< goal->target_point.y << ", "<< goal->target_point.z << "]"<< std::endl;
    std::cerr << "Push direction: [" << goal->push_direction.x << ", "
    		<< goal->push_direction.y << ", "<< goal->push_direction.z << "]" << std::endl;
    std::cerr << "Push distance: " << goal->push_distance << std::endl;

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);
    //wait for the move_base action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    // Move in front of objection
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
    }

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

    // Move forward and push object
    move_base_msgs::MoveBaseGoal goalMB2;
    goalMB2.target_pose.header.frame_id = "base_link";
    goalMB2.target_pose.header.stamp = ros::Time::now();
    goalMB2.target_pose.pose.position.x = goal->push_distance + offset;
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
    move_base_msgs::MoveBaseGoal goalMB3;
    goalMB3.target_pose.header.frame_id = "base_link";
    goalMB3.target_pose.header.stamp = ros::Time::now();
    goalMB3.target_pose.pose.position.x = -offset;
    goalMB3.target_pose.pose.orientation.w = 1.0;

    // TODO: Backward drive not via move_base (turns around at the moment)
    ROS_INFO("Sending goal to move_base. \n Moving back...");
    ROS_INFO("Moving back...");
    ac.sendGoal(goalMB3);
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
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

    ROS_INFO("Push action completed.");
    as_.setSucceeded();
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "shakey_push_server");

  PushAction push(ros::this_node::getName());
  ros::spin();

  return 0;
}
