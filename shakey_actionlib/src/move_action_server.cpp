#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base/move_base.h>
#include <geometry_msgs/Twist.h>
#include <shakey_actionlib/MoveAction.h>
#include <tf/transform_listener.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class MoveAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<shakey_actionlib::MoveAction> as_;
  ros::ServiceClient move_base_client;
  std::string action_name_;
  tf::TransformListener listener_;

public:

  MoveAction(std::string name) :
    as_(nh_, name, boost::bind(&MoveAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~MoveAction(void)
  {
  }

  void executeCB(const shakey_actionlib::MoveGoalConstPtr &goal)
  {
	// publish info to the console for the user
    std::cerr << "Move position: [" << goal->target_pose.position.x << ", "
    		<< goal->target_pose.position.y << ", "<< goal->target_pose.position.z << "]"<< std::endl;

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
    goalMB.target_pose.pose.position.x = goal->target_pose.position.x;
    goalMB.target_pose.pose.position.y = goal->target_pose.position.y;
    goalMB.target_pose.pose.position.z = goal->target_pose.position.z;
    goalMB.target_pose.pose.orientation = goal->target_pose.orientation;

    ROS_INFO("Sending goal to move_base.");
    ROS_INFO("Moving to position...");
    ac.sendGoal(goalMB);
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Moving to position completed.");
    else {
      ROS_INFO("Not able to move to position.");
      as_.setPreempted();
      return;
    }
    as_.setSucceeded();
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_action_server");

  MoveAction move(ros::this_node::getName());
  ros::spin();

  return 0;
}
