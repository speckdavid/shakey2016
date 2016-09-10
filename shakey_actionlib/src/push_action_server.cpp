#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <shakey_actionlib/PushAction.h>
#include <topic_tools/MuxSelect.h>
#include <move_base/move_base.h>
#include <geometry_msgs/Twist.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class PushAction {
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
		tf::StampedTransform start_transform, current_transform;

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
		while (!done && nh_.ok()) {
			//send the drive command
			cmd_vel_pub_.publish(base_cmd);
			rate.sleep();
			//get the current transform
			try {
				listener_.lookupTransform("base_footprint", "odom_combined",
						ros::Time(0), current_transform);
			} catch (tf::TransformException &ex) {
				ROS_ERROR("%s", ex.what());
				break;
			}
			//see how far we've traveled
			tf::Transform relative_transform = start_transform.inverse() * current_transform;
			double dist_moved = relative_transform.getOrigin().length();

			if (std::abs(dist_moved) > std::abs(distance)) done = true;
		}
		if (done) return true;
		return false;
	}

public:

	PushAction(std::string name) :
			as_(nh_, name, boost::bind(&PushAction::executeCB, this, _1),
					false), action_name_(name) {
		cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(
				"/base_controller/push_reverse", 1);
		as_.start();
	}

	~PushAction(void) {
	}

	void executeCB(const shakey_actionlib::PushGoalConstPtr &goal) {
		if (goal->push_distance.size() != goal->push_poses.size()
				|| goal->push_distance.size() > 2) {
			ROS_ERROR(
					"No fitting push-distance %zu and additional push-poses %zu (not equal or greater then 2)",
					goal->push_distance.size(), goal->push_poses.size());
			return;
		}
		//tell the action client that we want to spin a thread by default
		MoveBaseClient ac("move_base", true);
		//wait for the move_base action server to come up
		while (!ac.waitForServer(ros::Duration(5.0))) {
			ROS_INFO("Waiting for the move_base action server to come up");
		}
		for (int i = 0; i < goal->push_distance.size(); i++) {
			ROS_INFO("Push distance: %f - %f (offset)",
					goal->push_distance.at(i), 0.75);
			double remaining_distance = goal->push_distance.at(i);

			// Drive to next push pose for double push action
			if (i == 1 && goal->push_poses.size() == 2) {
				if (goal->push_distance.at(i) - 0.75 < 0.1) {
					ROS_WARN("Push distance smaller then 0.1. Do nothing");
					continue;
				}
				geometry_msgs::Pose p = goal->push_poses.at(i);
				move_base_msgs::MoveBaseGoal goalNewPushPose;
				goalNewPushPose.target_pose.header.frame_id = "map";
				goalNewPushPose.target_pose.header.stamp = ros::Time::now();
				// Orientation stays the same
				goalNewPushPose.target_pose.pose.orientation = p.orientation;
				// Get push direction -> translation direction for next push pose
				tf::Quaternion qt;
				tf::quaternionMsgToTF(goal->push_poses.at(i - 1).orientation,
						qt);
				tf::Matrix3x3 rot;
				rot.setIdentity();
				rot.setRotation(qt);
				tf::Vector3 direction1 = rot.getColumn(0).normalized();
				// Add translation to next push_pose
				tf::Point p2;
				tf::pointMsgToTF(p.position, p2);
				// +Half-base and uncertainty at driving
				tf::Point dest = p2
						+ (goal->push_distance.at(i - 1) - 0.575)
								* rot.getColumn(0);
				// !!! Iterative try to get behind the box for next push
				for (int k = 0; k < 5; k++) {
					tf::quaternionMsgToTF(goal->push_poses.at(i).orientation,
											qt);
					rot.setIdentity();
					rot.setRotation(qt);
					tf::Vector3 pushDirection = rot.getColumn(0).normalized();
					tf::Point cur_goal_pose = dest - (k * 0.25 * pushDirection);

					geometry_msgs::Point mv_dest;
					tf::pointTFToMsg(cur_goal_pose, mv_dest);
					goalNewPushPose.target_pose.pose.position = mv_dest;
					ROS_INFO("Sending goal to move_base.");
					ROS_INFO("Driving to next push pose [%f, %f, %f]...",
						goalNewPushPose.target_pose.pose.position.x,
						goalNewPushPose.target_pose.pose.position.y,
						goalNewPushPose.target_pose.pose.position.z);
				// Compute next push position (farer from originial)
				ac.sendGoal(goalNewPushPose);
				ac.waitForResult();
					if (ac.getState()
						== actionlib::SimpleClientGoalState::SUCCEEDED) {
						ROS_INFO("Driving to next push position completed.");
						remaining_distance += k * 0.25;
						break;
					} else {
						ROS_ERROR("Not able to drive to next push pose.");
						if (k == 4) return;
						else ros::Duration(2).sleep();
					}
				}

			}

			if (goal->push_distance.at(i) - 0.75 < 0.1) {
				ROS_WARN("Push distance smaller then 0.1. Do nothing");
				continue;
			}
			// Swap to base_scan_filtered
			mux_client = nh_.serviceClient<topic_tools::MuxSelect>("base_scan_mux_select");
			topic_tools::MuxSelect sel;
			sel.request.topic = "base_scan_filtered";
			if (mux_client.call(sel)) ROS_INFO("Swap to base_scan_filtered completed.");
			else {
				ROS_INFO("Not able to swap to base_scan_filtered.");
				as_.setPreempted();
				return;
			}
			// Clear cost map
			move_base_client = nh_.serviceClient<std_srvs::Empty>(
					"/move_base/clear_costmaps");
			std_srvs::Empty e;
			if (move_base_client.call(e)) ROS_INFO("Costmap cleared completed");
			else {
				ROS_INFO("Not able to clear costmap.");
				as_.setPreempted();
				return;
			}
			ros::Duration(0.25).sleep();
			while (remaining_distance > 0) {
				move_base_msgs::MoveBaseGoal goalMB2;
				goalMB2.target_pose.header.frame_id = "base_link";
				goalMB2.target_pose.header.stamp = ros::Time::now();
				goalMB2.target_pose.pose.position.x =
						remaining_distance >= 2.8 ? 2.8 : remaining_distance;
				goalMB2.target_pose.pose.orientation.w = 1.0;

				ROS_INFO("Sending goal to move_base.");
				ROS_INFO("Pushing object...");
				ac.sendGoal(goalMB2);
				ac.waitForResult();
				if (ac.getState()
						== actionlib::SimpleClientGoalState::SUCCEEDED) {
					std::ostringstream str_temp;
					str_temp << std::max(remaining_distance - 2.8, 0.0);
					ROS_INFO("Iterative pushing: %s meters remaining...",
							str_temp.str().c_str());
				} else {
					ROS_INFO("Not able to push the object.");
					as_.setPreempted();
					return;
				}
				remaining_distance -= 2.8;
			}
			std::ostringstream str_temp;
			str_temp << goal->push_distance.at(i);
			ROS_INFO("Pushing object (%s meters) completed.",
					str_temp.str().c_str());

			// Move back
			ROS_INFO("Moving back via cmd command...");
			ROS_INFO("Moving back...");
			if (this->dirveStraightOdom(-0.25, 0.75)) ROS_INFO("Moving back completed.");
			else {
				ROS_INFO("Not able to move back.");
				as_.setPreempted();
				return;
			}

			// Swap back to base_scan
			topic_tools::MuxSelect sel2;
			sel2.request.topic = "scan_filtered";
			if (mux_client.call(sel2)) ROS_INFO("Swap to base_scan completed.");
			else {
				ROS_INFO("Not able to swap to base_scan.");
				as_.setPreempted();
				return;
			}
		}
		ROS_INFO("Push action completed.");
		as_.setSucceeded();
	}

};

int main(int argc, char** argv) {
	ros::init(argc, argv, "push_action_server");

	PushAction push(ros::this_node::getName());
	ros::spin();

	return 0;
}
