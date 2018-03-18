#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <architecture_msgs/PositionMiddleMan.h>

namespace current_position_provider_node
{
	geometry_msgs::Point current_position;
	bool current_position_init;

	bool get_current_position(architecture_msgs::PositionMiddleMan::Request &req,
		architecture_msgs::PositionMiddleMan::Response &res)
	{
		if(current_position_init)
		{
			res.current_position = current_position;
			// ROS_INFO_STREAM("[Position Middle Man] All good,sending position.");
			return true;
		}
		else
		{
			// ROS_ERROR_STREAM("[Position Middle Man] No position received, please try later.");
			return false;
		}
	}

	void ground_truth_cb(const geometry_msgs::PoseStamped::ConstPtr& new_odometry )
	{
		current_position = new_odometry->pose.position;
		current_position_init = true;
	}
}

int main(int argc, char **argv)
{
	current_position_provider_node::current_position_init = false;
	ros::init(argc, argv, "current_position_provider");
	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService("get_current_position", current_position_provider_node::get_current_position);
	ros::Subscriber ground_truth_sub = nh.subscribe<geometry_msgs::PoseStamped>("/ground_truth_to_tf/pose", 1, current_position_provider_node::ground_truth_cb);
	
	ros::spin();
}