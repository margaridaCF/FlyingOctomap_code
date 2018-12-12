#include <ros/ros.h>
#include <chrono>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <architecture_msgs/PositionMiddleMan.h>

#include <marker_publishing_utils.h>
#include <visualization_msgs/Marker.h>


namespace current_position_provider_node
{
	geometry_msgs::Point current_position;
	bool current_position_init;

    ros::Publisher marker_pub;
    visualization_msgs::Marker marker;
    std::chrono::high_resolution_clock::time_point start;

	void updatePositionMarker()
	{
	    marker.points.push_back(current_position);
	    marker_pub.publish( marker );
	}

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
			ROS_ERROR_STREAM("[Position Middle Man] No position received, please try later.");
			return false;
		}
	}

	void ground_truth_cb(const geometry_msgs::PoseStamped::ConstPtr& new_odometry )
	{
		current_position = new_odometry->pose.position;
		current_position_init = true;

		std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(now - start);
		std::chrono::seconds seconds = std::chrono::duration_cast<std::chrono::seconds>(time_span);
		if( (seconds.count() % 2) == 0)
		{
			updatePositionMarker();
		}

	}
}

int main(int argc, char **argv)
{

	current_position_provider_node::current_position_init = false;
	ros::init(argc, argv, "current_position_provider");
	ros::NodeHandle nh;

	std::string current_position_topic;
	nh.getParam("current_position_topic", current_position_topic);
	
	ros::ServiceServer service = nh.advertiseService("get_current_position", current_position_provider_node::get_current_position);
	ros::Subscriber ground_truth_sub = nh.subscribe<geometry_msgs::PoseStamped>(current_position_topic, 1, current_position_provider_node::ground_truth_cb);

    current_position_provider_node::marker_pub = nh.advertise<visualization_msgs::Marker>("position_log", 1);
	current_position_provider_node::marker = rviz_interface::createEmptyLineStrip(30);
	current_position_provider_node::start = std::chrono::high_resolution_clock::now();

	
	ros::spin();
}