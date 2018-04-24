#ifndef RVIZ_INTERFACE_H
#define RVIZ_INTERFACE_H


#include <ros/ros.h>
#include <octomap/math/Vector3.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace rviz_interface
{

	void publish_geofence(octomath::Vector3 const& geofence_min, octomath::Vector3 const& geofence_max, ros::Publisher const& marker_pub);
	void publish_marker_safety_margin(geometry_msgs::Point const& frontier, double safety_margin, ros::Publisher const& marker_pub, int id);
	void init_point(geometry_msgs::Point & point, float x, float y, float z);
	void publish_deleteAll(ros::Publisher const& marker_pub);
	void publish_frontier_marker(octomath::Vector3 const& candidate, bool is_frontier, ros::Publisher const& marker_pub);
	void publish_frontier_marker(geometry_msgs::Point const& candidate, bool is_frontier, ros::Publisher const& marker_pub);
	void publish_voxel_free_occupied(octomath::Vector3 & candidate, bool is_occupied, ros::Publisher const& marker_pub, int id, double size, visualization_msgs::Marker & marker);
	void build_arrow_path(octomath::Vector3 & start, octomath::Vector3 & goal, int request_id, visualization_msgs::Marker & marker);
	void build_waypoint(octomath::Vector3 & candidate, double size, int color, int waypoint_id, visualization_msgs::Marker & marker);
	void publish_current_position(octomath::Vector3 & candidate, ros::Publisher const& marker_pub);
	void publish_start(geometry_msgs::Point const& candidate, ros::Publisher const& marker_pub);
	void publish_goal(geometry_msgs::Point const& candidate, ros::Publisher const& marker_pub);
	void publish_sensing_position(octomath::Vector3 const& position, ros::Publisher const& marker_pub);
}


#endif // RVIZ_INTERFACE_H