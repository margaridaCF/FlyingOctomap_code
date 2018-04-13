#ifndef RVIZ_INTERFACE_H
#define RVIZ_INTERFACE_H


#include <ros/ros.h>
#include <octomap/math/Vector3.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

namespace rviz_interface
{

	void publish_geofence(octomath::Vector3 const& geofence_min, octomath::Vector3 const& geofence_max, ros::Publisher const& marker_pub);
	void init_point(geometry_msgs::Point & point, float x, float y, float z);
	void push_segment(visualization_msgs::Marker & marker, geometry_msgs::Point & start, geometry_msgs::Point & end);
	void publish_marker_safety_margin(geometry_msgs::Point const& frontier, double safety_margin, ros::Publisher const& marker_pub, int id);
	void publish_deleteAll(ros::Publisher const& marker_pub);
	void publish_marker(octomath::Vector3 & candidate, bool is_occupied, ros::Publisher const& marker_pub, int id, double size);
}


#endif // RVIZ_INTERFACE_H