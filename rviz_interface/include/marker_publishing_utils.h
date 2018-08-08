#ifndef RVIZ_INTERFACE_H
#define RVIZ_INTERFACE_H


#include <ros/ros.h>
#include <octomap/math/Vector3.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <unordered_set>

namespace rviz_interface
{
	// GEOFENCES
	void publish_geofence 			(octomath::Vector3 const& geofence_min, octomath::Vector3 const& geofence_max, ros::Publisher const& marker_pub);
	void publish_safety_margin 		(geometry_msgs::Point const& frontier, double safety_margin, ros::Publisher const& marker_pub, int id);
	void publish_markerArray_safety_margin(geometry_msgs::Point const& frontier, double safety_margin, ros::Publisher const& marker_pub, int id);
	// ARROWS
	void build_arrow_path 			(octomath::Vector3 & start, octomath::Vector3 & goal, int request_id, visualization_msgs::Marker & marker, int series = 9);
	void publish_arrow_path_occupancyState(octomath::Vector3 const& start, octomath::Vector3 const& goal, ros::Publisher const& marker_pub, bool free);
	void publish_arrow_path_unreachable(octomath::Vector3 const& start, octomath::Vector3 const& goal, ros::Publisher const& marker_pub, int id);
	void publish_arrow_path_father	(octomath::Vector3 const& start, octomath::Vector3 const& goal, ros::Publisher const& marker_pub);
	void publish_arrow_corridor 	(octomath::Vector3 const& start, octomath::Vector3 const& goal, ros::Publisher const& marker_pub);
	void publish_arrow_corridor_center(octomath::Vector3 const& start, octomath::Vector3 const& goal, ros::Publisher const& marker_pub);
	void publish_arrow_straight_line(geometry_msgs::Point const& start, geometry_msgs::Point const& goal, ros::Publisher const& marker_pub, bool found_safe_alternative);
	// POINTS
	void publish_frontier_marker 	(octomath::Vector3 const& candidate, bool is_frontier, ros::Publisher const& marker_pub);
	void publish_frontier_marker 	(geometry_msgs::Point const& candidate, bool is_frontier, ros::Publisher const& marker_pub);
	void publish_voxel_free_occupied(octomath::Vector3 & candidate, bool is_occupied, ros::Publisher const& marker_pub, int id, double size, visualization_msgs::Marker & marker);
	void init_point 				(geometry_msgs::Point & point, float x, float y, float z);
	void build_waypoint 			(octomath::Vector3 & candidate, double size, int color, int waypoint_id, visualization_msgs::Marker & marker, int series = 9);
	void publish_current_position 	(octomath::Vector3 & candidate, ros::Publisher const& marker_pub);
	void publish_start 				(geometry_msgs::Point const& candidate, ros::Publisher const& marker_pub);
	void publish_goal 				(geometry_msgs::Point const& candidate, ros::Publisher const& marker_pub);
	void publish_random_important_cube(octomath::Vector3 const& candidate_vec3, ros::Publisher const& marker_pub);
	void publish_s 					(geometry_msgs::Point const& candidate, ros::Publisher const& marker_pub);
	void publish_visible_neighbor	(octomath::Vector3 const& candidate_vec3, ros::Publisher const& marker_pub);
	void publish_closed				(octomath::Vector3 const& candidate_vec3, ros::Publisher const& marker_pub);
	void publish_sensing_position 	(octomath::Vector3 const& position, ros::Publisher const& marker_pub);
	void publish_start_voxel 		(geometry_msgs::Point const& candidate, ros::Publisher const& marker_pub, double size);
	void publish_goal_voxel 		(geometry_msgs::Point const& candidate, ros::Publisher const& marker_pub, double size);
    // OTHER
	void publish_deleteAll  		(ros::Publisher const& marker_pub);
	void build_neighbor_array 		(std::unordered_set<std::shared_ptr<octomath::Vector3>> & neighbors, visualization_msgs::MarkerArray & marker_array);
	visualization_msgs::Marker createEmptyLineStrip(int id);
}


#endif // RVIZ_INTERFACE_H