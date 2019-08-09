#ifndef RVIZ_INTERFACE_H
#define RVIZ_INTERFACE_H


#include <ros/ros.h>
#include <octomap/math/Vector3.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <unordered_set>
#include <sstream>



namespace rviz_interface
{

	class PublishingInput
	{
	public:
		visualization_msgs::MarkerArray waypoint_array;
		ros::Publisher const& marker_pub;
		const bool publish;
		const std::string dataset_name;
		PublishingInput(ros::Publisher const& marker_pub, bool publish = false, std::string dataset_name = "unamed", visualization_msgs::MarkerArray waypoint_array = visualization_msgs::MarkerArray())
			: marker_pub(marker_pub), dataset_name(dataset_name), publish(publish), waypoint_array(waypoint_array)
		{}
	};

	// GEOFENCES
	void publish_geofence 			(octomath::Vector3 const& geofence_min, octomath::Vector3 const& geofence_max, visualization_msgs::MarkerArray & marker_array);
	void publish_safety_margin 		(geometry_msgs::Point const& frontier, double safety_margin, visualization_msgs::MarkerArray marker_array, int id);
	void publish_markerArray_safety_margin(geometry_msgs::Point const& frontier, double safety_margin, ros::Publisher const& marker_pub, int id);
    void build_geofence (octomath::Vector3 const& geofence_min, octomath::Vector3 const& geofence_max, visualization_msgs::Marker & marker, int id, std::string ns, double red, double green, double blue);
	
	// ARROWS
	void build_arrow_path 			(octomath::Vector3 & start, octomath::Vector3 & goal, int request_id, visualization_msgs::Marker & marker, int series = 9, std::string ns = "lazy_theta_star_path");
    void publish_arrow_path_visibility  (octomath::Vector3 const& start, octomath::Vector3 const& goal, ros::Publisher const& marker_pub, bool free, int id);
	void publish_arrow_path_occupancyState (octomath::Vector3 const& start, octomath::Vector3 const& goal, visualization_msgs::MarkerArray & marker_array, bool free, int id);
	void publish_arrow_path_unreachable	(octomath::Vector3 const& start, octomath::Vector3 const& goal, ros::Publisher const& marker_pub, int id);
	void publish_arrow_path_father		(octomath::Vector3 const& start, octomath::Vector3 const& goal, ros::Publisher const& marker_pub);
	void publish_arrow_corridor 		(octomath::Vector3 const& start, octomath::Vector3 const& goal, ros::Publisher const& marker_pub);
	void publish_arrow_corridor_center	(octomath::Vector3 const& start, octomath::Vector3 const& goal, ros::Publisher const& marker_pub);
	void publish_arrow_straight_line	(geometry_msgs::Point const& start, geometry_msgs::Point const& goal, ros::Publisher const& marker_pub, bool found_safe_alternative, int id = 9);
    void build_arrow_type				(octomath::Vector3 const& start, octomath::Vector3 const& goal, visualization_msgs::MarkerArray & marker_array, int id, bool occupied);

	// POINTS
	void publish_frontier_marker 	(octomath::Vector3 const& candidate, bool is_frontier, ros::Publisher const& marker_pub);
	void publish_frontier_marker 	(geometry_msgs::Point const& candidate, bool is_frontier, ros::Publisher const& marker_pub);
	void publish_voxel_free_occupied(octomath::Vector3 & candidate, bool is_occupied, ros::Publisher const& marker_pub, int id, double size, visualization_msgs::Marker & marker);
	void init_point 				(geometry_msgs::Point & point, float x, float y, float z);
	void build_waypoint 			(octomath::Vector3 & candidate, double size, int color, int waypoint_id, visualization_msgs::Marker & marker, int series = 9);
	void publish_current_position 	(octomath::Vector3 & candidate, visualization_msgs::MarkerArray marker_array);
	void publish_start 				(geometry_msgs::Point const& candidate, visualization_msgs::MarkerArray & marker_array);
    void publish_startSafetyZone	(geometry_msgs::Point const& candidate, visualization_msgs::MarkerArray & marker_array, double diameter);
    void build_safetyzone_flybyStart(geometry_msgs::Point const& candidate, visualization_msgs::MarkerArray & marker_array, double diameter);
    void build_safetyzone_flybyEnd  (geometry_msgs::Point const& candidate, visualization_msgs::MarkerArray & marker_array, double diameter);
    void build_safetyzone_unknown	(geometry_msgs::Point const& candidate, visualization_msgs::MarkerArray & marker_array, double diameter);
	void publish_goal 				(geometry_msgs::Point const& candidate, visualization_msgs::MarkerArray & marker_array);
    void publish_goalSafetyZone		(geometry_msgs::Point const& candidate, visualization_msgs::MarkerArray & marker_array, double diameter);
	void publish_random_important_cube(octomath::Vector3 const& candidate_vec3, ros::Publisher const& marker_pub);
	void publish_s 					(geometry_msgs::Point const& candidate, ros::Publisher const& marker_pub, visualization_msgs::MarkerArray & marker_array, int id, float size);
    void publish_rejected_neighbor  (geometry_msgs::Point const& candidate, ros::Publisher const& marker_pub, visualization_msgs::MarkerArray & marker_array, int id, float size);
	void publish_visible_neighbor	(geometry_msgs::Point const& candidate, ros::Publisher const& marker_pub, visualization_msgs::MarkerArray & marker_array, int id, float size);
	void publish_closed				(octomath::Vector3 const& candidate_vec3, ros::Publisher const& marker_pub, visualization_msgs::MarkerArray & marker_array, int id);
	void publish_sensing_position 	(octomath::Vector3 const& position, int id, visualization_msgs::MarkerArray & marker_array);
	void publish_start_voxel 		(geometry_msgs::Point const& candidate, visualization_msgs::MarkerArray & marker_array, double size);
	void publish_goal_voxel 		(geometry_msgs::Point const& candidate, visualization_msgs::MarkerArray & marker_array, double size);
    void build_small_marker			(octomath::Vector3 const& candidate, visualization_msgs::Marker & marker, float red, float green, float blue, std::string ns, int id, double size = 0.2f, double alpha = 1);

	// SPHERES
	void build_sphere(octomath::Vector3 & candidate, double size, int green_base, int marker_id, visualization_msgs::Marker & marker, int red_base, std::string ns);
    void build_stateManager(geometry_msgs::Point const& frontier,geometry_msgs::Point const& oppairStart, geometry_msgs::Point const& oppairEnd, geometry_msgs::Point const& start,visualization_msgs::MarkerArray & marker_array, double diameter);
    void build_sphere_basic(geometry_msgs::Point const& candidate, visualization_msgs::MarkerArray & marker_array, std::string ns, double red, double green, double blue, int oppair_id = 30, double alpha = 1, double diameter = 0.2 );
    void build_startOPP_outsideGeofence(geometry_msgs::Point const& oppairStart, visualization_msgs::MarkerArray & marker_array, int oppair_id);
    void build_endOPP_outsideGeofence(geometry_msgs::Point const& oppairEnd, visualization_msgs::MarkerArray & marker_array, int oppair_id);

    


    // OTHER
	void publish_deleteAll  		(ros::Publisher const& marker_pub);
	// void build_neighbor_array 		(std::unordered_set<std::shared_ptr<octomath::Vector3>> & neighbors, visualization_msgs::MarkerArray & marker_array);
	visualization_msgs::Marker createEmptyLineStrip(int id);
}


#endif // RVIZ_INTERFACE_H