#include <observation_maneuver.h>
#include <observation_maneuver_msgs/OPPairsReply.h>
#include <observation_maneuver_msgs/OPPairsRequest.h>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <octomap/math/Vector3.h>
#include <marker_publishing_utils.h>


namespace observation_node
{
	double distance_behind, distance_inFront;
	ros::Publisher marker_pub;

	void oppairs_callback(const observation_maneuver_msgs::OPPairsRequest::ConstPtr& opp_request)
	{
		rviz_interface::publish_deleteAll(marker_pub);
		visualization_msgs::MarkerArray waypoint_array;
		visualization_msgs::Marker marker;
		double size = 0.1f;
		int green_base = 0; 
		int red_base = opp_request->header.seq;
		// Uav position
		octomath::Vector3 uav_position_octoVec (opp_request->uav_position.x,  opp_request->uav_position.y,  opp_request->uav_position.z);
		marker = visualization_msgs::Marker();
		rviz_interface::build_sphere(uav_position_octoVec, size, green_base, 0, marker, red_base, "uav_position");
		waypoint_array.markers.push_back( marker );
		// Frontier
		octomath::Vector3 frontier_octoVec (opp_request->frontier.x,  opp_request->frontier.y,  opp_request->frontier.z);
		marker = visualization_msgs::Marker();
		rviz_interface::build_sphere(frontier_octoVec, size, green_base, 0, marker, red_base, "frontier");
		waypoint_array.markers.push_back( marker );
		
		Eigen::Vector3d uav_position( opp_request->uav_position.x,  opp_request->uav_position.y,  opp_request->uav_position.z);
		Eigen::Vector3d frontier    ( opp_request->frontier.x, opp_request->frontier.y, opp_request->frontier.z);
		double distance_behind = opp_request->distance_behind;
		double distance_inFront = opp_request->distance_inFront;
		int circle_divisions = opp_request->circle_divisions;
		observation_lib::OPPairs oppairs (circle_divisions, opp_request->distance_toTarget, distance_inFront, distance_behind);

		rviz_interface::PublishingInput pi (marker_pub, true, "test_node", waypoint_array);
		oppairs.NewFrontier(frontier, uav_position, pi);
		observation_lib::OPPair nextOPPair;
		while(oppairs.NextOPPair(nextOPPair, pi)){}
		marker_pub.publish(waypoint_array);
	}

}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "observation_node");
	ros::NodeHandle nh;

	// nh.getParam("observation/distance_behind", observation_node::distance_behind);
	// nh.getParam("observation/distance_inFront", observation_node::distance_inFront);

	observation_node::marker_pub = nh.advertise<visualization_msgs::MarkerArray>("observation_info", 1);
	ros::Subscriber opp_generation_sub = nh.subscribe<observation_maneuver_msgs::OPPairsRequest>("opp_request", 10, observation_node::oppairs_callback);


	ros::spin();
}