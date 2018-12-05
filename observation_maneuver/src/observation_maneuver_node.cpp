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

		// Eigen::MatrixXd circle_pointCloud (3, circle_divisions);
		// observation_lib::generateCirclePoints(circle_divisions, circle_pointCloud);


		Eigen::MatrixXd starts_zero (3, circle_divisions);
		Eigen::MatrixXd ends_zero (3, circle_divisions);
		Eigen::MatrixXd directions_zero (3, circle_divisions);
		observation_lib::precalculation (opp_request->distance_toTarget, circle_divisions, distance_inFront, distance_behind, starts_zero, ends_zero, directions_zero);


		for (int i = 0; i < circle_divisions; ++i)
		{
			// Eigen::Vector3d observationStart = observation_lib::calculatePointTranslation(circle_pointCloud.col(i), frontier, uav_position, distance_behind, observation_lib::calculateTrigStart);
			// Eigen::Vector3d observationEnd   = observation_lib::calculatePointTranslation(circle_pointCloud.col(i), frontier, uav_position, distance_inFront, observation_lib::calculateTrigEnd);
			// Circle point
			int marker_id = i;
			// octomath::Vector3 trig_circle_point_octoVec(circle_pointCloud.col(i)(0), circle_pointCloud.col(i)(1), circle_pointCloud.col(i)(2));
   //  		rviz_interface::build_sphere(trig_circle_point_octoVec, size, green_base, marker_id, marker, red_base, "trig_circle_point");
			// waypoint_array.markers.push_back( marker );
			// Testing direction (arrow)
			octomath::Vector3 observationStart_octoVec (starts_zero(0, i),  starts_zero(1, i),  starts_zero(2, i));
			octomath::Vector3 observationEnd_octoVec   (ends_zero(0, i),    ends_zero(1, i),    ends_zero(2, i));


			marker = visualization_msgs::Marker();
			marker_id = i;
    		rviz_interface::build_arrow_path(observationStart_octoVec, observationEnd_octoVec, 100+marker_id, marker, red_base, "testing_direction");
			waypoint_array.markers.push_back( marker );
		}
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