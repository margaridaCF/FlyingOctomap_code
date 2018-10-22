#include <ros/ros.h>
#include <ltStar_lib_ortho.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/Marker.h>
#include <marker_publishing_utils.h>
#include <std_srvs/Empty.h>

namespace LazyThetaStarOctree
{
    octomap::OcTree* octree;
	double sidelength_lookup_table  [16];
	ros::Publisher ltstar_reply_pub;
	ros::Publisher marker_pub;

	bool octomap_init;
	bool publish_free_corridor_arrows;


	void publishResultingPath(lazy_theta_star_msgs::LTStarReply reply, int series )
	{
		visualization_msgs::MarkerArray waypoint_array;
		visualization_msgs::MarkerArray arrow_array;
		visualization_msgs::Marker marker_temp;
		// Publish to rviz
		for (int i = 0; i < reply.waypoint_amount; ++i)
		{
			octomath::Vector3 candidate (reply.waypoints[i].position.x, reply.waypoints[i].position.y, reply.waypoints[i].position.z);
			std::unordered_set<std::shared_ptr<octomath::Vector3>> neighbors;
	        octomap::OcTreeKey key = octree->coordToKey(candidate);
	        double depth = getNodeDepth_Octomap(key, *octree);
	        double side_length = findSideLenght(octree->getTreeDepth(), depth, sidelength_lookup_table);
	        octomath::Vector3 cell_center = octree->keyToCoord(key, depth);
	        if( cell_center.distance(candidate) < 0.001 )
	        {
		        rviz_interface::build_waypoint(candidate, side_length, (0.3*i)/reply.waypoint_amount, i+series, marker_temp, series);
		        waypoint_array.markers.push_back( marker_temp );
	        }
	        if(i !=0)
	        {

				visualization_msgs::Marker marker_temp;
				octomath::Vector3 prev_candidate (reply.waypoints[i-1].position.x, reply.waypoints[i-1].position.y, reply.waypoints[i-1].position.z);
				rviz_interface::build_arrow_path(candidate, prev_candidate, i, marker_temp, series);
				arrow_array.markers.push_back( marker_temp );
	        }
		}
		marker_pub.publish(arrow_array);
		marker_pub.publish(waypoint_array);
	}
	
	void octomap_callback(const octomap_msgs::Octomap::ConstPtr& octomapBinary){
		delete octree;
		ROS_WARN_STREAM("Got the octomap");
		octree = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*octomapBinary);
	    LazyThetaStarOctree::fillLookupTable(octree->getResolution(), octree->getTreeDepth(), sidelength_lookup_table); 
		octomap_init = true;
	}


}

int main(int argc, char **argv)
{
	LazyThetaStarOctree::publish_free_corridor_arrows = true;
	ros::init(argc, argv, "ltstar_debug_node");
	ros::NodeHandle nh;
	LazyThetaStarOctree::marker_pub = nh.advertise<visualization_msgs::MarkerArray>("ltstar_path", 1);
	// ros::ServiceServer ltstar_status_service = nh.advertiseService("ltstar_status", LazyThetaStarOctree::check_status);
	ros::Subscriber octomap_sub = nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 10, LazyThetaStarOctree::octomap_callback);
	ros::Subscriber ltstar_benchmark_sub = nh.subscribe<lazy_theta_star_msgs::LTStarBenchmarkRequest>("ltstar_request_benchmark", 10, LazyThetaStarOctree::ltstar_benchmark_callback);
	LazyThetaStarOctree::ltstar_reply_pub = nh.advertise<lazy_theta_star_msgs::LTStarReply>("ltstar_reply", 10);

	// ros::Subscriber ltstars_sub = nh.subscribe<lazy_theta_star_msgs::LTStarRequest>("ltstar_request", 10, LazyThetaStarOctree::ltstar_callback);

	// octomap::OcTree octree ("/ros_ws/src/lazy_theta_star/test/data/d.bt");
	// LazyThetaStarOctree::ltstar_callback(request);
  	ros::spin();
}
