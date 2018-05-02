#include <ros/ros.h>
#include <frontiers.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <frontiers_msgs/FrontierNodeStatus.h>
#include <frontiers_msgs/CheckIsFrontier.h>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/Point.h>

#define SAVE_CSV 1

namespace frontiers_async_node
{
	octomap::OcTree* octree;
	ros::Publisher local_pos_pub;
	ros::Publisher marker_pub;
#ifdef SAVE_CSV
	std::ofstream log;
#endif
		
	bool octomap_init;

	bool check_status(frontiers_msgs::FrontierNodeStatus::Request  &req,
        frontiers_msgs::FrontierNodeStatus::Response &res)
	{
		res.is_accepting_requests = octomap_init;
	  	return true;
	}

	bool check_frontier(frontiers_msgs::CheckIsFrontier::Request  &req,
		frontiers_msgs::CheckIsFrontier::Response &res)
	{
		octomath::Vector3 candidate(req.candidate.x, req.candidate.y, req.candidate.z);
		res.is_frontier = Frontiers::isFrontier(*octree, candidate);
		return true;
	}

	void frontier_callback(const frontiers_msgs::FrontierRequest::ConstPtr& frontier_request)
	{
		frontiers_msgs::FrontierReply reply;
		if(octomap_init)
		{
#ifdef SAVE_CSV
				std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
#endif
			Frontiers::processFrontiersRequest(*octree, *frontier_request, reply, marker_pub);

#ifdef SAVE_CSV
				std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
				std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
				std::chrono::milliseconds millis = std::chrono::duration_cast<std::chrono::milliseconds>(time_span);
				std::chrono::seconds seconds = std::chrono::duration_cast<std::chrono::seconds>(time_span);
				log << millis.count() << ", " << seconds.count() << "\n";
#endif

			if(reply.frontiers_found == 0)
	        {
	            ROS_INFO_STREAM("[Frontiers] No frontiers could be found. Writing tree to file. Request was " << *frontier_request);
	            octree->writeBinary("/ros_ws/src/data/octree_noFrontiers.bt"); 
	        }
		}
		else
		{
			reply.success=false;
			reply.request_id = frontier_request->header.seq;
			reply.frontiers_found = 0;
		}
		local_pos_pub.publish(reply);
	}

	void octomap_callback(const octomap_msgs::Octomap::ConstPtr& octomapBinary){
		delete octree;
		octree = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*octomapBinary);
		octomap_init = true;
	}
}

int main(int argc, char **argv)
{
#ifdef SAVE_CSV
		frontiers_async_node::log.open ("/ros_ws/src/data/frontiers_computation_time.csv");
		frontiers_async_node::log << "computation_time_millis, computation_time_secs \n";
#endif

	ros::init(argc, argv, "frontier_node_async");
	ros::NodeHandle nh;
	ros::ServiceServer frontier_status_service = nh.advertiseService("frontier_status", frontiers_async_node::check_status);
	ros::ServiceServer is_frontier_service = nh.advertiseService("is_frontier", frontiers_async_node::check_frontier);
	ros::Subscriber octomap_sub = nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 10, frontiers_async_node::octomap_callback);
	ros::Subscriber frontiers_sub = nh.subscribe<frontiers_msgs::FrontierRequest>("frontiers_request", 10, frontiers_async_node::frontier_callback);
	frontiers_async_node::local_pos_pub = nh.advertise<frontiers_msgs::FrontierReply>("frontiers_reply", 10);
	frontiers_async_node::marker_pub = nh.advertise<visualization_msgs::MarkerArray>("frontiers/known_space", 1);

	ros::spin();
}
