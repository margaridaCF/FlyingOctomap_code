#include <ros/ros.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include <next_best_view.h>
#include <frontiers.h>
#include <frontiers_msgs/FrontierNodeStatus.h>
#include <frontiers_msgs/CheckIsFrontier.h>

#define SAVE_CSV 1

namespace nbv_node
{
	octomap::OcTree* octree;
	octomap::OcTree* octree_inUse;
	ros::Publisher marker_pub;
	ros::Publisher local_pos_pub;
	std::string folder_name;
    NextBestView::NextBestViewSM nbv_state_machine;

	#ifdef SAVE_CSV
	std::ofstream log;
	std::chrono::high_resolution_clock::time_point start_exploration;
	#endif

	bool octomap_init;

	bool check_status(frontiers_msgs::FrontierNodeStatus::Request  &req,
        frontiers_msgs::FrontierNodeStatus::Response &res)
	{
		res.is_accepting_requests = octomap_init;
	  	return true;
	}

	void frontier_callback(const frontiers_msgs::FrontierRequest::ConstPtr& frontier_request)
	{

		if (frontier_request.new_request)
		{
			delete octree_inUse;
			octree_inUse = octree;
			nbv_state_machine.NewRequest(&octree_inUse, frontier_request.request_number);
		}
		else
		{
			std::vector<observation_lib::OPPair> oppairs;
			nbv_state_machine.FindNextOPPairs(frontier_request.frontier_amount, oppairs, frontier_request.request_number);
		}
	}

	void octomap_callback(const octomap_msgs::Octomap::ConstPtr& octomapBinary){
		delete octree;
		octree = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*octomapBinary);
		nbv_state_machine = NextBestView::NextBestViewSM();
		octomap_init = true;
	}

	void init_variables(ros::NodeHandle nh)
	{
		double distance_inFront,  distance_behind, circle_divisions,  frontier_safety_margin;
        // Goal state machine
        nh.getParam("oppairs/distance_inFront", distance_inFront);
        nh.getParam("oppairs/distance_behind",  distance_behind);
        nh.getParam("oppairs/circle_divisions",  circle_divisions);
        nh.getParam("frontier/safety_margin", frontier_safety_margin);
    	nbv_state_machine( distance_inFront,  distance_behind, circle_divisions,  frontier_safety_margin);	
	}
}

int main(int argc, char **argv)
{
	#ifdef SAVE_CSV
	nbv_node::folder_name = "~/Flying_Octomap_code/src/data/current";
	nbv_node::log.open (nbv_node::folder_name + "/nbv_computation_time.csv", std::ofstream::app);
	nbv_node::log << "computation_time_millis, computation_time_secs \n";
	nbv_node::log.close();
	nbv_node::start_exploration = std::chrono::high_resolution_clock::now();
	#endif

	ros::init(argc, argv, "next_best_view_node");
	ros::NodeHandle nh;
	init_variables(nh);
	ros::ServiceServer frontier_status_service = nh.advertiseService("frontier_status", nbv_node::check_status);
	ros::Subscriber octomap_sub = nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 10, nbv_node::octomap_callback);
	ros::Subscriber frontiers_sub = nh.subscribe<frontiers_msgs::FrontierRequest>("frontiers_request", 10, nbv_node::frontier_callback);
	nbv_node::local_pos_pub = nh.advertise<frontiers_msgs::FrontierReply>("frontiers_reply", 10);
	nbv_node::marker_pub = nh.advertise<visualization_msgs::MarkerArray>("frontiers/next_best_view", 1);

	ros::spin();
}
