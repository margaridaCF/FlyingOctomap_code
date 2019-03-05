#include <ros/ros.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include <next_best_view.h>
#include <frontiers.h>
#include <frontiers_msgs/FrontierNodeStatus.h>
#include <frontiers_msgs/CheckIsFrontier.h>
// RAM
#include "sys/types.h"
#include "sys/sysinfo.h"

#define SAVE_CSV 1

namespace nbv_node
{
	octomap::OcTree* octree;
	ros::Publisher marker_pub;
	ros::Publisher local_pos_pub;
	std::string folder_name;
    NextBestView::NextBestViewSM nbv_state_machine;

	#ifdef SAVE_CSV
	struct sysinfo memInfo;
	std::ofstream log;
	std::ofstream volume_explored;
	std::chrono::high_resolution_clock::time_point start_exploration;
	#endif

	bool octomap_init;

	double calculate_volume_explored(octomath::Vector3 const& min, octomath::Vector3 const& max)
	{
        int frontiers_count = 0;
		octomap::OcTreeKey bbxMinKey, bbxMaxKey;
        if(!octree->coordToKeyChecked(min, bbxMinKey) || !octree->coordToKeyChecked(max, bbxMaxKey))
        {
            ROS_ERROR_STREAM("[Frontiers] Problems with write_volume_explored_to_csv");
        	return 0;
        }
		octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(bbxMinKey,bbxMaxKey);
		double volume = 0;
		while( !(it == octree->end_leafs_bbx()) )
		{
			volume += pow(it.getSize(), 3);
            it++;
		}
		return volume;
	}

	bool check_status(frontiers_msgs::FrontierNodeStatus::Request  &req,
        frontiers_msgs::FrontierNodeStatus::Response &res)
	{
		res.is_accepting_requests = octomap_init;
	  	return true;
	}

	void frontier_callback(const frontiers_msgs::FrontierRequest::ConstPtr& frontier_request)
	{

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
	nbv_node::folder_name = "~/Flying_Octomap_code/src/data/current";
	nbv_node::log.open (nbv_node::folder_name + "/nbv_computation_time.csv", std::ofstream::app);
	nbv_node::log << "computation_time_millis, computation_time_secs \n";
	nbv_node::log.close();
	nbv_node::volume_explored.open (nbv_node::folder_name + "/volume_explored.csv", std::ofstream::app);
	nbv_node::volume_explored << "time ellapsed minutes,volume,RAM\n";
	nbv_node::volume_explored.close();
	nbv_node::start_exploration = std::chrono::high_resolution_clock::now();
	#endif

	ros::init(argc, argv, "next_best_view_node");
	ros::NodeHandle nh;

	ros::ServiceServer frontier_status_service = nh.advertiseService("frontier_status", nbv_node::check_status);
	ros::Subscriber octomap_sub = nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 10, nbv_node::octomap_callback);
	ros::Subscriber frontiers_sub = nh.subscribe<frontiers_msgs::FrontierRequest>("frontiers_request", 10, nbv_node::frontier_callback);
	nbv_node::local_pos_pub = nh.advertise<frontiers_msgs::FrontierReply>("frontiers_reply", 10);
	nbv_node::marker_pub = nh.advertise<visualization_msgs::MarkerArray>("frontiers/next_best_view", 1);

	ros::spin();
}
