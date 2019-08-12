#include <ros/ros.h>
#include <frontiers.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <frontiers_msgs/FrontierNodeStatus.h>
#include <frontiers_msgs/CheckIsFrontier.h>
#include <frontiers_msgs/CheckIsExplored.h>
#include <visualization_msgs/MarkerArray.h>

#include <atomic>

#include <geometry_msgs/Point.h>
#include <frontiers_common.h>
#include <volume.h>
// RAM
#include "sys/types.h"
#include "sys/sysinfo.h"

#define SAVE_CSV 1

namespace frontiers_async_node
{
	octomap::OcTree* octree;
	octomap::OcTree* octree_inUse;
	std::atomic<bool> is_octree_inUse;
	std::atomic<bool> new_octree;

	ros::Publisher local_pos_pub;
	ros::Publisher marker_pub;
	std::string folder_name;
	int last_request_id;
	octomap::OcTree::leaf_bbx_iterator iterator;
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

	bool check_frontier(frontiers_msgs::CheckIsFrontier::Request  &req,
		frontiers_msgs::CheckIsFrontier::Response &res)
	{
		octomath::Vector3 candidate(req.candidate.x, req.candidate.y, req.candidate.z);
		try
		{
			res.is_frontier = Frontiers::isFrontier(*octree, candidate);
			return true;
		}
		catch(const std::out_of_range& oor)
		{
			ROS_ERROR_STREAM("[Frontiers] Candidate " << req.candidate << " is in unknown space.");
		}
	}

	bool check_unknown(frontiers_msgs::CheckIsExplored::Request  &req,
		frontiers_msgs::CheckIsExplored::Response &res)
	{
		octomath::Vector3 candidate(req.candidate.x, req.candidate.y, req.candidate.z);
		try
		{ 
			res.is_explored = Frontiers:: isExplored(candidate, *octree); 
			return true;
		}
		catch(const std::out_of_range& oor)
		{
			ROS_ERROR_STREAM("[Frontiers] [check_unknown] Candidate " << req.candidate << " std::out_of_range& oor.");
			return true;
		}
	}

	bool find_frontiers(frontiers_msgs::FindFrontiers::Request  &req,
		frontiers_msgs::FindFrontiers::Response &reply)
	{
		if(octomap_init)
		{
			#ifdef SAVE_CSV
			std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
			#endif
			if (req.new_request)
			{
				if (new_octree){
					delete octree_inUse;
					is_octree_inUse = true;
					octree_inUse = octree;
					new_octree = false;
				}
				iterator = Frontiers::processFrontiersRequest(*octree_inUse, req, reply, marker_pub);
				last_request_id = req.request_number;
			}
			else
			{
				octree->writeBinary(folder_name + "/current/octree_requestNumberOutOfSync.bt"); 
				if(last_request_id != req.request_id)
				{
					ROS_ERROR_STREAM("[Frontiers] Request number out of sync. Asked to continue search but current request number is " << last_request_id << " while in request message the id is " << req.request_id);
					reply.success=false;
					reply.frontiers_found = 0;
				}
				else
				{
					// ROS_INFO_STREAM("[Frontiers] Old map");
					Frontiers::searchFrontier(*octree_inUse, iterator, req, reply, marker_pub, true);
				}
			}

			#ifdef SAVE_CSV
			// Frontier computation time
			log.open (folder_name +"/current/frontiers_computation_time.csv", std::ofstream::app);
			std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
			std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
			std::chrono::milliseconds millis = std::chrono::duration_cast<std::chrono::milliseconds>(time_span);
			std::chrono::seconds seconds = std::chrono::duration_cast<std::chrono::seconds>(time_span);
			log << millis.count() << ", " << seconds.count() << "\n";
			log.close();
			// Explored volume
			volume_explored.open (folder_name + "/current/volume_explored.csv", std::ofstream::app);
			std::chrono::duration<double> ellapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>(end - start_exploration);
			std::chrono::milliseconds ellapsed_time_millis = std::chrono::duration_cast<std::chrono::milliseconds>(ellapsed_time);
			double resolution = octree->getResolution();
	        octomath::Vector3  max = octomath::Vector3(req.max.x-resolution, req.max.y-resolution, req.max.z-resolution);
	        octomath::Vector3  min = octomath::Vector3(req.min.x+resolution, req.min.y+resolution, req.min.z+resolution);
			double explored_volume_meters = volume::calculateVolume(*octree, min, max);
			volume_explored << ellapsed_time_millis.count() / 1000  << ", " << explored_volume_meters << std::endl;
			volume_explored.close();
			#endif

			if(reply.frontiers_found == 0 && reply.success)
	        {
	            ROS_INFO_STREAM("[Frontiers] No frontiers could be found. Writing tree to file. Request was " << req);
	            octree->writeBinary(folder_name + "/current/octree_noFrontiers.bt"); 
	        }
		}
		else
		{
			reply.success=false;
			reply.frontiers_found = 0;
		}
		return true;
	}

	void octomap_callback(const octomap_msgs::Octomap::ConstPtr& octomapBinary){
		if (!is_octree_inUse)
		{
			delete octree;
		}
		is_octree_inUse = false;
		new_octree = true;
		octree = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*octomapBinary);
		octomap_init = true;
	}
}

int main(int argc, char **argv)
{
#ifdef SAVE_CSV
    	std::stringstream aux_envvar_home (std::getenv("HOME"));
		frontiers_async_node::folder_name = aux_envvar_home.str() + "/Flying_Octomap_code/src/data";
		frontiers_async_node::log.open (frontiers_async_node::folder_name + "/current/frontiers_computation_time.csv", std::ofstream::app);
		frontiers_async_node::log << "computation_time_millis, computation_time_secs \n";
		frontiers_async_node::log.close();
		frontiers_async_node::volume_explored.open (frontiers_async_node::folder_name + "/current/volume_explored.csv", std::ofstream::app);
		frontiers_async_node::volume_explored << "time ellapsed minutes,volume\n";
		frontiers_async_node::volume_explored.close();
		frontiers_async_node::start_exploration = std::chrono::high_resolution_clock::now();
#endif

	ros::init(argc, argv, "frontier_node_async");
	ros::NodeHandle nh;

	ros::ServiceServer frontier_status_service = nh.advertiseService("frontier_status", frontiers_async_node::check_status);
	ros::ServiceServer is_frontier_service     = nh.advertiseService("is_frontier",     frontiers_async_node::check_frontier);
	ros::ServiceServer is_explored_service     = nh.advertiseService("is_explored",     frontiers_async_node::check_unknown);
	ros::ServiceServer find_frontiers_service  = nh.advertiseService("find_frontiers",  frontiers_async_node::find_frontiers);
	ros::Subscriber octomap_sub   = nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 10, frontiers_async_node::octomap_callback);
	frontiers_async_node::marker_pub    = nh.advertise<visualization_msgs::MarkerArray>("frontiers/known_space", 1);
	frontiers_async_node::last_request_id = 0;

	frontiers_async_node::new_octree = false;
	frontiers_async_node::is_octree_inUse = false;

	ros::spin();
}
