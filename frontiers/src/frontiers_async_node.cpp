#include <ros/ros.h>
#include <frontiers.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <frontiers_msgs/FrontierNodeStatus.h>
#include <frontiers_msgs/CheckIsFrontier.h>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/Point.h>
// RAM
#include "sys/types.h"
#include "sys/sysinfo.h"

#define SAVE_CSV 1

namespace frontiers_async_node
{
	octomap::OcTree* octree;
	ros::Publisher local_pos_pub;
	ros::Publisher marker_pub;
	std::string folder_name;
	double sensor_angle;
	int last_request_id;
	octomap::OcTree::leaf_bbx_iterator iterator;
	octomap::OcTree* octree_inUse;
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
			res.is_frontier = Frontiers::isFrontier(*octree, candidate, sensor_angle);
			return true;
		}
		catch(const std::out_of_range& oor)
		{
			ROS_ERROR_STREAM("[Frontiers] Candidate " << req.candidate << " is in unknown space.");
		}
	}

	void frontier_callback(const frontiers_msgs::FrontierRequest::ConstPtr& frontier_request)
	{
		frontiers_msgs::FrontierReply reply;
		if(octomap_init)
		{
			#ifdef SAVE_CSV
			std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
			#endif


			if (frontier_request->new_request)
			{
				delete octree_inUse;
				octree_inUse = octree;
				iterator = Frontiers::processFrontiersRequest(*octree, *frontier_request, reply, marker_pub);
				last_request_id = frontier_request->request_number;
			}
			else
			{
				if(last_request_id != frontier_request->request_number)
				{
					ROS_ERROR_STREAM("[Frontiers] Request number out of sync. Asked to continue search but current request number is " << last_request_id << " while in request message the id is " << frontier_request->request_number);
					return;
				}
				else
				{
					Frontiers::searchFrontier(*octree_inUse, iterator, *frontier_request, reply, marker_pub, true);
				}
			}

			#ifdef SAVE_CSV
			// Frontier computation time
			log.open (folder_name +"/frontiers_computation_time.csv", std::ofstream::app);
			std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
			std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
			std::chrono::milliseconds millis = std::chrono::duration_cast<std::chrono::milliseconds>(time_span);
			std::chrono::seconds seconds = std::chrono::duration_cast<std::chrono::seconds>(time_span);
			log << millis.count() << ", " << seconds.count() << "\n";
			log.close();
			// Explored volume
			volume_explored.open (folder_name + "/volume_explored.csv", std::ofstream::app);
			std::chrono::duration<double> ellapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>(end - start_exploration);
			std::chrono::milliseconds ellapsed_time_millis = std::chrono::duration_cast<std::chrono::milliseconds>(ellapsed_time);
			double resolution = octree->getResolution();
	        octomath::Vector3  max = octomath::Vector3(frontier_request->max.x-resolution, frontier_request->max.y-resolution, frontier_request->max.z-resolution);
	        octomath::Vector3  min = octomath::Vector3(frontier_request->min.x+resolution, frontier_request->min.y+resolution, frontier_request->min.z+resolution);
			double explored_volume_meters = calculate_volume_explored(min, max);
			volume_explored << ellapsed_time_millis.count() / 1000 / 60 << ", " << explored_volume_meters << std::endl;
			volume_explored.close();
			#endif

			if(reply.frontiers_found == 0)
	        {
	            ROS_INFO_STREAM("[Frontiers] No frontiers could be found. Writing tree to file. Request was " << *frontier_request);
	            octree->writeBinary(folder_name + "/current/octree_noFrontiers.bt"); 
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
    	std::stringstream aux_envvar_home (std::getenv("HOME"));
		frontiers_async_node::folder_name = aux_envvar_home.str() + "/Flying_Octomap_code/src/data";
		frontiers_async_node::log.open (frontiers_async_node::folder_name + "/frontiers_computation_time.csv", std::ofstream::app);
		frontiers_async_node::log << "computation_time_millis, computation_time_secs \n";
		frontiers_async_node::log.close();
		frontiers_async_node::volume_explored.open (frontiers_async_node::folder_name + "/volume_explored.csv", std::ofstream::app);
		frontiers_async_node::volume_explored << "time ellapsed minutes,volume,RAM\n";
		frontiers_async_node::volume_explored.close();
		frontiers_async_node::start_exploration = std::chrono::high_resolution_clock::now();
#endif

	ros::init(argc, argv, "frontier_node_async");
	ros::NodeHandle nh;

    nh.getParam("laser_angle", frontiers_async_node::sensor_angle);
	ros::ServiceServer frontier_status_service = nh.advertiseService("frontier_status", frontiers_async_node::check_status);
	ros::ServiceServer is_frontier_service = nh.advertiseService("is_frontier", frontiers_async_node::check_frontier);
	ros::Subscriber octomap_sub = nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 10, frontiers_async_node::octomap_callback);
	ros::Subscriber frontiers_sub = nh.subscribe<frontiers_msgs::FrontierRequest>("frontiers_request", 10, frontiers_async_node::frontier_callback);
	frontiers_async_node::local_pos_pub = nh.advertise<frontiers_msgs::FrontierReply>("frontiers_reply", 10);
	frontiers_async_node::marker_pub = nh.advertise<visualization_msgs::MarkerArray>("frontiers/known_space", 1);
	frontiers_async_node::last_request_id = 0;
	ros::spin();
}
