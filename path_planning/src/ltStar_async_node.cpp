#include <ros/ros.h>
#include <ltStar_temp.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/Marker.h>
#include <marker_publishing_utils.h>
#include <std_srvs/Empty.h>


#include <sstream>
#include <fstream>
#include <string>
#include <chrono>
#include <boost/filesystem.hpp>

// #define SAVE_CSV 1
#define STANDALONE 1

namespace LazyThetaStarOctree
{
	std::string folder_name;


    octomap::OcTree* octree;
	double sidelength_lookup_table  [16]; 
	ros::Publisher ltstar_reply_pub;
	ros::Publisher marker_pub;
		
	bool octomap_init;
	bool publish_free_corridor_arrows;

	bool check_status(path_planning_msgs::LTStarNodeStatus::Request  &req,
        path_planning_msgs::LTStarNodeStatus::Response &res)
	{
		res.is_accepting_requests = octomap_init;
	  	return true;
	}
	
	void publishResultingPath(path_planning_msgs::LTStarReply reply, int series )
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

	void ltstar_callback(const path_planning_msgs::LTStarRequest::ConstPtr& path_request)
	{
		rviz_interface::publish_deleteAll(marker_pub);
		path_planning_msgs::LTStarReply reply;
		reply.waypoint_amount = 0;
		reply.success = false;
		if(octomap_init)
		{
			// std::stringstream ss;
			// ss << folder_name << "/(" << path_request->start.x << "; " << path_request->start.y << "; " << path_request->start.z << ")_(" 
			// 	<<  path_request->goal.x << "; " << path_request->goal.y << "; " << path_request->goal.z << ").bt";
			// octree->writeBinary(ss.str());
			ROS_INFO_STREAM("[LTStar] Request message " << *path_request);
			// if(path_request->request_id > 5)
			// {
			// 	publish_free_corridor_arrows = true;
			// }
			// else
			// {
			// 	publish_free_corridor_arrows = false;
			// }
			LazyThetaStarOctree::processLTStarRequest(*octree, *path_request, reply, sidelength_lookup_table, marker_pub, true);
			if(reply.waypoint_amount == 1)
			{
				ROS_ERROR_STREAM("[LTStar] The resulting path has only one waypoint. Request: " << *path_request);
			}
			// ROS_INFO_STREAM("[LTStar] Reply " << reply);

			// octree->writeBinary(folder_name + "/octree_after_processing_request.bt");
		}
		else
		{
			ROS_ERROR_STREAM("[LTStar] Cannot generate path because no octomap has been received.");
			reply.success=false;
			reply.request_id = path_request->request_id;
			reply.waypoint_amount = 0;
		}
		ltstar_reply_pub.publish(reply);

		publishResultingPath(reply, 9);
	}

	void ltstar_benchmark_callback(const path_planning_msgs::LTStarBenchmarkRequest::ConstPtr& path_request)
	{
		rviz_interface::publish_deleteAll(marker_pub);
		path_planning_msgs::LTStarReply reply;
		reply.waypoint_amount = 0;
		reply.success = false;
		if(octomap_init)
		{
			path_planning_msgs::LTStarRequest request_vanilla;
			request_vanilla.start = path_request->start;
			request_vanilla.goal = path_request->goal;
			request_vanilla.safety_margin = path_request->safety_margin; 
			request_vanilla.max_search_iterations = path_request->max_search_iterations;

			ROS_INFO_STREAM("[LTStar] Request message " << request_vanilla);
			auto start = std::chrono::high_resolution_clock::now();
			LazyThetaStarOctree::processLTStarRequest(*octree, request_vanilla, reply, sidelength_lookup_table, marker_pub, true);
			auto finish = std::chrono::high_resolution_clock::now();
			auto time_span = finish - start;
			ROS_WARN_STREAM("Vanilla took " << std::chrono::duration_cast<std::chrono::milliseconds>(time_span).count());
			if(reply.waypoint_amount == 1)
			{
				ROS_ERROR_STREAM("[LTStar] [Vanilla] The resulting path has only one waypoint. Request: " << *path_request);
			}
			ltstar_reply_pub.publish(reply);
			publishResultingPath(reply, 9);

			start = std::chrono::high_resolution_clock::now();
			LazyThetaStarOctree::processLTStarRequest_margin(*octree, *path_request, reply, sidelength_lookup_table, marker_pub, true);
			finish = std::chrono::high_resolution_clock::now();
			time_span = finish - start;
			ROS_WARN_STREAM("Margin took " << std::chrono::duration_cast<std::chrono::milliseconds>(time_span).count());
			if(reply.waypoint_amount == 1)
			{
				ROS_ERROR_STREAM("[LTStar] [Margin] The resulting path has only one waypoint. Request: " << *path_request);
			}
			ltstar_reply_pub.publish(reply);
			publishResultingPath(reply, 2);
		}
		else
		{
			ROS_ERROR_STREAM("[LTStar] Cannot generate path because no octomap has been received.");
			reply.success=false;
			reply.request_id = path_request->request_id;
			reply.waypoint_amount = 0;
		}
		
	}

	void octomap_callback(const octomap_msgs::Octomap::ConstPtr& octomapBinary){
		delete octree;
		octree = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*octomapBinary);
		if(!octomap_init)
		{
	    	LazyThetaStarOctree::fillLookupTable(octree->getResolution(), octree->getTreeDepth(), sidelength_lookup_table); 
		}
		octomap_init = true;
	}
}

int main(int argc, char **argv)
{
	LazyThetaStarOctree::folder_name = "/ros_ws/src/data/current";

#ifdef STANDALONE
	LazyThetaStarOctree::folder_name = "/home/mfaria/Flying_Octomap_code/src/data/";
	auto timestamp_chrono = std::chrono::high_resolution_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(timestamp_chrono - std::chrono::hours(24));
    std::stringstream folder_name_stream;
    folder_name_stream << LazyThetaStarOctree::folder_name << (std::put_time(std::localtime(&now_c), "%F %T") );
	LazyThetaStarOctree::folder_name = LazyThetaStarOctree::folder_name + "current";
    boost::filesystem::create_directories(folder_name_stream.str());
    boost::filesystem::create_directory_symlink(folder_name_stream.str(), LazyThetaStarOctree::folder_name);
#endif

#ifdef SAVE_CSV
	std::ofstream csv_file;
	csv_file.open (LazyThetaStarOctree::folder_name+"/lazyThetaStar_computation_time.csv", std::ofstream::app);
	csv_file << "computation_time_millis,path_lenght_straight_line_meters,path_lenght_total_meters,has_obstacle,start,goal,safety_margin_meters,max_search_duration_seconds" << std::endl;
	csv_file.close();
#endif
	LazyThetaStarOctree::publish_free_corridor_arrows = true;
	ros::init(argc, argv, "ltstar_async_node");
	ros::NodeHandle nh;
	ros::ServiceServer ltstar_status_service = nh.advertiseService("ltstar_status", LazyThetaStarOctree::check_status);
	ros::Subscriber octomap_sub = nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 10, LazyThetaStarOctree::octomap_callback);
	ros::Subscriber ltstar_sub = nh.subscribe<path_planning_msgs::LTStarRequest>("ltstar_request", 10, LazyThetaStarOctree::ltstar_callback);
	ros::Subscriber ltstar_benchmark_sub = nh.subscribe<path_planning_msgs::LTStarBenchmarkRequest>("ltstar_request_benchmark", 10, LazyThetaStarOctree::ltstar_benchmark_callback);
	LazyThetaStarOctree::ltstar_reply_pub = nh.advertise<path_planning_msgs::LTStarReply>("ltstar_reply", 10);
	LazyThetaStarOctree::marker_pub = nh.advertise<visualization_msgs::MarkerArray>("ltstar_path", 1);

	ros::spin();
}