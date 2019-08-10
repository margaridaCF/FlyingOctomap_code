#include <ros/ros.h>
#include <ltStar_lib_ortho.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/Marker.h>
#include <marker_publishing_utils.h>
#include <std_srvs/Empty.h>
#include <lazy_theta_star_msgs/CheckFlightCorridor.h>
#include <lazy_theta_star_msgs/CheckVisibility.h>
#include <tf/transform_datatypes.h>



#include <sstream>
#include <fstream>
#include <string>
#include <chrono>
#include <boost/filesystem.hpp>

#define SAVE_CSV 1
// #define STANDALONE 1

namespace LazyThetaStarOctree
{


    octomap::OcTree* octree;
	double sidelength_lookup_table  [16]; 
	ros::Publisher ltstar_reply_pub;
	ros::Publisher marker_pub;
		
	bool octomap_init;
	bool publish_free_corridor_arrows;

	bool check_status(lazy_theta_star_msgs::LTStarNodeStatus::Request  &req,
        lazy_theta_star_msgs::LTStarNodeStatus::Response &res)
	{
		res.is_accepting_requests = octomap_init;
	  	return true;
	}

	bool checkVisibility(lazy_theta_star_msgs::CheckVisibility::Request &request,
		lazy_theta_star_msgs::CheckVisibility::Response &response)
	{
		octomath::Vector3 start(request.start.x, request.start.y, request.start.z);
		octomath::Vector3 end  (request.end.x, request.end.y, request.end.z);
		InputData input (*octree, start, end, 0);
		response.has_visibility = hasLineOfSight_UnknownAsFree(input, rviz_interface::PublishingInput( marker_pub, false));
		return true;
	}

	bool checkFligthCorridor_(double flight_corridor_width, octomath::Vector3 start, octomath::Vector3 end)
	{
		LazyThetaStarOctree::generateOffsets(octree->getResolution(), flight_corridor_width, semiSphereIn, semiSphereOut );
		InputData input (*octree, start, end, flight_corridor_width);
		return is_flight_corridor_free(input, rviz_interface::PublishingInput( marker_pub, false));
	}

	bool checkFligthCorridor(lazy_theta_star_msgs::CheckFlightCorridor::Request &request,
		lazy_theta_star_msgs::CheckFlightCorridor::Response &response)
	{
		octomath::Vector3 start(request.start.x, request.start.y, request.start.z);
		octomath::Vector3 end  (request.end.x, request.end.y, request.end.z);
		response.free = checkFligthCorridor_(request.flight_corridor_width, start, end);
		return true;
	}
	
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
	        octomath::Vector3 cell_center;
			double side_length;
			try
			{
		        octomap::OcTreeKey key = octree->coordToKey(candidate);
		        double depth = getNodeDepth_Octomap(key, *octree);
		        side_length = findSideLenght(octree->getTreeDepth(), depth, sidelength_lookup_table);
		        cell_center = octree->keyToCoord(key, depth);
		    }
		    catch(const std::out_of_range& oor)
		    {
		    	// This occurs when the start and end coordinates are the actual waypoints
		    	cell_center = candidate;
		    	side_length = octree->getResolution();
		    }

	        if( cell_center.distance(candidate) < 0.001 )
	        {
		        rviz_interface::build_waypoint(candidate, side_length, (0.3*i)/reply.waypoint_amount, i+series, marker_temp, series);
		        waypoint_array.markers.push_back( marker_temp );
	        }
	        if(i !=0)
	        {

				visualization_msgs::Marker marker_temp;
				octomath::Vector3 prev_candidate (reply.waypoints[i-1].position.x, reply.waypoints[i-1].position.y, reply.waypoints[i-1].position.z);
				rviz_interface::build_arrow_path(prev_candidate, candidate, i, marker_temp, series);
				arrow_array.markers.push_back( marker_temp );
	        }
		}
		marker_pub.publish(arrow_array);
		marker_pub.publish(waypoint_array);
	}

	void ltstar_callback(const lazy_theta_star_msgs::LTStarRequest::ConstPtr& path_request)
	{
		rviz_interface::publish_deleteAll(marker_pub);
		lazy_theta_star_msgs::LTStarReply reply;
		reply.waypoint_amount = 0;
		reply.success = false;
		if(octomap_init)
		{
			// std::stringstream ss;
			// ss << folder_name << "/(" << path_request->start.x << "; " << path_request->start.y << "; " << path_request->start.z << ")_(" 
			// 	<<  path_request->goal.x << "; " << path_request->goal.y << "; " << path_request->goal.z << ").bt";
			// octree->writeBinary(ss.str());
			ROS_INFO_STREAM("[LTStar] Request message " << *path_request);

			octomath::Vector3 start(path_request->start.x, path_request->start.y, path_request->start.z);
			octomath::Vector3 goal  (path_request->goal.x, path_request->goal.y, path_request->goal.z);
			bool free_straight_line = checkFligthCorridor_(path_request->safety_margin, start, goal);
			if(free_straight_line)
			{
				reply.success = true;
				reply.request_id = path_request->request_id;
				reply.waypoint_amount = 2;

				geometry_msgs::Pose waypoint;
	            waypoint.position.x = path_request->start.x;
	            waypoint.position.y = path_request->start.y;
	            waypoint.position.z = path_request->start.z;
	            waypoint.orientation = tf::createQuaternionMsgFromYaw(0);
	            reply.waypoints.push_back(waypoint);
	            waypoint.position.x = path_request->goal.x;
	            waypoint.position.y = path_request->goal.y;
	            waypoint.position.z = path_request->goal.z;
	            waypoint.orientation = tf::createQuaternionMsgFromYaw(0);
	            reply.waypoints.push_back(waypoint);
			}
			else
			{
				LazyThetaStarOctree::generateOffsets(octree->getResolution(), path_request->safety_margin, dephtZero, semiSphereOut );
				LazyThetaStarOctree::processLTStarRequest(*octree, *path_request, reply, sidelength_lookup_table, rviz_interface::PublishingInput( marker_pub, true) );
				if(reply.waypoint_amount == 1)
				{
					ROS_ERROR_STREAM("[LTStar] The resulting path has only one waypoint. Request: " << *path_request);
				}
				// ROS_INFO_STREAM("[LTStar] Reply " << reply);

				// octree->writeBinary(folder_name + "/octree_after_processing_request.bt");
			}
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

#ifdef STANDALONE
	
	std::stringstream aux_envvar_home (std::getenv("HOME"));
	LazyThetaStarOctree::folder_name = aux_envvar_home.str() + "/Flying_Octomap_code/src/data";
	auto timestamp_chrono = std::chrono::high_resolution_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(timestamp_chrono - std::chrono::hours(24));
    std::stringstream folder_name_stream;
    folder_name_stream << LazyThetaStarOctree::folder_name << (std::put_time(std::localtime(&now_c), "%F %T") );
	LazyThetaStarOctree::folder_name = LazyThetaStarOctree::folder_name + "/current";
    boost::filesystem::create_directories(folder_name_stream.str());
    boost::filesystem::create_directory_symlink(folder_name_stream.str(), LazyThetaStarOctree::folder_name);
#endif

#ifdef SAVE_CSV
	ROS_WARN_STREAM("[main] Saving to " << LazyThetaStarOctree::folder_name << "/current/lazyThetaStar_computation_time.csv");

	std::ofstream csv_file;
	csv_file.open (LazyThetaStarOctree::folder_name+"/current/lazyThetaStar_computation_time.csv", std::ofstream::app);
	csv_file << "success,computation_time_millis,path_lenght_straight_line_meters,path_lenght_total_meters,has_obstacle,start,goal,safety_margin_meters,max_search_duration_seconds,iteration_count,obstacle_hit_count,total_obstacle_checks,dataset_name" << std::endl;

	csv_file.close();
#endif
	LazyThetaStarOctree::publish_free_corridor_arrows = true;
	ros::init(argc, argv, "ltstar_async_node");
	ros::NodeHandle nh;
	ros::ServiceServer ltstar_status_service= nh.advertiseService("ltstar_status", LazyThetaStarOctree::check_status);
	ros::ServiceServer lineOfSight_sub 		= nh.advertiseService("is_fligh_corridor_free", LazyThetaStarOctree::checkFligthCorridor);
	ros::ServiceServer visibility_sub 		= nh.advertiseService("has_visibility", LazyThetaStarOctree::checkVisibility);
	ros::Subscriber octomap_sub 			= nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 10, LazyThetaStarOctree::octomap_callback);
	ros::Subscriber ltstar_sub 				= nh.subscribe<lazy_theta_star_msgs::LTStarRequest>("ltstar_request", 10, LazyThetaStarOctree::ltstar_callback);
	LazyThetaStarOctree::ltstar_reply_pub 	= nh.advertise<lazy_theta_star_msgs::LTStarReply>("ltstar_reply", 10);
	LazyThetaStarOctree::marker_pub 		= nh.advertise<visualization_msgs::MarkerArray>("ltstar_path", 1);

	ros::spin();
}