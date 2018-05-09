#include <ros/ros.h>
#include <ltStar_temp.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/Marker.h>
#include <marker_publishing_utils.h>
#include <std_srvs/Empty.h>

namespace LazyThetaStarOctree
{
    octomap::OcTree* octree;
	ros::Publisher ltstar_reply_pub;
	ros::Publisher marker_pub;
	ros::ServiceClient pauseGazebo;
	ros::ServiceClient unpauseGazebo;
		
	bool octomap_init;
	bool publish_free_corridor_arrows;

	bool check_status(path_planning_msgs::LTStarNodeStatus::Request  &req,
        path_planning_msgs::LTStarNodeStatus::Response &res)
	{
		res.is_accepting_requests = octomap_init;
	  	return true;
	}

	void ltstar_callback(const path_planning_msgs::LTStarRequest::ConstPtr& path_request)
	{
		rviz_interface::publish_deleteAll(marker_pub);
		path_planning_msgs::LTStarReply reply;
		reply.waypoint_amount = 0;
		reply.success = false;
		if(octomap_init)
		{
			std::stringstream ss;
			std::string path = "/ros_ws/src/data/";
			ss << path << "(" << path_request->start.x << "; " << path_request->start.y << "; " << path_request->start.z << ")_(" 
				<<  path_request->goal.x << "; " << path_request->goal.y << "; " << path_request->goal.z << ").bt";
			octree->writeBinary(ss.str());
			ROS_WARN_STREAM("[LTStar] Request message " << *path_request);
			if(path_request->request_id > 5)
			{
				publish_free_corridor_arrows = true;
			}
			else
			{
				publish_free_corridor_arrows = false;
			}
			LazyThetaStarOctree::processLTStarRequest(*octree, *path_request, reply, marker_pub, pauseGazebo, unpauseGazebo, publish_free_corridor_arrows);
			if(reply.waypoint_amount == 1)
			{
				ROS_ERROR_STREAM("[LTStar] The resulting path has only one waypoint. It should always have at least start and goal. Here is the request message (the octree was saved to /data) " << *path_request);
				ROS_ERROR_STREAM("[LTStar] And here is the reply " << reply);
				octree->writeBinary("/ros_ws/src/data/one_waypointed_path.bt");
			}
			octree->writeBinary("/ros_ws/src/data/octree_after_processing_request.bt");
		}
		else
		{
			reply.success=false;
			reply.request_id = path_request->request_id;
			reply.waypoint_amount = 0;
		}
		ltstar_reply_pub.publish(reply);

		visualization_msgs::MarkerArray waypoint_array;
		visualization_msgs::MarkerArray arrow_array;
		visualization_msgs::Marker marker_temp;
		// Publish to rviz
		for (int i = 0; i < reply.waypoint_amount; ++i)
		{

			octomath::Vector3 candidate (reply.waypoints[i].x, reply.waypoints[i].y, reply.waypoints[i].z);
			std::unordered_set<std::shared_ptr<octomath::Vector3>> neighbors;
	        octomap::OcTreeKey key = octree->coordToKey(candidate);
	        double depth = getNodeDepth_Octomap(key, *octree);
	        double side_length = findSideLenght(*octree, depth);
	        rviz_interface::build_waypoint(candidate, side_length, (0.3*i)/reply.waypoint_amount, i, marker_temp);
	        waypoint_array.markers.push_back( marker_temp );
	        if(i !=0)
	        {

				visualization_msgs::Marker marker_temp;
				octomath::Vector3 prev_candidate (reply.waypoints[i-1].x, reply.waypoints[i-1].y, reply.waypoints[i-1].z);
				rviz_interface::build_arrow_path(candidate, prev_candidate, i, marker_temp);
	        	// ROS_WARN_STREAM("[LTStar] " << i << " Publish arrow from " << candidate << " to " << prev_candidate << marker_temp);
				arrow_array.markers.push_back( marker_temp );
	        }
		}
		marker_pub.publish(arrow_array);
		marker_pub.publish(waypoint_array);
	}
	
	void octomap_callback(const octomap_msgs::Octomap::ConstPtr& octomapBinary){
		delete octree;
		octree = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*octomapBinary);
		octomap_init = true;
	}
}

int main(int argc, char **argv)
{
	LazyThetaStarOctree::publish_free_corridor_arrows = true;
	ros::init(argc, argv, "ltstar_async_node");
	ros::NodeHandle nh;
	ros::ServiceServer ltstar_status_service = nh.advertiseService("ltstar_status", LazyThetaStarOctree::check_status);
	ros::Subscriber octomap_sub = nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 10, LazyThetaStarOctree::octomap_callback);
	ros::Subscriber ltstars_sub = nh.subscribe<path_planning_msgs::LTStarRequest>("ltstar_request", 10, LazyThetaStarOctree::ltstar_callback);
	LazyThetaStarOctree::ltstar_reply_pub = nh.advertise<path_planning_msgs::LTStarReply>("ltstar_reply", 10);
	LazyThetaStarOctree::marker_pub = nh.advertise<visualization_msgs::MarkerArray>("ltstar_path", 1);
	LazyThetaStarOctree::pauseGazebo = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
	LazyThetaStarOctree::unpauseGazebo = nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");

	ros::spin();
}