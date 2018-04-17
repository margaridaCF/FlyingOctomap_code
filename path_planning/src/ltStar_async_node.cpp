#include <ros/ros.h>
#include <ltStar_temp.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/Marker.h>
#include <marker_publishing_utils.h>

namespace LazyThetaStarOctree
{
    octomap::OcTree* octree;
	ros::Publisher ltstar_reply_pub;
	ros::Publisher marker_pub;
		
	bool octomap_init;

	bool check_status(path_planning_msgs::LTStarNodeStatus::Request  &req,
        path_planning_msgs::LTStarNodeStatus::Response &res)
	{
		res.is_accepting_requests = octomap_init;
	  	return true;
	}

	void ltstar_callback(const path_planning_msgs::LTStarRequest::ConstPtr& path_request)
	{
		path_planning_msgs::LTStarReply reply;
		reply.waypoint_amount = 0;
		reply.success = false;
		if(octomap_init)
		{
			LazyThetaStarOctree::processLTStarRequest(*octree, *path_request, reply);
			if(reply.waypoint_amount == 1)
			{
				ROS_ERROR_STREAM("[LTStar] The resulting path has only one waypoint. It should always have at least start and goal. Here is the request message (the octree was saved to /data) " << *path_request);
				ROS_ERROR_STREAM("[LTStar] And here is the reply " << reply);
				octree->writeBinary("/data/one_waypointed_path.bt");
			}
		}
		else
		{
			reply.success=false;
			reply.request_id = path_request->request_id;
			reply.waypoint_amount = 0;
		}
		ltstar_reply_pub.publish(reply);

		// Publish to rviz
		for (int i = 0; i < reply.waypoint_amount; ++i)
		{

			octomath::Vector3 candidate (reply.waypoints[i].x, reply.waypoints[i].y, reply.waypoints[i].z);
			std::unordered_set<std::shared_ptr<octomath::Vector3>> neighbors;
	        // double resolution = octree->getResolution();
	        // int tree_depth = octree->getTreeDepth();
	        // octomap::OcTreeKey key = octree->coordToKey(candidate);
	        // int depth = LazyThetaStarOctree::getNodeDepth_Octomap(key, *octree);
	        // double voxel_size = ((tree_depth + 1) - depth) * resolution;
	        octomap::OcTreeKey key = octree->coordToKey(candidate);
	        double depth = getNodeDepth_Octomap(key, *octree);
	        double side_length = findSideLenght(*octree, depth);
			// ROS_WARN_STREAM("[LTStar] Waypoint " << candidate << ". Side " << side_length);
	        rviz_interface::publish_waypoint(candidate, side_length, (0.3*i)/reply.waypoint_amount, i, marker_pub);
	        if(i !=0)
	        {
				octomath::Vector3 prev_candidate (reply.waypoints[i-1].x, reply.waypoints[i-1].y, reply.waypoints[i-1].z);
	        	rviz_interface::publish_arrow_path(candidate, prev_candidate, path_request->request_id, marker_pub);
	        }
		}
	}
	
	void octomap_callback(const octomap_msgs::Octomap::ConstPtr& octomapBinary){
		delete octree;
		octree = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*octomapBinary);
		octomap_init = true;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ltstar_async_node");
	ros::NodeHandle nh;
	ros::ServiceServer ltstar_status_service = nh.advertiseService("ltstar_status", LazyThetaStarOctree::check_status);
	ros::Subscriber octomap_sub = nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 10, LazyThetaStarOctree::octomap_callback);
	ros::Subscriber ltstars_sub = nh.subscribe<path_planning_msgs::LTStarRequest>("ltstar_request", 10, LazyThetaStarOctree::ltstar_callback);
	LazyThetaStarOctree::ltstar_reply_pub = nh.advertise<path_planning_msgs::LTStarReply>("ltstar_reply", 10);
	LazyThetaStarOctree::marker_pub = nh.advertise<visualization_msgs::Marker>("ltstar_path", 1);

	ros::spin();
}