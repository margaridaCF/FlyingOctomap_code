#include <ros/ros.h>
#include <frontiers.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <frontiers_msgs/FrontierNodeStatus.h>
#include <frontiers_msgs/CheckIsFrontier.h>
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/Point.h>

namespace frontiers_async_node
{
	octomap::OcTree* octree;
	ros::Publisher local_pos_pub;
	ros::Publisher marker_pub;
		
	bool octomap_init;

	bool check_status(frontiers_msgs::FrontierNodeStatus::Request  &req,
        frontiers_msgs::FrontierNodeStatus::Response &res)
	{
		res.is_accepting_requests = octomap_init;
	  	return true;
	}

	void publish_marker(octomath::Vector3 & candidate, bool is_frontier)
	{
		uint32_t shape = visualization_msgs::Marker::CUBE;
		visualization_msgs::Marker marker;
	    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
	    marker.header.frame_id = "/map";
	    marker.header.stamp = ros::Time::now();
	    marker.ns = "frontier_candidate";
    	marker.id = 0;
    	marker.type = shape;
    	marker.action = visualization_msgs::Marker::ADD;
    	marker.pose.position.x = candidate.x();
	    marker.pose.position.y = candidate.y();
	    marker.pose.position.z = candidate.z();
	    marker.pose.orientation.x = 0.0;
	    marker.pose.orientation.y = 0.0;
	    marker.pose.orientation.z = 0.0;
	    marker.pose.orientation.w = 1.0;
	    marker.scale.x = 0.2;
	    marker.scale.y = 0.2;
	    marker.scale.z = 0.2;
	    if(is_frontier)
	    {
		    marker.color.r = 0.0f;
		    marker.color.g = 1.0f;
		    marker.color.b = 0.0f;
		    marker.color.a = 1.0;
	    }
	    else
	    {
		    marker.color.r = 1.0f;
		    marker.color.g = 0.0f;
		    marker.color.b = 0.0f;
		    marker.color.a = 1.0;
	    }
	    marker.lifetime = ros::Duration();
	    marker_pub.publish(marker);
	}

	bool check_frontier(frontiers_msgs::CheckIsFrontier::Request  &req,
		frontiers_msgs::CheckIsFrontier::Response &res)
	{
		octomath::Vector3 candidate(req.candidate.x, req.candidate.y, req.candidate.z);
		res.is_frontier = Frontiers::isFrontier(*octree, candidate);
		publish_marker(candidate, res.is_frontier);
		return true;
	}

	void frontier_callback(const frontiers_msgs::FrontierRequest::ConstPtr& frontier_request)
	{
		frontiers_msgs::FrontierReply reply;
		if(octomap_init)
		{
			Frontiers::processFrontiersRequest(*octree, *frontier_request, reply);
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
	ros::init(argc, argv, "frontier_node_async");
	ros::NodeHandle nh;
	ros::ServiceServer frontier_status_service = nh.advertiseService("frontier_status", frontiers_async_node::check_status);
	ros::ServiceServer is_frontier_service = nh.advertiseService("is_frontier", frontiers_async_node::check_frontier);
	ros::Subscriber octomap_sub = nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 10, frontiers_async_node::octomap_callback);
	ros::Subscriber frontiers_sub = nh.subscribe<frontiers_msgs::FrontierRequest>("frontiers_request", 10, frontiers_async_node::frontier_callback);
	frontiers_async_node::local_pos_pub = nh.advertise<frontiers_msgs::FrontierReply>("frontiers_reply", 10);
	frontiers_async_node::marker_pub = nh.advertise<visualization_msgs::Marker>("is_frontier_markers", 1);

	ros::spin();
}
