#include <ros/ros.h>
#include <frontiers.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <frontiers_msgs/FrontierNodeStatus.h>

namespace frontiers_async_node
{
	octomap::OcTree* octree;
	ros::Publisher local_pos_pub;
	bool octomap_init;

	bool check_status(frontiers_msgs::FrontierNodeStatus::Request  &req,
        frontiers_msgs::FrontierNodeStatus::Response &res)
	{
		res.is_accepting_requests = octomap_init;
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
	ros::ServiceServer service = nh.advertiseService("frontier_status", frontiers_async_node::check_status);
	ros::Subscriber octomap_sub = nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 10, frontiers_async_node::octomap_callback);
	ros::Subscriber frontiers_sub = nh.subscribe<frontiers_msgs::FrontierRequest>("frontiers_request", 10, frontiers_async_node::frontier_callback);
	frontiers_async_node::local_pos_pub = nh.advertise<frontiers_msgs::FrontierReply>("frontiers_reply", 10);

	ros::spin();
}
