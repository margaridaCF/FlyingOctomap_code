#include <ros/ros.h>
#include <ltStar_temp.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

namespace LazyThetaStarOctree
{
    octomap::OcTree* octree;
	ros::Publisher ltstar_reply_pub;
	// ros::Publisher marker_pub;
		
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
		if(octomap_init)
		{
			LazyThetaStarOctree::processLTStarRequest(*octree, *path_request, reply);
		}
		else
		{
			reply.success=false;
			reply.request_id = path_request->request_id;
			reply.waypoint_amount = 0;
		}
		ltstar_reply_pub.publish(reply);
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
	// LazyThetaStarOctree::marker_pub = nh.advertise<visualization_msgs::Marker>("is_ltstar_markers", 1);

	ros::spin();
}