#include <ros/ros.h>
#include <frontiers.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>


octomap::OcTree* octree;
ros::Publisher local_pos_pub;

void frontier_callback(const frontiers_msgs::FrontierRequest::ConstPtr& frontier_request)
{
	frontiers_msgs::FrontierReply reply;
	bool outcome = Frontiers::processFrontiersRequest(*octree, *frontier_request, reply);
	local_pos_pub.publish(reply);
}

void octomap_callback(const octomap_msgs::Octomap::ConstPtr& octomapBinary){
	delete octree;
	octree = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*octomapBinary);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "frontier_node_async");
	ros::NodeHandle nh;
	ros::Subscriber octomap_sub = nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 10, octomap_callback);
	local_pos_pub = nh.advertise<frontiers_msgs::FrontierReply>("frontiers_found", 10);

	ros::spin();
}
