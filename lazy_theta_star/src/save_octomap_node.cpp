#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <ltStar_lib_ortho.h>
#include <octomap_msgs/conversions.h>

namespace save_octomap_node
{
    octomap::OcTree* octree;
		
	bool octomap_init;

	void ltstar_callback(const lazy_theta_star_msgs::LTStarRequest::ConstPtr& path_request)
	{
		if(octomap_init)
		{
			octree->writeBinary("octomap.bt");
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
	ros::init(argc, argv, "save_octomap_node");
	ros::NodeHandle nh;

	ros::Subscriber octomap_sub = nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 10, save_octomap_node::octomap_callback);
	ros::Subscriber ltstar_sub = nh.subscribe<lazy_theta_star_msgs::LTStarRequest>("ltstar_request", 10, save_octomap_node::ltstar_callback);
	ros::spin();
}