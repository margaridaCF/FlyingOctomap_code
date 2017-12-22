#include <ros/ros.h>
#include "pcmaker.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pc_maker_node");
  
	// Resolv RGBD camera name
	ros::NodeHandle lnh("~");
	std::string cameraTopic;
	if(!lnh.getParam("camera_topic", cameraTopic))
		cameraTopic = "camera";
  
	// Visual odometry instance
    std::string nodeName = "pc_maker_node";
    PcMaker m(nodeName, cameraTopic);
	
	// Spin for ever
	ros::spin();
}




