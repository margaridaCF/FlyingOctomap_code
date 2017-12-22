#include <ros/ros.h>
#include "twopcmaker.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "two_pc_maker_node");
  
	// Resolv RGBD camera name
	ros::NodeHandle lnh("~");
    std::string frontCameraTopic, backCameraTopic;
    if(!lnh.getParam("front_camera_topic", frontCameraTopic))
        frontCameraTopic = "front";
    if(!lnh.getParam("back_camera_topic", backCameraTopic))
        backCameraTopic = "back";

	// Visual odometry instance
    std::string nodeName = "two_pc_maker_node";
    TwoPcMaker m(nodeName, frontCameraTopic, backCameraTopic);
	
	// Spin for ever
	ros::spin();
}




