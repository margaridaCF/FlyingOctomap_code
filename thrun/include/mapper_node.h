#ifndef MAPPER_NODE_H
#define MAPPER_NODE_H


#include "mapper2d.h"
#include <tf2_ros/transform_listener.h>
#include "octomap_msgs/Octomap.h"

namespace mapper
{

	class MapperNode
	{
	public:
	  MapperNode(ros::NodeHandle nh, std::string name);
	  bool test_unit();

	private:
	  ros::Subscriber sub_octomap;
	  void callback(const octomap_msgs::Octomap::ConstPtr& octomapBinary);
	  void getRobotTransform(geometry_msgs::TransformStamped& transformStamped);
	  tf2_ros::Buffer tfBuffer;
      tf2_ros::TransformListener tfListener;
      //Mapper2D mapper;
	};

}

#endif // MAPPER_NODE_H
