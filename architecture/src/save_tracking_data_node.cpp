#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <iostream>
#include <fstream>
#include <chrono>

int main(int argc, char** argv)
{
  std::string folder_name = "/home/mfaria/Flying_Octomap_code/src/data";
  ros::init(argc, argv, "save_tracking_data_node");

  ros::NodeHandle node;

  // ros::service::waitForService("spawn");
  // ros::ServiceClient add_turtle =
  //   node.serviceClient<turtlesim::Spawn>("spawn");
  // turtlesim::Spawn srv;
  // add_turtle.call(srv);

  // ros::Publisher turtle_vel = node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

  tf::TransformListener listener;

  while (node.ok())
  {
    
    // ROS_INFO_STREAM("[currPos] Here I am");
    std::ofstream csv_file;
    csv_file.open (folder_name + "/current/current_position.csv", std::ofstream::app);
    auto duration = std::chrono::system_clock::now().time_since_epoch();
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform("/world", "/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) 
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    csv_file << millis << "," << transform.getOrigin().x() << "," << transform.getOrigin().y() << "," << transform.getOrigin().z() << std::endl;
    csv_file.close();
    ros::Duration(1.0).sleep();
  } 
  return 0;
};