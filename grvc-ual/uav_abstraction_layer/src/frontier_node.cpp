/**
 * @file frontier_node.cpp
 * @ TODO: Description
 */

 #include <ros/ros.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <mavros_msgs/CommandBool.h>
 #include <mavros_msgs/SetMode.h>
 #include <mavros_msgs/State.h>
 #include <mavros_msgs/PositionTarget.h>
 #include <std_msgs/Empty.h>
 #include <geometry_msgs/TwistStamped.h>
 #include <octomap/OcTree.h>
 #include <octomap/math/Vector3.h>


geometry_msgs::PoseStamped frontier_pose;
/*
void octomapCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  break;
}*/

/*void subscriberCallback(const ........... msg){
  break;
}*/

void frontierSpin(){
  ros::spin();
}

int main(int argc, char **argv)
{
  // Initialise a few objects etc.
  ros::init(argc, argv, "frontier_node");
  ros::NodeHandle nh;
  ros::Rate poll_rate(100);

  bool latch_on = 1 ;

//  ros::Subscriber octomap_sub = nh.subscribe<octomap::

  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/new_frontier", 10, latch_on);

  frontier_pose.pose.position.x = 0.0;
  frontier_pose.pose.position.y = 0.0;
  frontier_pose.pose.position.z = 2.0;

  while(local_pos_pub.getNumSubscribers()==0)
 {
    ROS_ERROR("Waiting for subscibers");
    sleep(10);
 }
 ROS_ERROR("Got subscriber");
 local_pos_pub.publish(frontier_pose);
 ros::spinOnce();

  while(ros::ok())
 {
    ros::spin();
 }




}
