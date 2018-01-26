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
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <frontier_cells_node.h>


geometry_msgs::PoseStamped frontier_pose;
// TODO WARNING - concurrency!!
octomap::OcTree* octree;
mavros_msgs::State current_state;
bool get_frontier_node;
// ======================

void octomap_callback(const octomap_msgs::Octomap::ConstPtr& octomapBinary){
  octree = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*octomapBinary);
}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
  current_state = *msg;
}

octomath::Vector3 findNextFrontier()
{
  mapper::GridBenchmark generic(experimental, fileName, octree);

  float grid_resolution = octree.getResolution();
  std::tuple<double, double, double> min, max;
      octree.getMetricMin(std::get<0>(min), std::get<1>(min), std::get<2>(min));
      octree.getMetricMax(std::get<0>(max), std::get<1>(max), std::get<2>(max));
  real_max  =octomath::Vector3 (std::get<0>(max), std::get<1>(max), 1);
  real_min = octomath::Vector3(std::get<0>(min), std::get<1>(min), 0);
  draw_3D_max  =octomath::Vector3 (real_max.x(), real_max.y(), 1);
  draw_3D_min = octomath::Vector3(real_min.x(), real_min.y(), 0);

  mapper::SparseGrid grid3D("test", mapper::Algorithm::sparseGrid_3d, octree, grid_resolution);  
  mapper::ResultSet result_sparse_3D = generic.findFrontierCells(real_max, real_min, threeD, grid3D);

  return generic.frontierCells.front();
}



void frontierSpin(){
  ros::spin();
}

int main(int argc, char **argv)
{
  // Initialise a few objects etc.
  ros::init(argc, argv, "frontier_node");
  ros::NodeHandle nh;
  ros::Subscriber octomap_sub = nh.subscribe<mavros_msgs::State>("octomap_binary", 10, octomap_callback);
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
            

  ros::Rate poll_rate(100);
  ros::Rate frontier_check_rate(100);

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

 //  get_frontier_node = true;
 //  while(ros::ok())
 // {
    octomath::Vector3 next_frontier_vector3 = findNextFrontier();
    ROS_WARN_STREAM("Nest frontier " << next_frontier_vector3);
    frontier_pose.pose.position.x = next_frontier_vector3.x();
    frontier_pose.pose.position.y = next_frontier_vector3.y();
    frontier_pose.pose.position.z = next_frontier_vector3.z();
    local_pos_pub.publish(frontier_pose);
    get_frontier_node = false;
    frontier_check_rate.sleep();
    ros::spin();
 // }




}
