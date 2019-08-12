#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/GetLinkState.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle nh;


  ros::ServiceClient get_transform           = nh.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
  static tf::TransformBroadcaster br;

  while (ros::ok())
  {
    gazebo_msgs::GetLinkState srv;
    srv.request.link_name = "iris_hokuyo_1::base_link";
    srv.request.reference_frame = "map";

    if(get_transform.call(srv))
    {
      auto timestamp = ros::Time::now();
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(srv.response.link_state.pose.position.x, srv.response.link_state.pose.position.y, srv.response.link_state.pose.position.z) );
      tf::Quaternion q (srv.response.link_state.pose.orientation.x, srv.response.link_state.pose.orientation.y, srv.response.link_state.pose.orientation.z, srv.response.link_state.pose.orientation.w);
      transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(transform, timestamp, srv.request.reference_frame, "base_link"));
    }
    else
    {
        ROS_ERROR("[tf_broadcaster] Gazebo not accepting get_link_state requests.");
    }
  }
}