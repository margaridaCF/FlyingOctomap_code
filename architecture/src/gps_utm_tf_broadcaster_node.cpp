#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>


namespace gps_utm_tf_broadcaster
{
  std::string uav_f;
  std::string world_f;

  void poseCallback(const nav_msgs::OdometryConstPtr& msg){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z) );
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, world_f, uav_f));
  }
  
}



int main(int argc, char** argv){
  ros::init(argc, argv, "gps_utm_tf_broadcaster");
  ros::NodeHandle node;

  node.getParam("uav_frame", gps_utm_tf_broadcaster::uav_f);
  node.getParam("world_frame", gps_utm_tf_broadcaster::world_f);
  ROS_WARN_STREAM("uav_frame " << gps_utm_tf_broadcaster::world_f << " world_frame " << gps_utm_tf_broadcaster::uav_f);

  ros::Subscriber sub = node.subscribe(+"mavros/global_position/local", 10, &gps_utm_tf_broadcaster::poseCallback);

  ros::spin();
  return 0;
};
