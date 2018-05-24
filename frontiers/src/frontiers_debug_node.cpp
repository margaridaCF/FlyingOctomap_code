#include <ros/ros.h>
#include <frontiers.h>
#include <neighbors.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

namespace frontiers_debug_node
{
	ros::Publisher marker_pub;
	void neighbor_callback (const frontiers_msgs::VoxelMsg::ConstPtr& voxel)
	{
		std::unordered_set<std::shared_ptr<octomath::Vector3>> neighbors;
		octomath::Vector3 center_coords(voxel->xyz_m.x, voxel->xyz_m.y, voxel->xyz_m.z);
		float resolution = 0.5;
		LazyThetaStarOctree::generateNeighbors_frontiers_pointers( neighbors, center_coords, voxel->size, resolution);
		visualization_msgs::MarkerArray marker_array;
		rviz_interface::build_neighbor_array(neighbors, marker_array);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "frontier_candidate";
        marker.id = 10;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = center_coords.x();
        marker.pose.position.y = center_coords.y();
        marker.pose.position.z = center_coords.z();
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = resolution;
        marker.scale.y = resolution;
        marker.scale.z = resolution;
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();
        marker_array.markers.push_back(marker);

        marker_pub.publish(marker_array);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ltstar_debug_node");
	ros::NodeHandle nh;
	frontiers_debug_node::marker_pub = nh.advertise<visualization_msgs::MarkerArray>("ltstar_path", 1);
	ros::Subscriber octomap_sub = nh.subscribe<frontiers_msgs::VoxelMsg>("/neighbor_generation", 10, frontiers_debug_node::neighbor_callback);
	ros::spin();
}