#include <ros/ros.h>
#include <frontiers.h>
#include <neighbors.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <frontiers_msgs/LocalGeofenceRequest.h>
#include <Eigen/Dense>
#include <frontiers_msgs/FindFrontiers.h>
#include <geometry_msgs/Point.h>

namespace frontiers_debug_node
{
	ros::Publisher marker_pub;
	void neighbor_callback (const frontiers_msgs::VoxelMsg::ConstPtr& voxel)
	{
		LazyThetaStarOctree::unordered_set_pointers neighbors;
		octomath::Vector3 center_coords(voxel->xyz_m.x, voxel->xyz_m.y, voxel->xyz_m.z);
		float resolution = 0.5;
		LazyThetaStarOctree::generateNeighbors_frontiers_pointers( neighbors, center_coords, voxel->size, resolution);
		visualization_msgs::MarkerArray marker_array;
		// rviz_interface::build_neighbor_array(neighbors, marker_array);

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

        bool fillLocalGeofence(Eigen::Vector3d start, Eigen::Vector3d end, int range)
        {
                geometry_msgs::Point geofence_max, geofence_min;
                geofence_min.x = 0;
                geofence_min.y = 0;
                geofence_min.z = 0;
                geofence_max.x = 10;
                geofence_max.y = 10;
                geofence_max.z = 10;

                frontiers_msgs::FindFrontiers           frontier_srv;

                Eigen::Vector3d direction = end - start;
                if(direction.norm() <= 0.01)
                {
                        ROS_ERROR_STREAM("Still have to do this! start " << start << "; end " << end);
                }
                Eigen::Vector3d ortho = Eigen::Vector3d::UnitZ().cross(direction);
                ortho.normalize();
                Eigen::Vector3d a = start + (ortho * range);
                Eigen::Vector3d b = a + direction;
                Eigen::Vector3d c = start - (ortho * range);
                Eigen::Vector3d d = c + direction;


                // Geofence
                // Min
                frontier_srv.request.min.x = std::max(std::min({a.x(), b.x(), c.x(), d.x()}), geofence_min.x);
                frontier_srv.request.min.y = std::max(std::min({a.y(), b.y(), c.y(), d.y()}), geofence_min.y);
                frontier_srv.request.min.z = std::max(std::min({start.z(), end.z()}), geofence_min.z);
                // Max
                frontier_srv.request.max.x = std::min(std::max({a.x(), b.x(), c.x(), d.x()}), geofence_max.x);
                frontier_srv.request.max.y = std::min(std::max({a.y(), b.y(), c.y(), d.y()}), geofence_max.y);
                frontier_srv.request.max.z = std::min(frontier_srv.request.min.z+ std::abs(start.z() - end.z()) + range, geofence_max.z);


                octomath::Vector3 a_o(a.x(), a.y(), a.z());
                octomath::Vector3 b_o(b.x(), b.y(), b.z());
                octomath::Vector3 c_o(c.x(), c.y(), c.z());
                octomath::Vector3 d_o(d.x(), d.y(), d.z());
                visualization_msgs::MarkerArray marker_array;
                visualization_msgs::Marker marker;
                octomath::Vector3 start_o(start.x(), start.y(), start.z());
                rviz_interface::build_waypoint(start_o, 0.1, 0,   10, marker, 5);
                marker.ns = "start";
                marker_array.markers.push_back( marker );
                octomath::Vector3 end_o(end.x(), end.y(), end.z());
                rviz_interface::build_waypoint(end_o, 0.1, 0,   11, marker, 5);
                marker.ns = "end";
                marker_array.markers.push_back( marker );

                rviz_interface::build_waypoint(a_o, 0.5, 0,   1, marker, 1);
                marker.ns = "a";
                marker_array.markers.push_back( marker );
                rviz_interface::build_waypoint(b_o, 0.5, 250, 2, marker, 1);
                marker.ns = "b";
                marker_array.markers.push_back( marker );
                rviz_interface::build_waypoint(c_o, 0.5, 0  , 3, marker, 9);
                marker.ns = "c";
                marker_array.markers.push_back( marker );
                rviz_interface::build_waypoint(d_o, 0.5, 250, 4, marker, 9);
                marker.ns = "d";
                marker_array.markers.push_back( marker );
                rviz_interface::build_arrow_type(start_o, end_o, marker_array, 20, true);
                octomath::Vector3 end_ortho(start_o.x()+ortho.x(), start_o.y()+ortho.y(), start_o.z()+ortho.z());
                rviz_interface::build_arrow_type(start_o, end_ortho, marker_array, 21, false);
                

                ROS_INFO_STREAM("");
                // ROS_INFO_STREAM("Start = (" << start.x() << ", " << start.y() << ", " << start.z() << ")");
                // ROS_INFO_STREAM("End = (" << end.x() << ", " << end.y() << ", " << end.z() << ")");
                // ROS_INFO_STREAM("End Ortho = (" << end_ortho.x() << ", " << end_ortho.y() << ", " << end_ortho.z() << ")");
                
                ROS_INFO_STREAM("a = (" << a.x() << ", " << a.y() << ", " << a.z() << ")");
                ROS_INFO_STREAM("b = (" << b.x() << ", " << b.y() << ", " << b.z() << ")");

                ROS_INFO_STREAM("c = (" << c.x() << ", " << c.y() << ", " << c.z() << ")");
                ROS_INFO_STREAM("d = (" << d.x() << ", " << d.y() << ", " << d.z() << ")");

                // ROS_INFO_STREAM("Direction = (" << direction.x() << ", " << direction.y() << ", " << direction.z() << ")");
                // ROS_INFO_STREAM("Ortho = (" << ortho.x() << ", " << ortho.y() << ", " << ortho.z() << ")");

                ROS_INFO_STREAM("Geofence min " << geofence_min);
                ROS_INFO_STREAM("Geofence max " << geofence_max);


                visualization_msgs::Marker marker_2;
                octomath::Vector3 g_min(geofence_min.x, geofence_min.y, geofence_min.z);
                octomath::Vector3 g_max(geofence_max.x, geofence_max.y, geofence_max.z);
                rviz_interface::build_geofence (g_min, g_max, marker_2, 111, "global", 1, 0, 0);
                marker_array.markers.push_back( marker_2 );

                // ROS_INFO_STREAM( "std::max(std::min({" << a.x() << ", " << b.x() << ", " << c.x() << ", " << d.x() << "}), " << geofence_min.x << ")" );
                // ROS_INFO_STREAM("Request " << frontier_srv.request);
                visualization_msgs::Marker marker_3;
                octomath::Vector3 min(frontier_srv.request.min.x, frontier_srv.request.min.y, frontier_srv.request.min.z);
                octomath::Vector3 max(frontier_srv.request.max.x, frontier_srv.request.max.y, frontier_srv.request.max.z);
                rviz_interface::build_geofence (min, max, marker_3, 100, "local", 1, 1, 0);
                marker_array.markers.push_back( marker_3 );
                marker_pub.publish(marker_array);
        }

        void localFence_callback (const frontiers_msgs::LocalGeofenceRequest::ConstPtr& request)
        {

                Eigen::Vector3d start(request->start.x, request->start.y, request->start.z);
                Eigen::Vector3d end(request->end.x, request->end.y, request->end.z);
                fillLocalGeofence(start, end, request->range);
        }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ltstar_debug_node");
	ros::NodeHandle nh;
	frontiers_debug_node::marker_pub = nh.advertise<visualization_msgs::MarkerArray>("debug_markers", 1);
	ros::Subscriber neighbor_sub = nh.subscribe<frontiers_msgs::VoxelMsg>("/neighbor_generation", 10, frontiers_debug_node::neighbor_callback);
        ros::Subscriber geofence_sub = nh.subscribe<frontiers_msgs::LocalGeofenceRequest>("/local_geofence_request", 10, frontiers_debug_node::localFence_callback);
	ros::spin();
}