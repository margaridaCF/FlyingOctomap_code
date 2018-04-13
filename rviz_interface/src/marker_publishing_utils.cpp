
#include <marker_publishing_utils.h>

namespace rviz_interface
{

    void init_point(geometry_msgs::Point & point, float x, float y, float z)
    {
        point.x = x;
        point.y = y;
        point.z = z;
    }

    void push_segment(visualization_msgs::Marker & marker, geometry_msgs::Point & start, geometry_msgs::Point & end)
    {
        marker.points.push_back(start);
        marker.points.push_back(end);
    }

    void publish_cube_wire(visualization_msgs::Marker & marker, octomath::Vector3 const& geofence_min, octomath::Vector3 const& geofence_max, ros::Publisher const& marker_pub)
    {
        uint32_t shape = visualization_msgs::Marker::LINE_LIST; 
        // Set the frame ID and timestamp.  See the TF tutorials for information on these. 
        marker.header.frame_id = "/map"; 
        marker.header.stamp = ros::Time::now(); 
        marker.type = shape; 
        marker.action = visualization_msgs::Marker::ADD; 
        marker.scale.x = 0.2; 
        marker.scale.y = 0.2; 
        marker.scale.z = 0.2; 
        marker.color.r = 0.0f; 
        marker.color.g = 0.0f; 
        marker.color.b = 1.0f; 
        marker.color.a = 1.0; 
        geometry_msgs::Point A, B, C, D, E, F, G, H; 
        init_point( A, geofence_min.x(), geofence_min.y(), geofence_min.z()); 
        init_point( B, geofence_max.x(), geofence_min.y(), geofence_min.z()); 
        init_point( C, geofence_min.x(), geofence_max.y(), geofence_min.z()); 
        init_point( D, geofence_max.x(), geofence_max.y(), geofence_min.z()); 
        init_point( E, geofence_min.x(), geofence_max.y(), geofence_max.z()); 
        init_point( F, geofence_max.x(), geofence_max.y(), geofence_max.z()); 
        init_point( G, geofence_min.x(), geofence_min.y(), geofence_max.z()); 
        init_point( H, geofence_max.x(), geofence_min.y(), geofence_max.z()); 
        push_segment(marker, A, B); 
        push_segment(marker, A, G); 
        push_segment(marker, A, C); 
        push_segment(marker, B, H); 
        push_segment(marker, B, D); 
        push_segment(marker, G, H); 
        push_segment(marker, H, F); 
        push_segment(marker, C, D); 
        push_segment(marker, C, E); 
        push_segment(marker, F, D); 
        push_segment(marker, G, E); 
        push_segment(marker, E, F); 
        marker_pub.publish(marker); 
    }

	void publish_geofence(octomath::Vector3 const& geofence_min, octomath::Vector3 const& geofence_max, ros::Publisher const& marker_pub) 
    { 
        visualization_msgs::Marker marker; 
        // Set the frame ID and timestamp.  See the TF tutorials for information on these. 
        marker.ns = "geofence"; 
        marker.id = 20; 
        marker.lifetime = ros::Duration(); 
        publish_cube_wire(marker, geofence_min, geofence_max, marker_pub);
    } 

    void publish_marker_safety_margin(geometry_msgs::Point const& frontier, double safety_margin, ros::Publisher const& marker_pub, int id)
    {   
        visualization_msgs::Marker marker;
        octomath::Vector3  max = octomath::Vector3(frontier.x - safety_margin, frontier.y - safety_margin, frontier.z - safety_margin);
        octomath::Vector3  min = octomath::Vector3(frontier.x + safety_margin, frontier.y + safety_margin, frontier.z + safety_margin);
        marker.lifetime = ros::Duration(7);
        marker.ns = "safety_margin";
        marker.id = id;
        publish_cube_wire(marker, min, max, marker_pub);
    }

    void publish_deleteAll(ros::Publisher const& marker_pub)
    {
        visualization_msgs::Marker marker;
        marker.action = visualization_msgs::Marker::DELETEALL;
        marker_pub.publish(marker); 
    }

    void publish_voxel_free_occupied(octomath::Vector3 & candidate, bool is_occupied, ros::Publisher const& marker_pub, int id, double size)
    {
        uint32_t shape = visualization_msgs::Marker::CUBE;
        visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "neighbor_frontier";
        marker.id = id;
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = candidate.x();
        marker.pose.position.y = candidate.y();
        marker.pose.position.z = candidate.z();
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = size;
        marker.scale.y = size;
        marker.scale.z = size;
        if(is_occupied)
        {
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;
        }
        else
        {
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;
        }
        marker.lifetime = ros::Duration(2);
        marker_pub.publish(marker);
    }

    void publish_marker(octomath::Vector3 & candidate, bool is_occupied, ros::Publisher const& marker_pub, int id, double size)
    {
        uint32_t shape = visualization_msgs::Marker::CUBE;
        visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "neighbor_frontier";
        marker.id = id;
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = candidate.x();
        marker.pose.position.y = candidate.y();
        marker.pose.position.z = candidate.z();
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = size;
        marker.scale.y = size;
        marker.scale.z = size;
        if(is_occupied)
        {
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;
        }
        else
        {
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;
        }
        marker.lifetime = ros::Duration(2);
        marker_pub.publish(marker);
    }
}