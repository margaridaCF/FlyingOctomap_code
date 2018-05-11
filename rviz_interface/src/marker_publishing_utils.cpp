
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

    void build_cube_wire(visualization_msgs::Marker & marker, octomath::Vector3 const& geofence_min, octomath::Vector3 const& geofence_max, ros::Publisher const& marker_pub, octomath::Vector3 color = octomath::Vector3(1, 1, 1))
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
        marker.color.r = color.x(); 
        marker.color.g = color.y(); 
        marker.color.b = color.z(); 
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
    }

	void publish_geofence(octomath::Vector3 const& geofence_min, octomath::Vector3 const& geofence_max, ros::Publisher const& marker_pub) 
    { 
        visualization_msgs::Marker marker; 
        // Set the frame ID and timestamp.  See the TF tutorials for information on these. 
        marker.ns = "geofence"; 
        marker.id = 20; 
        marker.lifetime = ros::Duration(); 
        build_cube_wire(marker, geofence_min, geofence_max, marker_pub);
        marker_pub.publish(marker); 
    } 

    void publish_marker_safety_margin(geometry_msgs::Point const& frontier, double safety_margin, ros::Publisher const& marker_pub, int id)
    {   
        visualization_msgs::Marker marker;
        octomath::Vector3  max = octomath::Vector3(frontier.x - safety_margin, frontier.y - safety_margin, frontier.z - safety_margin);
        octomath::Vector3  min = octomath::Vector3(frontier.x + safety_margin, frontier.y + safety_margin, frontier.z + safety_margin);
        marker.lifetime = ros::Duration();
        marker.ns = "frontier_safety_margin";
        marker.id = id;
        build_cube_wire(marker, min, max, marker_pub);
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.push_back(marker);
        marker_pub.publish(marker_array); 
    }

    void publish_deleteAll(ros::Publisher const& marker_pub)
    {
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker marker;
        marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(marker);
        marker_pub.publish(marker_array); 
    }

    void publish_voxel_free_occupied(octomath::Vector3 & candidate, bool is_occupied, ros::Publisher const& marker_pub, int id, double size, visualization_msgs::Marker & marker)
    {
        uint32_t shape = visualization_msgs::Marker::CUBE;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "frontier_neighborhood";
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
            marker.color.a = 0.8;
        }
        else
        {
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.8;
        }
        marker.lifetime = ros::Duration(4);
    }

    void build_small_marker(octomath::Vector3 const& candidate, visualization_msgs::Marker & marker, float red, float green, float blue, std::string ns, int id, double size = 0.2f, double alpha = 1)
    {
        uint32_t shape = visualization_msgs::Marker::CUBE;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
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
        marker.color.r = red;
        marker.color.g = green;
        marker.color.b = blue;
        marker.color.a = alpha;
    
        marker.lifetime = ros::Duration();
    }

    void publish_small_marker(octomath::Vector3 const& candidate, ros::Publisher const& marker_pub, float red, float green, float blue, std::string ns, int id)
    {
        visualization_msgs::Marker marker;
        build_small_marker(candidate, marker, red, green, blue, ns, id);
        marker_pub.publish(marker);
    }

    void publish_current_position(octomath::Vector3 & candidate, ros::Publisher const& marker_pub)
    {
        float red = 0.0f;
        float green = 1.0f;
        float blue = 1.0f;
        publish_small_marker(candidate, marker_pub,red,  green,  blue, "current_position", 21);
    }

    void publish_start(geometry_msgs::Point const& candidate, ros::Publisher const& marker_pub)
    {
        octomath::Vector3 candidate_vec3 (candidate.x, candidate.y, candidate.z);
        float red = 1.0f;
        float green = 1.0f;
        float blue = 0.0f;
        publish_small_marker(candidate_vec3, marker_pub,red,  green,  blue, "start", 22);
    }

    void publish_goal(geometry_msgs::Point const& candidate, ros::Publisher const& marker_pub)
    {
        octomath::Vector3 candidate_vec3 (candidate.x, candidate.y, candidate.z);
        float red = 1.0f;
        float green = 0.5f;
        float blue = 0.0f;
        publish_small_marker(candidate_vec3, marker_pub,red,  green,  blue, "goal", 23);
    }


    void publish_sensing_position(octomath::Vector3 const& position, ros::Publisher const& marker_pub)
    {
        float red = 0.0f;
        float green = 0.0f;
        float blue = 0.25f;
        publish_small_marker(position, marker_pub,red,  green,  blue, "sensing_position", 24);
    }

    void publish_start_voxel(geometry_msgs::Point const& candidate, ros::Publisher const& marker_pub, double size)
    {
        visualization_msgs::Marker marker;
        octomath::Vector3 candidate_vec3 (candidate.x, candidate.y, candidate.z);
        float red = 1.0f;
        float green = 1.0f;
        float blue = 0.0f;
        build_small_marker(candidate_vec3, marker, red, green,  blue, "start_voxel", 25, size, 0.3  );
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.push_back(marker);
        marker_pub.publish(marker_array);
    }

    void publish_goal_voxel(geometry_msgs::Point const& candidate, ros::Publisher const& marker_pub, double size)
    {
        visualization_msgs::Marker marker;
        octomath::Vector3 candidate_vec3 (candidate.x, candidate.y, candidate.z);
        float red = 1.0f;
        float green = 0.5f;
        float blue = 0.0f;
        build_small_marker(candidate_vec3, marker, red, green,  blue, "goal_voxel", 26, size, 0.3);
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.push_back(marker);
        marker_pub.publish(marker_array);
    }

    void publish_s(geometry_msgs::Point const& candidate, ros::Publisher const& marker_pub)
    {
        octomath::Vector3 candidate_vec3 (candidate.x, candidate.y, candidate.z);
        float red = 0.9f;
        float green = 0.5f;
        float blue = 1.0f;
        visualization_msgs::Marker marker;
        int id = 5000 + ( std::rand() % ( 999 + 1 ) );
        build_small_marker(candidate_vec3, marker, red,  green,  blue, "s", id);
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.push_back(marker);
        marker_pub.publish(marker_array);   
    }

    void publish_visible_neighbor(octomath::Vector3 const& candidate_vec3, ros::Publisher const& marker_pub)
    {
        float red = 1.f;
        float green = 1.f;
        float blue = 0.f;
        float size = 0.1f;
        int id = 30000 + ( std::rand() % ( 9999 + 1 ) );
        uint32_t shape = visualization_msgs::Marker::SPHERE;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "visible neighbor";
        marker.id = id;
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = candidate_vec3.x();
        marker.pose.position.y = candidate_vec3.y();
        marker.pose.position.z = candidate_vec3.z();
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = size;
        marker.scale.y = size;
        marker.scale.z = size;
        marker.color.r = red;
        marker.color.g = green;
        marker.color.b = blue;
        marker.color.a = 1;
        marker.lifetime = ros::Duration();

        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.push_back(marker);
        marker_pub.publish(marker_array);   
    }

    void publish_frontier_marker(geometry_msgs::Point const& candidate, bool is_frontier, ros::Publisher const& marker_pub)
    {
        octomath::Vector3 candidate_vec3 (candidate.x, candidate.y, candidate.z);
        publish_frontier_marker(candidate_vec3, is_frontier, marker_pub);
    }
    
    void publish_frontier_marker(octomath::Vector3 const& candidate, bool is_frontier, ros::Publisher const& marker_pub)
    {
        uint32_t shape = visualization_msgs::Marker::CUBE;
        visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "frontier_candidate";
        marker.id = 10;
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = candidate.x();
        marker.pose.position.y = candidate.y();
        marker.pose.position.z = candidate.z();
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        if(is_frontier)
        {
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;
            marker.color.a = 1.0;
        }
        else
        {
            marker.color.r = 0.5f;
            marker.color.g = 0.5f;
            marker.color.b = 0.75f;
            marker.color.a = 1.0;
        }
        marker.lifetime = ros::Duration();
        // ROS_WARN_STREAM("[RVIZ PUB] Frontier at " << marker.pose.position << ". Color: " << marker.color.r << ", " << marker.color.g << ", " << marker.color.b);
        marker_pub.publish(marker);
    }

    void build_arrow_path(octomath::Vector3 & start, octomath::Vector3 & goal, int request_id, visualization_msgs::Marker & marker)
    {
        uint32_t shape = visualization_msgs::Marker::ARROW;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "lazy_theta_star_path";
        marker.id = request_id;
        marker.type = shape;
        geometry_msgs::Point goal_point;
        goal_point.x = goal.x();
        goal_point.y = goal.y();
        goal_point.z = goal.z();
        marker.points.push_back(goal_point);
        marker.action = visualization_msgs::Marker::ADD;
        geometry_msgs::Point start_point;
        start_point.x = start.x();
        start_point.y = start.y();
        start_point.z = start.z();
        marker.points.push_back(start_point);
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.3;
        marker.scale.z = 0;
        marker.color.r = 200;
        marker.color.g = 100;
        marker.color.b = 0;
        marker.color.a = 1;
        
        marker.lifetime = ros::Duration();
    }

    void publish_arrow_path_occupancyState(octomath::Vector3 const& start, octomath::Vector3 const& goal, ros::Publisher const& marker_pub, bool free)
    {
        // ROS_WARN_STREAM("publish_arrow_path_occupancyState");
        visualization_msgs::Marker marker;
        uint32_t shape = visualization_msgs::Marker::ARROW;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "corridor_occupancy";
        marker.id = 10000 + ( std::rand() % ( 9999 + 1 ) );
        marker.type = shape;
        geometry_msgs::Point goal_point;
        goal_point.x = goal.x();
        goal_point.y = goal.y();
        goal_point.z = goal.z();
        marker.points.push_back(goal_point);
        marker.action = visualization_msgs::Marker::ADD;
        geometry_msgs::Point start_point;
        start_point.x = start.x();
        start_point.y = start.y();
        start_point.z = start.z();
        marker.points.push_back(start_point);
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.3;
        marker.scale.z = 0;
        if(free)
        {
            marker.color.r = 0;
            marker.color.g = 255;   
        }
        else
        {
            marker.color.r = 255;
            marker.color.g = 0;   
        }
        marker.color.b = 0;
        marker.color.a = 1;
        
        marker.lifetime = ros::Duration();
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.push_back(marker);
        marker_pub.publish(marker_array);
    }

    void publish_arrow_path_occupied(octomath::Vector3 const& start, octomath::Vector3 const& goal, ros::Publisher const& marker_pub)
    {
        // ROS_WARN_STREAM("publish_arrow_path_occupancyState");
        visualization_msgs::Marker marker;
        uint32_t shape = visualization_msgs::Marker::ARROW;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "corridor_occupied";
        marker.id = 400;
        marker.type = shape;
        geometry_msgs::Point goal_point;
        goal_point.x = goal.x();
        goal_point.y = goal.y();
        goal_point.z = goal.z();
        marker.points.push_back(goal_point);
        marker.action = visualization_msgs::Marker::ADD;
        geometry_msgs::Point start_point;
        start_point.x = start.x();
        start_point.y = start.y();
        start_point.z = start.z();
        marker.points.push_back(start_point);
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.3;
        marker.scale.z = 0;
        marker.color.r = 255;
        marker.color.g = 140;   
        marker.color.b = 50;
        marker.color.a = 1;
        
        marker.lifetime = ros::Duration();
        // visualization_msgs::MarkerArray marker_array;
        // marker_array.markers.push_back(marker);
        marker_pub.publish(marker);
    }


    void publish_arrow_path_father(octomath::Vector3 const& start, octomath::Vector3 const& goal, ros::Publisher const& marker_pub)
    {
        // ROS_WARN_STREAM("publish_arrow_path_occupancyState");
        visualization_msgs::Marker marker;
        uint32_t shape = visualization_msgs::Marker::ARROW;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "parent";
        marker.id = 20000 + ( std::rand() % ( 9999 + 1 ) );;
        marker.type = shape;
        geometry_msgs::Point goal_point;
        goal_point.x = goal.x();
        goal_point.y = goal.y();
        goal_point.z = goal.z();
        marker.points.push_back(goal_point);
        marker.action = visualization_msgs::Marker::ADD;
        geometry_msgs::Point start_point;
        start_point.x = start.x();
        start_point.y = start.y();
        start_point.z = start.z();
        marker.points.push_back(start_point);
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.01;
        marker.scale.y = 0.03;
        marker.scale.z = 0;
        marker.color.r = 0;
        marker.color.g = 255;   
        marker.color.b = 255;
        marker.color.a = 1;
        
        marker.lifetime = ros::Duration();
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.push_back(marker);
        marker_pub.publish(marker_array);
    }

    void publish_arrow_corridor(octomath::Vector3 const& start, octomath::Vector3 const& goal, ros::Publisher const& marker_pub)
    {
        // ROS_WARN_STREAM("publish_arrow_path_occupancyState");
        visualization_msgs::Marker marker;
        uint32_t shape = visualization_msgs::Marker::ARROW;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
        marker.id = 50000 + ( std::rand() % ( 9999 + 1 ) );;
        marker.ns = "corridor_";
        marker.type = shape;
        geometry_msgs::Point goal_point;
        goal_point.x = goal.x();
        goal_point.y = goal.y();
        goal_point.z = goal.z();
        marker.points.push_back(goal_point);
        marker.action = visualization_msgs::Marker::ADD;
        geometry_msgs::Point start_point;
        start_point.x = start.x();
        start_point.y = start.y();
        start_point.z = start.z();
        marker.points.push_back(start_point);
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.01;
        marker.scale.y = 0.03;
        marker.scale.z = 0;
        marker.color.r = 255;
        marker.color.g = 255;   
        marker.color.b = 255;
        marker.color.a = 1;
        
        marker.lifetime = ros::Duration();
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.push_back(marker);
        marker_pub.publish(marker_array);
    }

    void publish_arrow_corridor_center(octomath::Vector3 const& start, octomath::Vector3 const& goal, ros::Publisher const& marker_pub)
    {
        // ROS_WARN_STREAM("publish_arrow_path_occupancyState");
        visualization_msgs::Marker marker;
        uint32_t shape = visualization_msgs::Marker::ARROW;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
        marker.id = 60000 + ( std::rand() % ( 9999 + 1 ) );;
        marker.ns = "corridor_center";
        marker.type = shape;
        geometry_msgs::Point goal_point;
        goal_point.x = goal.x();
        goal_point.y = goal.y();
        goal_point.z = goal.z();
        marker.points.push_back(goal_point);
        marker.action = visualization_msgs::Marker::ADD;
        geometry_msgs::Point start_point;
        start_point.x = start.x();
        start_point.y = start.y();
        start_point.z = start.z();
        marker.points.push_back(start_point);
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.01;
        marker.scale.y = 0.03;
        marker.scale.z = 0;
        marker.color.r = 255;
        marker.color.g = 255;   
        marker.color.b = 0;
        marker.color.a = 1;
        marker.lifetime = ros::Duration();
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.push_back(marker);
        marker_pub.publish(marker_array);
    }

    void build_waypoint(octomath::Vector3 & candidate, double size, int color, int waypoint_id, visualization_msgs::Marker & marker)
    {   
        uint32_t shape = visualization_msgs::Marker::CUBE;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "lazy_theta_star_waypoint";
        marker.id = waypoint_id;
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = candidate.x();
        marker.pose.position.y = candidate.y();
        marker.pose.position.z = candidate.z();
        marker.pose.orientation.w = 1.0;
        marker.scale.x = size;
        marker.scale.y = size;
        marker.scale.z = size;
        marker.color.r = 0.9f;
        marker.color.g = color+0.4;
        marker.color.b = 1.0f;
        // ROS_WARN_STREAM("[RVIZ PUB] color " << marker.color.r << ", " << marker.color.g << ", " << marker.color.b << " i: " << waypoint_id);
        marker.color.a = 0.8;
        
        marker.lifetime = ros::Duration();
    }
}