/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/TwistStamped.h>

namespace state_manager_node
{
    int request_count;
    bool goal_set, waypoint_list_set;
    struct StateData { 
        int reply_seq_id; 
        bool goal_set; 
        bool waypoint_list_set; 
        bool fully_explored; 
        double x; 
        double y; 
        double z;};
    std::atomic<StateData> state_data_atomic;

    void askForGoal(int request_count, octomath::Vector3 const& geofence_min, octomath::Vector3 const& geofence_max, ros::Publisher const& frontier_request_pub)
    {
        frontiers_msgs::FrontierRequest frontier_request;
        request.header.seq = request_count;
        request.header.frame_id = "world";
        request.min.x = geofence_min.x();
        request.min.y = geofence_min.y();
        request.min.z = geofence_min.z();
        request.max.x = geofence_max.x();
        request.max.y = geofence_max.y();
        request.max.z = geofence_max.z();
        request.frontier_amount = 1;
        frontier_request_pub.publish();
        request_count++;
    }

    bool reachedGoal(geometry_msgs::PoseStamped const& pose)
    {
        return pose.pose.position.x == new_frontier.pose.position.x 
            && pose.pose.position.y == new_frontier.pose.position.y 
            && pose.pose.position.z == new_frontier.pose.position.z;
    }

    void stop_cb(const std_msgs::Empty::ConstPtr& msg)
    {
        StateData temp_state_data;
        temp_state_data.fully_explored = false;
        temp_state_data.goal_set = false;
        temp_state_data.waypoint_list_set = false;      // TODO change flags with waypoint sequence
        state_data_atomic.store(temp_state_data); 
    }

    void frontier_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
        StateData temp_state_data;
        if(msg->frontiers_found == 0)
        {
            temp_state_data.fully_explored = true;
            temp_state_data.goal_set = false;
            temp_state_data.waypoint_list_set = false;
        }
        else
        {
            temp_state_data.reply_seq_id = msg->request_id;
            temp_state_data.goal_set = false;
            temp_state_data.goal_set = true;            // TODO change flags with waypoint sequence
            temp_state_data.waypoint_list_set = true;   // TODO change flags with waypoint sequence
            temp_state_data.x = msg->frontiers[0].x;    
            temp_state_data.y = msg->frontiers[0].y;    
            temp_state_data.z = msg->frontiers[0].z;   
            state_data_atomic.store(temp_state_data); 
            ROS_INFO_STREAM("[Satate manager node] New frontier ("
                <<temp_state_data.x << ", "
                <<temp_state_data.y << ", "
                <<temp_state_data.z << ") ");
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_manager");
    ros::NodeHandle nh;
    ros::Subscriber stop_sub = nh.subscribe<std_msgs::Empty>("/stop_uav", 10, stop_cb);
    ros::Subscriber frontiers_reply_sub = nh.subscribe<frontiers_msgs::FrontierReply>("frontiers_reply", 10, frontier_cb);
    ros::Publisher frontier_request_pub = nh.advertise<frontiers_msgs::FrontierRequest>("frontiers_request", 10);
    ros::Publisher target_position_pub = nh.advertise<geometry_msgs::PointStamped>("target_position", 10);
    

    geometry_msgs::PoseStamped pose;
    state_manager_node::StateData temp_state_data;
    temp_state_data.reply_seq_id = 0;
    temp_state_data.fully_explored = false;
    temp_state_data.goal_set = false;
    temp_state_data.waypoint_list_set = false; 
    state_data_atomic.store(temp_state_data);
    // TODO Lazy theta star topics
    octomath::Vector3 geofence_min (0, 0, 0);
    octomath::Vector3 geofence_max (6, 2, 2);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20);
    ros::Time last_request = ros::Time::now();
    bool all_done = false;
    while(ros::ok() && !all_done) 
    {
        temp_state_data = state_data_atomic.load();
        // === Is all space explored? ===
        if(temp_state_data.fully_explored)
        {
            ROS_INFO_STREAM("[State Manager] Finished exploration!");
            all_done = true;
        }
        // === Is the goal defined? ===
        else if(!temp_state_data.goal_set)
        {
            state_manager_node::askForGoal(request_count, geofence_min, geofence_max, frontier_request_pub);
        }
        // else if(!temp_state_data.waypoint_list_set)
        // {
        //     state_manager_node::askObstacleAvoidingPath();
        // }
        else
        {
            pose.pose.position.x = temp_state_data.x;
            pose.pose.position.y = temp_state_data.y;
            pose.pose.position.z = temp_state_data.z;
            local_pos_pub.publish(next_waypoint);
        }    
        
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
