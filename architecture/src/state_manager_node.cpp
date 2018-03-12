/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/TwistStamped.h>
#include <octomap/math/Vector3.h>

#include <architecture_msgs/PositionRequest.h>

#include <frontiers_msgs/FrontierReply.h>
#include <frontiers_msgs/FrontierRequest.h>

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
    StateData state_data;

    void askForGoal(int request_count, octomath::Vector3 const& geofence_min, octomath::Vector3 const& geofence_max, ros::Publisher const& frontier_request_pub)
    {
        frontiers_msgs::FrontierRequest request;
        request.header.seq = request_count;
        request.header.frame_id = "world";
        request.min.x = geofence_min.x();
        request.min.y = geofence_min.y();
        request.min.z = geofence_min.z();
        request.max.x = geofence_max.x();
        request.max.y = geofence_max.y();
        request.max.z = geofence_max.z();
        request.frontier_amount = 1;
        frontier_request_pub.publish(request);
        request_count++;
    }

    void stop_cb(const std_msgs::Empty::ConstPtr& msg)
    {
        state_data.fully_explored = false;
        state_data.goal_set = false;
        state_data.waypoint_list_set = false;      // TODO change flags with waypoint sequence
    }

    void frontier_cb(const frontiers_msgs::FrontierReply::ConstPtr& msg){
        if(msg->frontiers_found == 0)
        {
            state_data.fully_explored = true;
            state_data.goal_set = false;
            state_data.waypoint_list_set = false;
        }
        else
        {
            state_data.reply_seq_id = msg->request_id;
            state_data.goal_set = false;
            state_data.goal_set = true;            // TODO change flags with waypoint sequence
            state_data.waypoint_list_set = true;   // TODO change flags with waypoint sequence
            state_data.x = msg->frontiers[0].xyz_m.x;    
            state_data.y = msg->frontiers[0].xyz_m.y;    
            state_data.z = msg->frontiers[0].xyz_m.z;   
            ROS_INFO_STREAM("[Satate manager node] New frontier ("
                <<state_data.x << ", "
                <<state_data.y << ", "
                <<state_data.z << ") ");
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_manager");
    ros::NodeHandle nh;
    ros::Subscriber stop_sub = nh.subscribe<std_msgs::Empty>("/stop_uav", 10, state_manager_node::stop_cb);
    ros::Subscriber frontiers_reply_sub = nh.subscribe<frontiers_msgs::FrontierReply>("frontiers_reply", 10, state_manager_node::frontier_cb);
    ros::Publisher frontier_request_pub = nh.advertise<frontiers_msgs::FrontierRequest>("frontiers_request", 10);
    ros::Publisher target_position_pub = nh.advertise<architecture_msgs::PositionRequest>("target_position", 10);
    

    state_manager_node::StateData state_data;
    state_data.reply_seq_id = 0;
    state_data.fully_explored = false;
    state_data.goal_set = false;
    state_data.waypoint_list_set = false; 
    // TODO Lazy theta star topics
    octomath::Vector3 geofence_min (0, 0, 0);
    octomath::Vector3 geofence_max (2, 2, 2);
    state_manager_node::request_count = 1;
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20);
    ros::Time last_request = ros::Time::now();
    bool all_done = false;
    while(ros::ok() && !all_done) 
    {
            // === Is all space explored? ===
        if(state_data.fully_explored)
        {
            ROS_INFO_STREAM("[State Manager] Finished exploration!");
            all_done = true;
        }
        // === Is the goal defined? ===
        else if(!state_data.goal_set)
        {
            state_manager_node::askForGoal(state_manager_node::request_count, geofence_min, geofence_max, frontier_request_pub);
        }
        // else if(!state_data.waypoint_list_set)
        // {
        //     state_manager_node::askObstacleAvoidingPath();
        // }
        else
        {
            architecture_msgs::PositionRequest position_request;
            position_request.waypoint_sequence_id = state_data.reply_seq_id;
            position_request.position.x = state_data.x;
            position_request.position.y = state_data.y;
            position_request.position.z = state_data.z;
            target_position_pub.publish(position_request);
        }    
        
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
