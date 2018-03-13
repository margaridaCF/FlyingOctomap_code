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
#include <frontiers_msgs/FrontierNodeStatus.h>

namespace state_manager_node
{
    
    enum follow_path_state_t{init, on_route, reached_waypoint, finished_sequence};
    enum exploration_state_t {exploration_start, choosing_goal, generating_path, visit_waypoints, finished_exploring};
    struct StateData { 
        int reply_seq_id; 
        int request_count;
        exploration_state_t exploration_state;
        follow_path_state_t follow_path_state;
        frontiers_msgs::FrontierReply frontiers_msg;
        int sequence_progress;
    };  

    state_manager_node::StateData state_data;
    ros::Publisher target_position_pub;
    ros::Publisher frontier_request_pub;
    ros::ServiceClient frontier_status_client;

    void askForGoal(int request_count, octomath::Vector3 const& geofence_min, octomath::Vector3 const& geofence_max, ros::Publisher const& frontier_request_pub)
    {
        frontiers_msgs::FrontierRequest request;
        request.header.seq = state_data.request_count;
        request.header.frame_id = "world";
        request.min.x = geofence_min.x();
        request.min.y = geofence_min.y();
        request.min.z = geofence_min.z();
        request.max.x = geofence_max.x();
        request.max.y = geofence_max.y();
        request.max.z = geofence_max.z();
        request.frontier_amount = 1;
        frontier_request_pub.publish(request);
        state_data.request_count++;
    }

    void stop_cb(const std_msgs::Empty::ConstPtr& msg)
    {
        state_data.exploration_state = exploration_start;
        ROS_INFO_STREAM("[State manager] switching to exploration_start");
    }


    void frontier_cb(const frontiers_msgs::FrontierReply::ConstPtr& msg){
        if(msg->frontiers_found > 0 && state_data.exploration_state == exploration_start)
        {
            state_data.reply_seq_id = msg->request_id;
            state_data.exploration_state = generating_path;
            state_data.frontiers_msg = *msg;
            ROS_INFO_STREAM("[State manager] switching to generating_path");
            ROS_INFO_STREAM("[Satate manager node] New frontier ("
                <<state_data.frontiers_msg.frontiers[0].xyz_m.x << ", "
                <<state_data.frontiers_msg.frontiers[0].xyz_m.y << ", "
                <<state_data.frontiers_msg.frontiers[0].xyz_m.z << ") ");
        }
    }


    bool is_goal_reached()
    {
        switch(state_data.follow_path_state)
        {
            case init:
            {
                state_data.sequence_progress = 0;
                architecture_msgs::PositionRequest position_request;
                position_request.waypoint_sequence_id = state_data.reply_seq_id;
                position_request.position.x = state_data.frontiers_msg.frontiers[state_data.sequence_progress].xyz_m.x;
                position_request.position.y = state_data.frontiers_msg.frontiers[state_data.sequence_progress].xyz_m.y;
                position_request.position.z = state_data.frontiers_msg.frontiers[state_data.sequence_progress].xyz_m.z;
                target_position_pub.publish(position_request);
                state_data.follow_path_state = on_route;
                ROS_INFO_STREAM("[State manager] Switching path follow state to on_route");
                break;
            }
            case on_route:
            {
                // TODO figure out how to check if destination was reached
                break;
            }
            case reached_waypoint:
            {    // TODO - no sequence for the moment so the sequence is finished
                state_data.follow_path_state = finished_sequence;
                ROS_INFO_STREAM("[State manager] Switching path follow state to finished_sequence");
                break;
            }
            case finished_sequence:
            {
                return true;
                break;
            }
        }
        return false;
        
    }

    void init_state_variables(state_manager_node::StateData& state_data)
    {
        state_data.request_count = 0;
        state_data.exploration_state = exploration_start;
        state_data.follow_path_state = init;
        ROS_INFO_STREAM("[State manager] Initializing state to exploration_start and follow path state to init");
    }

    void update_state(octomath::Vector3 const& geofence_min, octomath::Vector3 const& geofence_max)
    {
        switch(state_data.exploration_state)
        {
            case exploration_start:
            {
                frontiers_msgs::FrontierNodeStatus srv;
                if (frontier_status_client.call(srv))
                {
                    if((bool)srv.response.is_accepting_requests)
                    {
                        askForGoal(state_data.request_count, geofence_min, geofence_max, frontier_request_pub);
                    }
                }
                else
                {
                    ROS_WARN("[State manager] Frontier node not accepting requests.");
                }
                break;
            }
            case choosing_goal:
            {
                // TODO Pick one - this is currently being done as soon as the frontiers arrive
                state_data.exploration_state = generating_path;
                ROS_INFO_STREAM("[State manager] Switching to generating_path");
                break;
            }
            case generating_path:
            {
                // TODO Lazy Theta Star - just assume no obstacles
                state_data.follow_path_state = init;
                state_data.exploration_state = visit_waypoints;
                ROS_INFO_STREAM("[State manager] Switching to visit_waypoints and follow path state to init");
                break;
            }
            case visit_waypoints:
            {
                if (is_goal_reached())
                {
                    state_data.exploration_state = exploration_start;
                    ROS_INFO_STREAM("[State manager] switching to exploration_start");
                }
                break;
            }
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_manager");
    ros::NodeHandle nh;

    state_manager_node::frontier_status_client = nh.serviceClient<frontiers_msgs::FrontierNodeStatus>("frontier_status");

    ros::Subscriber stop_sub = nh.subscribe<std_msgs::Empty>("stop_uav", 10, state_manager_node::stop_cb);
    ros::Subscriber frontiers_reply_sub = nh.subscribe<frontiers_msgs::FrontierReply>("frontiers_reply", 10, state_manager_node::frontier_cb);
    state_manager_node::frontier_request_pub = nh.advertise<frontiers_msgs::FrontierRequest>("frontiers_request", 10);
    state_manager_node::target_position_pub = nh.advertise<architecture_msgs::PositionRequest>("target_position", 10);
    

    
    init_state_variables(state_manager_node::state_data);
    // TODO Lazy theta star topics
    octomath::Vector3 geofence_min (0, 0, 0);
    octomath::Vector3 geofence_max (2, 2, 2);
    ros::Rate rate(2);
    while(ros::ok() && state_manager_node::state_data.exploration_state != state_manager_node::finished_exploring) 
    {
        state_manager_node::update_state(geofence_min, geofence_max);
    
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
