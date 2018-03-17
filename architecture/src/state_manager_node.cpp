/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <octomap/math/Vector3.h>
#include <unordered_set>

#include <architecture_msgs/PositionRequest.h>
#include <architecture_msgs/PositionMiddleMan.h>

#include <frontiers_msgs/CheckIsFrontier.h>
#include <frontiers_msgs/FrontierReply.h>
#include <frontiers_msgs/FrontierRequest.h>
#include <frontiers_msgs/FrontierNodeStatus.h>
#include <frontiers_msgs/VoxelMsg.h>

#include <path_planning_msgs/LTStarReply.h>
#include <path_planning_msgs/LTStarRequest.h>
#include <path_planning_msgs/LTStarNodeStatus.h>


namespace state_manager_node
{

    struct Vector3Hash
    {
        std::size_t operator()(const octomath::Vector3 & v) const 
        {
            int scale = 0.1;
            std::size_t hx = std::hash<float>{}( (int)(v.x() / scale) * scale );
            std::size_t hy = std::hash<float>{}( (int)(v.y() / scale) * scale );
            std::size_t hz = std::hash<float>{}( (int)(v.z() / scale) * scale );
            std::size_t return_value = ((hx 
               ^ (hy << 1)) >> 1)
               ^ (hz << 1);
            return return_value;
        }
    };

    ros::Publisher target_position_pub;
    ros::Publisher frontier_request_pub;
    ros::Publisher ltstar_request_pub;
    ros::ServiceClient is_frontier_client;
    ros::ServiceClient frontier_status_client;
    ros::ServiceClient ltstar_status_cliente;
    ros::ServiceClient current_position_client;

    // TODO - transform this into parameters at some point
    double const px4_loiter_radius = 0.5;   // TODO - checkout where this is set
    double const odometry_error = 0;      // TODO - since it is simulation none
    double error_margin = std::max(px4_loiter_radius, odometry_error);
    double safety_margin = 1.5;
    int const max_search_iterations = 500;
    enum follow_path_state_t{init, on_route, reached_waypoint, finished_sequence};
    enum exploration_state_t {clear_from_ground, exploration_start, choosing_goal, generating_path, visit_waypoints, finished_exploring};
    struct StateData { 
        int reply_seq_id;       // id for the request in use
        int request_count;      // generate id for new frontier requests
        int sequence_progress;  // id of the waypoint that is currently the waypoint
        // int sequence_waypoint_count;  // amount of waypoints in sequence
        int frontier_id;        // id of the frontier in use
        exploration_state_t exploration_state;
        follow_path_state_t follow_path_state;
        frontiers_msgs::FrontierReply frontiers_msg;
        path_planning_msgs::LTStarReply ltstar_msg;
        std::unordered_set<octomath::Vector3, Vector3Hash> unobservable_set; 
    };  
    state_manager_node::StateData state_data;

    // TODO when thre is generation of path these two will be different
    geometry_msgs::Point& get_current_position()
    {
        // if(state_data.sequence_progress == -1)
        // {
        //     ROS_ERROR_STREAM("[State manager] Calling get_current_position, state_data.sequence_progress is " << state_data.sequence_progress);
        // }
        // return state_data.ltstar_msg.waypoints[state_data.sequence_progress];
        return state_data.frontiers_msg.frontiers[state_data.sequence_progress].xyz_m;
    }

    geometry_msgs::Point get_current_frontier()
    {
        return state_data.frontiers_msg.frontiers[state_data.frontier_id].xyz_m;
    }

    void requestingPostionOfCurrentWaypointSequence()
    {
        architecture_msgs::PositionRequest position_request;
        position_request.waypoint_sequence_id = state_data.reply_seq_id;
        position_request.position = get_current_position();
        target_position_pub.publish(position_request);
    }

    void askForObstacleAvoidingPath(octomath::Vector3 const& start, octomath::Vector3 const& goal, ros::Publisher const& ltstar_request_pub)
    {
        path_planning_msgs::LTStarRequest request;
        request.header.seq = state_data.request_count;
        request.header.frame_id = "world";
        request.start.x = start.x();
        request.start.y = start.y();
        request.start.z = start.z();
        request.goal.x = goal.x();
        request.goal.y = goal.y();
        request.goal.z = goal.z();
        request.max_search_iterations = max_search_iterations;
        ltstar_request_pub.publish(request);
    }

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
        request.frontier_amount = state_data.unobservable_set.size()+1;
        frontier_request_pub.publish(request);
        state_data.request_count++;
    }

    void stop_cb(const std_msgs::Empty::ConstPtr& msg)
    {
        state_data.exploration_state = exploration_start;
        ROS_INFO_STREAM("[State manager][Exploration] exploration_start");
    }

    void ltstar_cb(const path_planning_msgs::LTStarReply::ConstPtr& msg)
    {
        if(state_data.exploration_state == generating_path 
            && msg->request_id == state_data.reply_seq_id)
        {
            if(msg->success)
            {
                ROS_INFO_STREAM("[State manager] Received path from Lazy Theta Star " << *msg);
                // Update state variables
                // state_data.sequence_waypoint_count = msg.waypoint_amount;
                state_data.ltstar_msg = *msg;
                state_data.follow_path_state = init;
                state_data.exploration_state = visit_waypoints;
                state_data.sequence_progress = 1;
                ROS_INFO_STREAM("[State manager][Exploration] visit_waypoints");
                ROS_INFO_STREAM("[State manager]            [Follow path] init");
            }
            else
            {
                octomath::Vector3 unreachable (get_current_frontier().x, get_current_frontier().y, get_current_frontier().z);
                state_data.unobservable_set.insert(unreachable);
                state_data.exploration_state = choosing_goal;
                state_data.sequence_progress = -1;
                ROS_INFO_STREAM("[State manager][Exploration] choosing_goal");
            }
            
        }
    }

    void frontier_cb(const frontiers_msgs::FrontierReply::ConstPtr& msg){
        if(msg->frontiers_found == 0)
        {
            // TODO - go back to base and land
            ROS_INFO_STREAM("[State manager][Exploration] finished_exploring");
            state_data.exploration_state = finished_exploring;
        }
        else if(msg->frontiers_found > 0 && state_data.exploration_state == exploration_start)
        {
            state_data.reply_seq_id = msg->request_id;
            state_data.exploration_state = choosing_goal;
            state_data.frontiers_msg = *msg;
            ROS_INFO_STREAM("[State manager][Exploration] choosing_goal");
        }
    }

    bool is_in_target_position(geometry_msgs::Point const& target_waypoint, 
        geometry_msgs::Point & current_position, double error_margin )
    {
        // ROS_INFO_STREAM("[State manager] Target position " << target_waypoint );
        // ROS_INFO_STREAM("[State manager] Current position " << current_position );
        ROS_INFO_STREAM("[State manager] Position offset (" << std::abs(target_waypoint.x - current_position.x) << ", "
            << std::abs(target_waypoint.y - current_position.y) << ", "
            << std::abs(target_waypoint.z - current_position.z) << ") ");

        return std::abs(target_waypoint.x - current_position.x) <= error_margin
            && std::abs(target_waypoint.y - current_position.y) <= error_margin
            && std::abs(target_waypoint.z - current_position.z) <= error_margin;
    }

    void switch_to_finished_sequence()
    {
        if(state_data.ltstar_msg.waypoint_amount == state_data.sequence_progress+1)
        {
            // Reached Frontier
            state_data.sequence_progress = -1;
            state_data.follow_path_state = finished_sequence;
            ROS_INFO_STREAM("[State manager]            [Path follow]  finished_sequence A");
        }
        else {
            state_data.sequence_progress++;
            requestingPostionOfCurrentWaypointSequence();
            // ROS_INFO_STREAM("[State manager]  state_data.ltstar_msg " << state_data.ltstar_msg);
            // TODO - Move to the next waypoit
            ROS_ERROR_STREAM("[State manager] Move to waypoit " << state_data.sequence_progress << " = " << get_current_position());
        }
    }

    bool getUavPositionServiceCall(geometry_msgs::Point& current_position)
    {
        architecture_msgs::PositionMiddleMan srv;
        if(current_position_client.call(srv))
        {
            current_position = srv.response.current_position;
            return true;
        }
        else
        {
            return false;
        }
    }


    bool updateWaypointSequenceStateMachine()
    {
        switch(state_data.follow_path_state)
        {
            case init:
            {
                requestingPostionOfCurrentWaypointSequence();
                state_data.follow_path_state = on_route;
                ROS_INFO_STREAM("[State manager]            [Path follow] on_route to " << get_current_position());
                break;
            }
            case on_route:
            {
                geometry_msgs::Point current_position;
                bool service_call_successfull = getUavPositionServiceCall(current_position);
                if(service_call_successfull)
                {
                    // compare target with postition allowing for error margin
                    geometry_msgs::Point target_waypoint;
                    target_waypoint = get_current_position();
                    if( is_in_target_position(target_waypoint, current_position, error_margin) )
                    {
                        switch_to_finished_sequence();
                    }
                }
                break;
            }
            case reached_waypoint:
            {    // TODO - no sequence for the moment so the sequence is finished
                state_data.follow_path_state = finished_sequence;
                ROS_ERROR_STREAM("[State manager]            [Path follow] finished_sequence");
                break;
            }
        }
    }

    bool chooseFrontier()
    {
        geometry_msgs::Point candidate_frontier;
        std::unordered_set<octomath::Vector3, Vector3Hash>::const_iterator unobservable_frontier;
        // Pick one the is not in the unobservable
        for(int i = 0; i < state_data.frontiers_msg.frontiers_found; i++)
        {
            candidate_frontier = state_data.frontiers_msg.frontiers[i].xyz_m;
            octomath::Vector3 candidate_frontier_vector (candidate_frontier.x, candidate_frontier.y, candidate_frontier.z);
            unobservable_frontier = state_data.unobservable_set.find (candidate_frontier_vector);
            if ( unobservable_frontier == state_data.unobservable_set.end() )
            {
                // This frontier hasn't been explored yet. Let's pick this one
                state_data.reply_seq_id = state_data.frontiers_msg.request_id;
                state_data.sequence_progress = -1;
                state_data.frontier_id = i;
                ROS_INFO_STREAM("[Satate manager node] New frontier ("
                    <<get_current_frontier().x << ", "
                    <<get_current_frontier().y << ", "
                    <<get_current_frontier().z << ") ");
                return true;
            }
        }
        ROS_INFO_STREAM("[Satate manager node] Could not find an observable frontier.");
        return false;
    }

    void init_state_variables(state_manager_node::StateData& state_data)
    {
        state_data.request_count = 0;
        state_data.exploration_state = clear_from_ground;
        state_data.follow_path_state = init;
        ROS_INFO_STREAM("[State manager][Exploration] clear_from_ground");
        ROS_INFO_STREAM("[State manager]            [Path follow] init");
    }

    void update_state(octomath::Vector3 const& geofence_min, octomath::Vector3 const& geofence_max)
    {
        switch(state_data.exploration_state)
        {
            case clear_from_ground:
            {
                // Find current position
                architecture_msgs::PositionMiddleMan srv;
                if(current_position_client.call(srv))
                {
                    geometry_msgs::Point current_position = srv.response.current_position;
                    state_data.reply_seq_id = -1;

                    frontiers_msgs::VoxelMsg voxel_msg;
                    voxel_msg.xyz_m.x = current_position.x;
                    voxel_msg.xyz_m.y = current_position.y;
                    voxel_msg.xyz_m.z = current_position.z + 10;
                    voxel_msg.size = -1;
                    state_data.frontiers_msg.frontiers.push_back(voxel_msg);
                    voxel_msg.xyz_m.x = current_position.x;
                    voxel_msg.xyz_m.y = current_position.y;
                    voxel_msg.xyz_m.z = 2;
                    voxel_msg.size = -1;
                    state_data.frontiers_msg.frontiers.push_back(voxel_msg);
                    state_data.ltstar_msg.waypoint_amount = 2;
                    state_data.sequence_progress = 0;
                    state_data.frontiers_msg.frontiers_found = 1;
                    state_data.exploration_state = visit_waypoints;
                    state_data.follow_path_state = init;
                    ROS_INFO_STREAM("[State manager][Exploration] visit_waypoints");
                    ROS_INFO_STREAM("[State manager]            [Follow path] init");
                }
                else
                {
                    ROS_WARN("[State manager] Current position middle man node not accepting requests.");
                }

                break;
            }
            case exploration_start:
            {
                frontiers_msgs::FrontierNodeStatus srv;
                if (frontier_status_client.call(srv))
                {
                    if((bool)srv.response.is_accepting_requests)
                    {
                        // ROS_INFO_STREAM("[State manager] Asking for frontiers.");
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
                if(chooseFrontier()) 
                {
                    state_data.exploration_state = generating_path;
                    ROS_INFO_STREAM("[State manager][Exploration] generating_path");
                }
                else
                {
                    ROS_ERROR_STREAM("[State manager] Huston, we have a problem - all " 
                        << state_data.frontiers_msg.frontiers_found << " frontiers are unobservable.");
                    state_data.exploration_state = finished_exploring;                
                }

                break;
            }
            case generating_path:
            {
                path_planning_msgs::LTStarNodeStatus srv;
                if(ltstar_status_cliente.call(srv))
                {
                    if((bool)srv.response.is_accepting_requests)
                    {
                        geometry_msgs::Point current_position;
                        bool service_call_successfull = getUavPositionServiceCall(current_position);
                        if(service_call_successfull)
                        {
                            octomath::Vector3 start(current_position.x, current_position.y, current_position.z);
                            octomath::Vector3 goal (get_current_frontier().x, get_current_frontier().y, get_current_frontier().z);
                            askForObstacleAvoidingPath(start, goal, ltstar_request_pub);
                        }
                    }
                }
                else
                {
                    ROS_WARN("[State manager] Lazy Theta Star node not accepting requests.");
                }
                break;
            }
            case visit_waypoints:
            {
                updateWaypointSequenceStateMachine();
                if (state_data.follow_path_state == finished_sequence)
                {
                    ROS_INFO_STREAM("[State manager] Reached frontier to explore: " << get_current_frontier());

                    // Check if the frontier was observerd
                    // geometry_msgs::Point current_position;
                    // bool service_call_successfull = getUavPositionServiceCall(current_position);
                    // frontiers_msgs::CheckIsFrontier srv;
                    // srv.request.candidate = get_current_frontier();
                    // if (service_call_successfull && is_frontier_client.call(srv))
                    // {
                    //     ROS_INFO_STREAM("[State manager] Frontier node declares this point a frontier? " << (bool)srv.response.is_frontier);
                    //     if((bool)srv.response.is_frontier)
                    //     {
                    //         octomath::Vector3 frontier_vector (get_current_frontier().x, get_current_frontier().y, get_current_frontier().z);
                    //         state_data.unobservable_set.insert(frontier_vector);
                    //         ROS_WARN("[State manager] Unfortunatly this frontier was not observable, adding to the unobservable set. Size is now " << state_data.unobservable_set.size());
                    //     }
                    // }
                    // else
                    // {
                    //     ROS_WARN("[State manager] Frontier node not accepting requests.");
                    // }

                    state_data.exploration_state = exploration_start;
                    ROS_INFO_STREAM("[State manager][Exploration] exploration_start");
                }
                break;
            }
            default:
            {
                ROS_ERROR_STREAM("[State manager] Something went very wrong. State is unknown "<< state_data.exploration_state);
                break;
            }
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_manager");
    ros::NodeHandle nh;
    // Service client
    state_manager_node::ltstar_status_cliente = nh.serviceClient<path_planning_msgs::LTStarNodeStatus>("ltstar_status");
    state_manager_node::frontier_status_client = nh.serviceClient<frontiers_msgs::FrontierNodeStatus>("frontier_status");
    state_manager_node::is_frontier_client = nh.serviceClient<frontiers_msgs::CheckIsFrontier>("is_frontier");
    state_manager_node::current_position_client = nh.serviceClient<architecture_msgs::PositionMiddleMan>("get_current_position");
    // Topic subscribers
    ros::Subscriber stop_sub = nh.subscribe<std_msgs::Empty>("stop_uav", 5, state_manager_node::stop_cb);
    ros::Subscriber frontiers_reply_sub = nh.subscribe<frontiers_msgs::FrontierReply>("frontiers_reply", 5, state_manager_node::frontier_cb);
    ros::Subscriber ltstar_reply_sub = nh.subscribe<path_planning_msgs::LTStarReply>("ltstar_reply", 5, state_manager_node::ltstar_cb);
    // Topic publishers
    state_manager_node::ltstar_request_pub = nh.advertise<path_planning_msgs::LTStarRequest>("ltstar_request", 10);
    state_manager_node::frontier_request_pub = nh.advertise<frontiers_msgs::FrontierRequest>("frontiers_request", 10);
    state_manager_node::target_position_pub = nh.advertise<architecture_msgs::PositionRequest>("target_position", 10);
    

    
    init_state_variables(state_manager_node::state_data);
    // TODO Lazy theta star topics
    octomath::Vector3 geofence_min (-30, -30, 1);
    octomath::Vector3 geofence_max (30, 30, 10);
    ros::Rate rate(0.5);
    while(ros::ok() && state_manager_node::state_data.exploration_state != state_manager_node::finished_exploring) 
    {
        state_manager_node::update_state(geofence_min, geofence_max);
    
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
