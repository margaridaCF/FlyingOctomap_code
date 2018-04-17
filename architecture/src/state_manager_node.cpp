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
#include <visualization_msgs/Marker.h>

#include <marker_publishing_utils.h>

#include <architecture_msgs/PositionRequest.h>
#include <architecture_msgs/PositionMiddleMan.h>
#include <architecture_msgs/YawSpin.h>

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

    ros::Publisher frontier_request_pub;
    ros::Publisher ltstar_request_pub;
    ros::Publisher marker_pub;
    ros::ServiceClient target_position_client;
    ros::ServiceClient is_frontier_client;
    ros::ServiceClient frontier_status_client;
    ros::ServiceClient ltstar_status_cliente;
    ros::ServiceClient current_position_client;
    ros::ServiceClient yaw_spin_client;


    // TODO - transform this into parameters at some point
    double px4_loiter_radius;
    double odometry_error;
    double safety_margin = 3;
    double error_margin;
    ros::Duration exploration_maneuver_duration_secs;
    int max_search_iterations = 500;
    int max_cycles_waited_for_path = 3;
    octomath::Vector3 geofence_min (-5, -5, 1);
    octomath::Vector3 geofence_max (5, 5, 10);
    enum follow_path_state_t{init, on_route, arrived_at_waypoint, finished_sequence};
    enum exploration_state_t {clear_from_ground, exploration_start, choosing_goal, generating_path, waiting_path_response, visit_waypoints, finished_exploring, gather_data_maneuver};
    struct StateData { 
        int frontier_request_id;       // id for the request in use
        int frontier_request_count;      // generate id for new frontier requests
        int waypoint_index;  // id of the waypoint that is currently the waypoint
        int ltstar_request_id;
        int frontier_index;        // id of the frontier in use
        int cycles_waited_for_path;
        bool handbrake_enabled;
        bool exploration_maneuver_started;
        ros::Time request_exploration_maneuver;
        exploration_state_t exploration_state;
        follow_path_state_t follow_path_state;
        frontiers_msgs::FrontierReply frontiers_msg;
        path_planning_msgs::LTStarReply ltstar_msg;
        std::unordered_set<octomath::Vector3, Vector3Hash> unobservable_set; 
    };  
    state_manager_node::StateData state_data;

    // TODO when thre is generation of path these two will be different
    geometry_msgs::Point& get_current_waypoint()
    {
        return state_data.ltstar_msg.waypoints[state_data.waypoint_index];
    }

    geometry_msgs::Point get_current_frontier()
    {
        return state_data.frontiers_msg.frontiers[state_data.frontier_index].xyz_m;
    }

    bool getUavPositionServiceCall(geometry_msgs::Point& current_position)
    {
        architecture_msgs::PositionMiddleMan srv;
        if(current_position_client.call(srv))
        {
            current_position = srv.response.current_position;

            // ROS_INFO_STREAM("[State manager] 1 Current (" << current_position.x << ", " << current_position.y << ", " << current_position.z << ");");
            return true;
        }
        else
        {
            ROS_WARN("[State manager] Current position middle man node not accepting requests.");
            return false;
        }
    }

    bool askYawSpinServiceCall()
    {
        architecture_msgs::YawSpin yaw_spin_srv;
        yaw_spin_srv.request.position = get_current_waypoint();
        if(yaw_spin_client.call(yaw_spin_srv))
        {
            state_data.request_exploration_maneuver = ros::Time::now();
            return true;
        }
        else
        {
            ROS_WARN("[State manager] YawSpin node not accepting requests.");
            return false;
        }
    }

    bool askPositionServiceCall(geometry_msgs::Point& position)
    {
        architecture_msgs::PositionRequest position_request_srv;
        position_request_srv.request.waypoint_sequence_id = state_data.ltstar_request_id;
        position_request_srv.request.position = position;
        if(target_position_client.call(position_request_srv))
        {
            ROS_INFO_STREAM("[State manager] Requesting position " << state_data.waypoint_index << " = " << get_current_waypoint());
            return position_request_srv.response.is_going_to_position;
        }
        else
        {
            ROS_WARN("[State manager] In YawSpin, node not accepting position requests.");
            return false;
        }
    }

    void askForObstacleAvoidingPath(octomath::Vector3 const& start, octomath::Vector3 const& goal, ros::Publisher const& ltstar_request_pub)
    {
        path_planning_msgs::LTStarRequest request;
        state_data.ltstar_request_id++;
        request.request_id = state_data.ltstar_request_id;
        request.header.frame_id = "world";
        request.start.x = start.x();
        request.start.y = start.y();
        request.start.z = start.z();
        request.goal.x = goal.x();
        request.goal.y = goal.y();
        request.goal.z = goal.z();
        request.max_search_iterations = max_search_iterations;
        ltstar_request_pub.publish(request);
        rviz_interface::publish_start(request.start, marker_pub);
        rviz_interface::publish_goal(request.goal, marker_pub);
        // ROS_WARN_STREAM("[State manager] Start position is " << request.start);
    }

    void askForFrontiers(int request_count, octomath::Vector3 const& geofence_min, octomath::Vector3 const& geofence_max, ros::Publisher const& frontier_request_pub)
    {
        frontiers_msgs::FrontierRequest request;
        request.header.seq = state_data.frontier_request_count;
        request.header.frame_id = "world";
        request.min.x = geofence_min.x();
        request.min.y = geofence_min.y();
        request.min.z = geofence_min.z();
        request.max.x = geofence_max.x();
        request.max.y = geofence_max.y();
        request.max.z = geofence_max.z();
        request.safety_margin = safety_margin;
        request.frontier_amount = state_data.unobservable_set.size()+1;
        request.min_distance = px4_loiter_radius;
        while(!getUavPositionServiceCall(request.current_position));
        frontier_request_pub.publish(request);
        state_data.frontier_request_count++;
    }

    void ltstar_cb(const path_planning_msgs::LTStarReply::ConstPtr& msg)
    {
        if(state_data.exploration_state != waiting_path_response)
        {
            ROS_INFO_STREAM("[State manager] Received path from Lazy Theta Star but the state is not waiting_path_response. Discarding.");
        }
        else if(msg->request_id != state_data.ltstar_request_id)
        {
            ROS_INFO_STREAM("[State manager] Received path from Lazy Theta Star with request id " << msg->request_id << " but the current request_id is " << state_data.ltstar_request_id << ". Discarding.");
        }
        else
        {
            if(msg->success)
            {
                // ROS_INFO_STREAM("[State manager] Received path from Lazy Theta Star " << *msg);
                // Update state variables
                // state_data.sequence_waypoint_count = msg.waypoint_amount;
                state_data.ltstar_msg = *msg;
                state_data.follow_path_state = init;
                state_data.exploration_state = visit_waypoints;
                state_data.waypoint_index = 1;
                ROS_INFO_STREAM("[State manager][Exploration] visit_waypoints");
                ROS_INFO_STREAM("[State manager]            [Follow path] init");
            }
            else
            {
                octomath::Vector3 unreachable (get_current_frontier().x, get_current_frontier().y, get_current_frontier().z);
                state_data.unobservable_set.insert(unreachable);
                state_data.exploration_state = choosing_goal;
                state_data.waypoint_index = -1;
                ROS_INFO_STREAM("[State manager][Exploration] choosing_goal (Lazy Theta Star exhausted iterations)");
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
            state_data.frontier_request_id = msg->request_id;
            state_data.exploration_state = choosing_goal;
            state_data.frontiers_msg = *msg;
            ROS_INFO_STREAM("[State manager][Exploration] choosing_goal");

            if(get_current_frontier().x < geofence_min.x() 
                || get_current_frontier().y < geofence_min.y() 
                || get_current_frontier().x < geofence_min.y() 
                || get_current_frontier().x > geofence_max.x() 
                || get_current_frontier().y > geofence_max.y()  
                || get_current_frontier().z > geofence_max.z())
            {
                ROS_ERROR_STREAM("[State manager] Breaching geofence " <<  *msg);
            }
        }
    }

    bool is_in_target_position(geometry_msgs::Point const& target_waypoint, 
        geometry_msgs::Point & current_position, double error_margin )
    {
        // ROS_INFO_STREAM("[State manager] Target position " << target_waypoint );
        // ROS_INFO_STREAM("[State manager] Current position " << current_position );
        // ROS_INFO_STREAM("[State manager] 3 Position offset");
        // ROS_INFO_STREAM("[State manager] Current (" << current_position.x << ", " << current_position.y << ", " << current_position.z << ");");
        // ROS_INFO_STREAM("[State manager]  Target (" << target_waypoint.x << ", " << target_waypoint.y << ", " << target_waypoint.z << ");");
        ROS_INFO_STREAM("[State manager] Position offset (" << std::abs(target_waypoint.x - current_position.x) << ", "
            << std::abs(target_waypoint.y - current_position.y) << ", "
            << std::abs(target_waypoint.z - current_position.z) << ") ");

        return std::abs(target_waypoint.x - current_position.x) <= error_margin
            && std::abs(target_waypoint.y - current_position.y) <= error_margin
            && std::abs(target_waypoint.z - current_position.z) <= error_margin;
    }

    bool updateWaypointSequenceStateMachine()
    {
        switch(state_data.follow_path_state)
        {
            case init:
            {
                if(askPositionServiceCall(get_current_waypoint()))
                {
                    state_data.follow_path_state = on_route;
                    ROS_INFO_STREAM("[State manager]            [Path follow] on_route to " << get_current_waypoint());
                }
                else
                {
                    ROS_WARN_STREAM("[State manager] Failed to set next position. Going to keep trying.");
                }
                break;
            }
            case on_route:
            {
                geometry_msgs::Point current_position;
                if(getUavPositionServiceCall(current_position))
                {
                    // compare target with postition allowing for error margin 
                    // ROS_INFO_STREAM("[State manager] 2 Current (" << current_position.x << ", " << current_position.y << ", " << current_position.z << ");");
                    geometry_msgs::Point target_waypoint;
                    target_waypoint = get_current_waypoint();
                    if( is_in_target_position(target_waypoint, current_position, error_margin) )
                    {
                        state_data.follow_path_state = arrived_at_waypoint;
                    }
                }
                break;
            }
            case arrived_at_waypoint:
            {
                if(state_data.ltstar_msg.waypoint_amount == state_data.waypoint_index+1)
                {
                    // Reached Frontier
                    ROS_INFO_STREAM("[State manager] Reached final waypoint (" << state_data.waypoint_index << ") of sequence " << state_data.frontier_request_id << ": " << get_current_waypoint());
                    state_data.follow_path_state = finished_sequence;
                    ROS_INFO_STREAM("[State manager]            [Path follow]  finished_sequence");
                }
                else {
                    state_data.waypoint_index++;
                    state_data.follow_path_state = init;
                }
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
                state_data.frontier_request_id = state_data.frontiers_msg.request_id;
                state_data.waypoint_index = -1;
                state_data.frontier_index = i;
                // publish_marker_safety_margin(get_current_frontier(), safety_margin);
                ROS_INFO_STREAM("[State manager] New frontier ("
                    <<get_current_frontier().x << ", "
                    <<get_current_frontier().y << ", "
                    <<get_current_frontier().z << ") ");
                rviz_interface::publish_frontier_marker(get_current_frontier(), true, marker_pub);
                return true;
            }
        }
        ROS_INFO_STREAM("[State manager] Could not find an observable frontier.");
        return false;
    }

    void init_state_variables(state_manager_node::StateData& state_data)
    {
        state_data.exploration_maneuver_started = false;
        state_data.handbrake_enabled = false;
        state_data.ltstar_request_id = 0;
        state_data.frontier_request_count = 0;
        state_data.exploration_state = clear_from_ground;
        ROS_INFO_STREAM("[State manager][Exploration] clear_from_ground");
    }

    void init_param_variables(ros::NodeHandle& nh)
    {
        double temp;
        nh.getParam("exploration_maneuver_duration_secs", temp);
        exploration_maneuver_duration_secs = ros::Duration(temp);
        nh.getParam("px4_loiter_radius", px4_loiter_radius);
        nh.getParam("odometry_error", odometry_error);
        nh.getParam("safety_margin", safety_margin);
        error_margin = std::max(px4_loiter_radius, odometry_error);
        nh.getParam("path/max_search_iterations", max_search_iterations);
        nh.getParam("path/max_cycles_waited_for_path", max_cycles_waited_for_path);

        float x, y, z;
        nh.getParam("geofence_min/x", x);
        nh.getParam("geofence_min/y", y);
        nh.getParam("geofence_min/z", z);
        geofence_min = octomath::Vector3  (x, y, z);
        
        nh.getParam("geofence_max/x", x);
        nh.getParam("geofence_max/y", y);
        nh.getParam("geofence_max/z", z);
        geofence_max = octomath::Vector3  (x, y, z);
    }

    void update_state(octomath::Vector3 const& geofence_min, octomath::Vector3 const& geofence_max)
    {
        switch(state_data.exploration_state)
        {
            case clear_from_ground:
            {
                // Find current position
                // architecture_msgs::PositionMiddleMan srv;
                // if(current_position_client.call(srv))
                geometry_msgs::Point current_position;
                if(getUavPositionServiceCall(current_position))
                {
                    // geometry_msgs::Point current_position = srv.response.current_position;
                    state_data.frontier_request_id = 0;
                    state_data.waypoint_index = 0;
                    state_data.exploration_state = visit_waypoints;
                    state_data.follow_path_state = init;

                    geometry_msgs::Point waypoint;
                    waypoint.x = current_position.x;
                    waypoint.y = current_position.y;
                    waypoint.z = current_position.z + 5;
                    state_data.ltstar_msg.waypoints.push_back(waypoint);
                    waypoint.x = current_position.x;
                    waypoint.y = current_position.y;
                    waypoint.z = 2;
                    state_data.ltstar_msg.waypoints.push_back(waypoint);
                    state_data.frontiers_msg.frontiers_found = 1;
                    
                    state_data.ltstar_msg.waypoint_amount = 2;
                    ROS_INFO_STREAM("[State manager][Exploration] visit_waypoints");
                    ROS_INFO_STREAM("[State manager]            [Follow path] init");
                }

                break;
            }
            case exploration_start:
            {
                state_data.exploration_maneuver_started = false;
                state_data.waypoint_index = -1;
                frontiers_msgs::FrontierNodeStatus srv;
                if (frontier_status_client.call(srv))
                {
                    if((bool)srv.response.is_accepting_requests)
                    {
                        // ROS_INFO_STREAM("[State manager] Asking for frontiers.");
                        askForFrontiers(state_data.frontier_request_count, geofence_min, geofence_max, frontier_request_pub);
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
                        if(getUavPositionServiceCall(current_position))
                        {
                            octomath::Vector3 current_position_v (current_position.x, current_position.y, current_position.z);
                            rviz_interface::publish_current_position(current_position_v, marker_pub);
                            octomath::Vector3 start(current_position.x, current_position.y, current_position.z);
                            octomath::Vector3 goal (get_current_frontier().x, get_current_frontier().y, get_current_frontier().z);
                            askForObstacleAvoidingPath(start, goal, ltstar_request_pub);
                            state_data.exploration_state = waiting_path_response;
                            state_data.cycles_waited_for_path = 0;
                        }
                    }
                }
                else
                {
                    ROS_WARN("[State manager] Lazy Theta Star node not accepting requests.");
                }
                break;
            }
            case waiting_path_response:
            {
                ROS_INFO_STREAM("[State manager] Waited " << state_data.cycles_waited_for_path << " cycles of " << max_cycles_waited_for_path);
                if(state_data.cycles_waited_for_path < max_cycles_waited_for_path)
                {
                    state_data.cycles_waited_for_path++;
                }
                else
                {
                    octomath::Vector3 unreachable (get_current_frontier().x, get_current_frontier().y, get_current_frontier().z);
                    state_data.unobservable_set.insert(unreachable);
                    state_data.exploration_state = choosing_goal;
                    state_data.waypoint_index = -1;
                    ROS_INFO_STREAM("[State manager][Exploration] choosing_goal (No response from Lazy Theta Star)");
                }
                break;
            }
            case visit_waypoints:
            {
                updateWaypointSequenceStateMachine();
                if (state_data.follow_path_state == finished_sequence)
                {
                    state_data.exploration_state = gather_data_maneuver;
                    ROS_INFO_STREAM("[State manager][Exploration] gather_data_maneuver");
                }
                break;
            }
            case gather_data_maneuver:
            {
                if (!state_data.exploration_maneuver_started)
                {
                    state_data.exploration_maneuver_started = askYawSpinServiceCall();
                }
                else
                {
                    ros::Duration time_lapse = ros::Time::now() - state_data.request_exploration_maneuver;
                    if(time_lapse > exploration_maneuver_duration_secs)
                    {
                        state_data.exploration_state = exploration_start;
                    }
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
    state_manager_node::init_param_variables(nh);
    // Service client
    state_manager_node::ltstar_status_cliente = nh.serviceClient<path_planning_msgs::LTStarNodeStatus>("ltstar_status");
    state_manager_node::frontier_status_client = nh.serviceClient<frontiers_msgs::FrontierNodeStatus>("frontier_status");
    state_manager_node::is_frontier_client = nh.serviceClient<frontiers_msgs::CheckIsFrontier>("is_frontier");
    state_manager_node::current_position_client = nh.serviceClient<architecture_msgs::PositionMiddleMan>("get_current_position");
    state_manager_node::yaw_spin_client = nh.serviceClient<architecture_msgs::YawSpin>("yaw_spin");
    state_manager_node::target_position_client = nh.serviceClient<architecture_msgs::PositionRequest>("target_position");
    // Topic subscribers 
    ros::Subscriber frontiers_reply_sub = nh.subscribe<frontiers_msgs::FrontierReply>("frontiers_reply", 5, state_manager_node::frontier_cb);
    ros::Subscriber ltstar_reply_sub = nh.subscribe<path_planning_msgs::LTStarReply>("ltstar_reply", 5, state_manager_node::ltstar_cb);
    // Topic publishers
    state_manager_node::ltstar_request_pub = nh.advertise<path_planning_msgs::LTStarRequest>("ltstar_request", 10);
    state_manager_node::frontier_request_pub = nh.advertise<frontiers_msgs::FrontierRequest>("frontiers_request", 10);
    state_manager_node::marker_pub = nh.advertise<visualization_msgs::Marker>("geofence", 1);
    

    
    state_manager_node::init_state_variables(state_manager_node::state_data);
    ros::Rate rate(2);
    while(ros::ok() && state_manager_node::state_data.exploration_state != state_manager_node::finished_exploring) 
    {
        rviz_interface::publish_geofence(state_manager_node::geofence_min, state_manager_node::geofence_max, state_manager_node::marker_pub);
        state_manager_node::update_state(state_manager_node::geofence_min, state_manager_node::geofence_max);
    
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
