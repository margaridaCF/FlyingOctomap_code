
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <octomap/math/Vector3.h>
#include <unordered_set>
#include <visualization_msgs/Marker.h>

#include <sstream>
#include <fstream>
#include <string>
#include <chrono>
#include <boost/filesystem.hpp>

#include <architecture_math.h>
#include <goal_state_machine.h>
#include <observation_maneuver.h>
#include <marker_publishing_utils.h>

#include <architecture_msgs/PositionRequest.h>
#include <architecture_msgs/PositionMiddleMan.h>
#include <architecture_msgs/YawSpin.h>

#include <frontiers_msgs/CheckIsFrontier.h>
#include <frontiers_msgs/FrontierReply.h>
#include <frontiers_msgs/FrontierRequest.h>
#include <frontiers_msgs/FrontierNodeStatus.h>
#include <frontiers_msgs/VoxelMsg.h>

#include <lazy_theta_star_msgs/LTStarReply.h>
#include <lazy_theta_star_msgs/LTStarRequest.h>
#include <lazy_theta_star_msgs/LTStarNodeStatus.h>
#include <lazy_theta_star_msgs/CheckFlightCorridor.h>


#define SAVE_CSV 1
#define SAVE_LOG 1


namespace state_manager_node
{
    // std::string folder_name = "/ros_ws/src/data";
    std::stringstream aux_envvar_home (std::getenv("HOME"));
    std::string folder_name = aux_envvar_home.str() + "/Flying_Octomap_code/src/data";


    ros::Publisher frontier_request_pub;
    ros::Publisher ltstar_request_pub;
    ros::Publisher marker_pub;
    ros::ServiceClient target_position_client;
    ros::ServiceClient is_frontier_client;
    ros::ServiceClient frontier_status_client;
    ros::ServiceClient ltstar_status_cliente;
    ros::ServiceClient current_position_client;
    ros::ServiceClient yaw_spin_client;

    ros::Timer timer;
    std::chrono::high_resolution_clock::time_point start;
    bool is_successfull_exploration = false;
    std::ofstream log_file;

    // TODO - transform this into parameters at some point
    double px4_loiter_radius;
    double laser_range_xy;
    double odometry_error;
    double safety_margin = 3;
    double error_margin;
    bool do_initial_maneuver;
    double distance_inFront, distance_behind;
    int circle_divisions = 12;
    ros::Duration exploration_maneuver_duration_secs;
    int max_time_secs =5000;
    double ltstar_safety_margin;
    octomath::Vector3 geofence_min (-5, -5, 1);
    octomath::Vector3 geofence_max (5, 5, 10);
    enum follow_path_state_t{init, on_route, arrived_at_waypoint, finished_sequence};
    enum exploration_state_t {clear_from_ground = 0, exploration_start= 1, generating_path = 2, waiting_path_response = 3, visit_waypoints = 4, finished_exploring = 5, gather_data_maneuver = 6};
    struct StateData { 
        int frontier_request_id;    // id for the request in use
        int frontier_request_count; // generate id for new frontier requests
        int waypoint_index;         // id of the waypoint that is currently the waypoint
        int ltstar_request_id;
        int frontier_index;         // id of the frontier in use
        bool exploration_maneuver_started, initial_maneuver;
        exploration_state_t exploration_state;
        follow_path_state_t follow_path_state;
        frontiers_msgs::FrontierRequest frontiers_request;
        frontiers_msgs::FrontierReply frontiers_msg;
        lazy_theta_star_msgs::LTStarRequest ltstar_request;
        lazy_theta_star_msgs::LTStarReply ltstar_reply;
        std::shared_ptr<goal_state_machine::GoalStateMachine> goal_state_machine;
    };  
    state_manager_node::StateData state_data;

    // TODO when thre is generation of path these two will be different
    geometry_msgs::Pose& get_current_waypoint()
    {
        return state_data.ltstar_reply.waypoints[state_data.waypoint_index];
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

    bool askPositionServiceCall(geometry_msgs::Pose& pose)
    {
        architecture_msgs::PositionRequest position_request_srv;
        position_request_srv.request.waypoint_sequence_id = state_data.ltstar_request_id;
        position_request_srv.request.pose = pose;
        if(target_position_client.call(position_request_srv))
        {
            #ifdef SAVE_LOG
            log_file << "[State manager] Requesting position " << state_data.waypoint_index << " = " << position_request_srv.request.pose << std::endl;
            #endif
            return position_request_srv.response.is_going_to_position;
        }
        else
        {
            // ROS_WARN("[State manager] In YawSpin, node not accepting position requests.");
            return false;
        }
    }

    void askForObstacleAvoidingPath(octomath::Vector3 const& start, octomath::Vector3 const& goal, ros::Publisher const& ltstar_request_pub)
    {
        lazy_theta_star_msgs::LTStarRequest request;
        state_data.ltstar_request_id++;
        request.request_id = state_data.ltstar_request_id;
        request.header.frame_id = "world";
        request.start.x = start.x();
        request.start.y = start.y();
        request.start.z = start.z();
        request.goal.x = goal.x();
        request.goal.y = goal.y();
        request.goal.z = goal.z();
        request.max_time_secs = max_time_secs;
        request.safety_margin = ltstar_safety_margin;
        #ifdef SAVE_LOG
        log_file << "[State manager] Requesting path " << request << std::endl;
        #endif
        ROS_INFO_STREAM ("[State manager] Requesting path from " << request.start << " to " << request.goal);
        ltstar_request_pub.publish(request);
        state_data.ltstar_request = request;
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
        request.frontier_amount = state_data.goal_state_machine->getUnobservableSetSize()+10;
        request.min_distance = px4_loiter_radius;
        request.sensing_distance = laser_range_xy;
        while(!getUavPositionServiceCall(request.current_position));
        #ifdef SAVE_LOG
        log_file << "[State manager] Requesting frontier " << request << std::endl;
        #endif
        ROS_WARN_STREAM (     "[State manager] Requesting frontier " << request.frontier_amount  << " frontiers." );
        state_data.frontiers_request = request;
        frontier_request_pub.publish(request);
        state_data.frontier_request_count++;
    }

    bool askIsFrontierServiceCall(geometry_msgs::Point candidate) 
    { 
        frontiers_msgs::CheckIsFrontier is_frontier_msg; 
        is_frontier_msg.request.candidate = candidate; 
        if(is_frontier_client.call(is_frontier_msg)) 
        { 
            #ifdef SAVE_LOG
            log_file << "[State manager] Is frontier " << candidate << std::endl;
            #endif
            return is_frontier_msg.response.is_frontier; 
        } 
        else 
        { 
            ROS_WARN("[State manager] Frontier node not accepting is frontier requests."); 
            return false; 
        } 
    } 

    void ltstar_cb(const lazy_theta_star_msgs::LTStarReply::ConstPtr& msg)
    {
        if(state_data.exploration_state != waiting_path_response)
        {
            #ifdef SAVE_LOG
            log_file << "[State manager] Received path from Lazy Theta Star but the state is not waiting_path_response. Discarding." << std::endl;
            #endif
        }
        else if(msg->request_id != state_data.ltstar_request_id)
        {
            #ifdef SAVE_LOG
            log_file << "[State manager] Received path from Lazy Theta Star with request id " << msg->request_id << " but the current request_id is " << state_data.ltstar_request_id << ". Discarding." << std::endl;
            #endif
        }
        else
        {
            if(msg->success)
            {
                state_data.ltstar_reply = *msg;
                state_data.follow_path_state = init;
                state_data.exploration_state = visit_waypoints;
                state_data.waypoint_index = 0;
                #ifdef SAVE_LOG
                log_file << "[State manager] Path reply " << *msg << std::endl;
                log_file << "[State manager]            [Follow path] init" << std::endl;
                #endif
            }
            else
            {
                ROS_WARN_STREAM (     "[State manager] Path reply failed!");
                state_data.exploration_state = generating_path;
            }
            
        }
    }

    void frontier_cb(const frontiers_msgs::FrontierReply::ConstPtr& msg)
    {
        if(msg->frontiers_found == 0)
        {
            // TODO - go back to base and land
            #ifdef SAVE_LOG
            log_file << "[State manager][Exploration] finished_exploring - no frontiers reported." << std::endl;
            #endif
            ROS_INFO_STREAM("[State manager][Exploration] finished_exploring - no frontiers reported.");
            is_successfull_exploration = true;
            state_data.exploration_state = finished_exploring;
        }
        else if(msg->frontiers_found > 0 && state_data.exploration_state == exploration_start)
        {
            log_file << "[State Manager] [oppairs] 1. New frontier arrived from node. Reset all." << std::endl;
            #ifdef SAVE_LOG
            log_file << "[State manager]Frontier reply " << *msg << std::endl;
            #endif
            state_data.frontier_request_id = msg->request_id;
            state_data.exploration_state = generating_path;
            state_data.frontiers_msg = *msg;
            state_data.goal_state_machine->NewFrontiers(state_data.frontiers_msg);
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
        // ROS_INFO_STREAM("[State manager] Position offset (" << std::abs(target_waypoint.x - current_position.x) << ", "
        //     << std::abs(target_waypoint.y - current_position.y) << ", "
        //     << std::abs(target_waypoint.z - current_position.z) << ") ");

        return std::abs(target_waypoint.x - current_position.x) <= error_margin
            && std::abs(target_waypoint.y - current_position.y) <= error_margin
            && std::abs(target_waypoint.z - current_position.z) <= error_margin;
    }

    bool hasArrived(geometry_msgs::Point target)
    {
        geometry_msgs::Point current_position;
        if(getUavPositionServiceCall(current_position))
        {
            // compare target with postition allowing for error margin 
            // ROS_INFO_STREAM("[State manager] 2 Current (" << current_position.x << ", " << current_position.y << ", " << current_position.z << ");");
            geometry_msgs::Point target_waypoint;
            if( is_in_target_position(target, current_position, error_margin) )
            {
                return true;
            }
        }
        return false;
    }

    void buildTargetPose(geometry_msgs::Pose & target)
    {
        geometry_msgs::Point current_position;
        getUavPositionServiceCall(current_position);
        target = get_current_waypoint();
        Eigen::Vector3d current_e (current_position.x, current_position.y, current_position.z);
        Eigen::Vector3d next_e (target.position.x, target.position.y, target.position.z);
        // log_file << "[State manager] buildTargetPose current_e to next_e" << std::endl;
        double yaw = architecture_math::calculateOrientation(Eigen::Vector2d(current_e.x(), current_e.y()), Eigen::Vector2d(next_e.x(), next_e.y()));
        // log_file << "[State manager] buildTargetPose yaw = " << yaw << std::endl;

        target.orientation = tf::createQuaternionMsgFromYaw(yaw + M_PI);
    }

    bool updateWaypointSequenceStateMachine()
    {
        switch(state_data.follow_path_state)
        {
            case init:
            {
                geometry_msgs::Pose next_waypoint;
                buildTargetPose(next_waypoint);
                // ROS_WARN_STREAM("[State manager]            [Path follow] updateWaypointSequenceStateMachine at init");
                if(askPositionServiceCall(next_waypoint))
                {
                    state_data.follow_path_state = on_route;
                    // #ifdef SAVE_LOG
                    // log_file << "[State manager]            [Path follow] on_route to " << get_current_waypoint() << std::endl;
                    // #endif
                }
                else
                {
                    #ifdef SAVE_LOG
                    log_file << "[State manager] Failed to set next position. Going to keep trying." << std::endl;
                    #endif
                }
                break;
            }
            case on_route:
            {
                // ROS_WARN_STREAM("[State manager]            [Path follow] updateWaypointSequenceStateMachine at on_route");
                geometry_msgs::Point target_waypoint = get_current_waypoint().position;
                if(hasArrived(target_waypoint))
                {
                    state_data.follow_path_state = arrived_at_waypoint;
                }
                break;
            }
            case arrived_at_waypoint:
            {
                ROS_WARN_STREAM("[State manager]            [Path follow] updateWaypointSequenceStateMachine at arrived_at_waypoint");
                if(state_data.ltstar_reply.waypoint_amount == state_data.waypoint_index+1)
                {
                    // Reached Frontier
                    #ifdef SAVE_LOG
                    log_file << "[State manager] Reached final waypoint (" << state_data.waypoint_index << ") of sequence " << state_data.frontier_request_id << ": " << get_current_waypoint() << std::endl;
                    log_file << "[State manager]            [Path follow]  finished_sequence" << std::endl;
                    #endif
                    state_data.follow_path_state = finished_sequence;
                }
                else {
                    state_data.waypoint_index++;
                    state_data.follow_path_state = init;
                }
                break;
            }
        }
    }

    void init_state_variables(state_manager_node::StateData& state_data, ros::NodeHandle& nh)
    {
        state_data.exploration_maneuver_started = false;
        state_data.initial_maneuver = true;
        state_data.ltstar_request_id = 0;
        state_data.frontier_request_count = 0;
        if(do_initial_maneuver)
        {
            state_data.exploration_state = clear_from_ground;
        }
        else
        {
            state_data.exploration_state = waiting_path_response;
        }
        #ifdef SAVE_LOG
        log_file << "[State manager][Exploration] Switch to clear_from_ground" << std::endl;
        #endif
    }

    void init_param_variables(ros::NodeHandle& nh)
    {
        double temp;
        nh.getParam("exploration_maneuver_duration_secs", temp);
        exploration_maneuver_duration_secs = ros::Duration(temp);
        nh.getParam("px4_loiter_radius", px4_loiter_radius);
        double laser_range;
        nh.getParam("laser_range", laser_range);
        do_initial_maneuver = false;
        nh.getParam("do_initial_maneuver", do_initial_maneuver);
        double laser_angle;
        nh.getParam("laser_angle", laser_angle);
        laser_range_xy = std::cos(laser_angle)*laser_range;
        ROS_INFO_STREAM("[architecture] Laser xy range: " << laser_range_xy);
        nh.getParam("odometry_error", odometry_error);
        nh.getParam("frontier/safety_margin", safety_margin);
        error_margin = std::max(px4_loiter_radius, odometry_error);
        nh.getParam("path/max_time_secs", max_time_secs);
        nh.getParam("path/safety_margin", ltstar_safety_margin);

        float x, y, z;
        nh.getParam("geofence_min/x", x);
        nh.getParam("geofence_min/y", y);
        nh.getParam("geofence_min/z", z);
        geofence_min = octomath::Vector3  (x, y, z);
        
        nh.getParam("geofence_max/x", x);
        nh.getParam("geofence_max/y", y);
        nh.getParam("geofence_max/z", z);
        geofence_max = octomath::Vector3  (x, y, z);

        // Goal state machine
        nh.getParam("oppairs/distance_inFront", distance_inFront);
        nh.getParam("oppairs/distance_behind",  distance_behind);
        nh.getParam("oppairs/circle_divisions",  circle_divisions);
    }

    void publishGoalToRviz(geometry_msgs::Point current_position)
    {
        geometry_msgs::Point frontier_geom = get_current_frontier();
        geometry_msgs::Point start_geom;
        start_geom.x = current_position.x;
        start_geom.y = current_position.y;
        start_geom.z = current_position.z;
        geometry_msgs::Point oppair_start_geom;
        state_data.goal_state_machine->getFlybyStart(oppair_start_geom);
        geometry_msgs::Point oppair_end_geom;
        state_data.goal_state_machine->getFlybyEnd(oppair_end_geom);
        visualization_msgs::MarkerArray marker_array;
        rviz_interface::build_stateManager(frontier_geom, oppair_start_geom, oppair_end_geom, start_geom, state_data.frontiers_request.safety_margin, marker_array);
        marker_pub.publish(marker_array);
    }

    void update_state(octomath::Vector3 const& geofence_min, octomath::Vector3 const& geofence_max)
    {
        log_file << "[State Manager] [update_state] " << state_data.exploration_state << std::endl;
        switch(state_data.exploration_state)
        {
            case clear_from_ground:
            {
                ROS_INFO("[architecture] Clear from ground");
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

                    geometry_msgs::Pose waypoint;
                    waypoint.position.x = 0;
                    waypoint.position.y = 0;
                    waypoint.position.z = 2;
                    Eigen::Vector3d fake_uav_position (waypoint.position.x, waypoint.position.y, waypoint.position.z);
                    state_data.ltstar_reply.waypoints.push_back(waypoint);
                    waypoint.position.x = 5;
                    waypoint.position.y = -5;
                    waypoint.position.z = 2;
                    state_data.ltstar_reply.waypoints.push_back(waypoint);
                    waypoint.position.x = 0;
                    waypoint.position.y = 0;
                    waypoint.position.z = 2;
                    state_data.ltstar_reply.waypoints.push_back(waypoint);

                    waypoint.position.x = 5;
                    waypoint.position.y = 5;
                    waypoint.position.z = 2;
                    state_data.ltstar_reply.waypoints.push_back(waypoint);
                    waypoint.position.x = 0;
                    waypoint.position.y = 0;
                    waypoint.position.z = 2;
                    state_data.ltstar_reply.waypoints.push_back(waypoint);

                    waypoint.position.x = -5;
                    waypoint.position.y = 5;
                    waypoint.position.z = 2;
                    state_data.ltstar_reply.waypoints.push_back(waypoint);
                    waypoint.position.x = 0;
                    waypoint.position.y = 0;
                    waypoint.position.z = 2;
                    state_data.ltstar_reply.waypoints.push_back(waypoint);

                    waypoint.position.x = -5;
                    waypoint.position.y = -5;
                    waypoint.position.z = 2;
                    state_data.ltstar_reply.waypoints.push_back(waypoint);
                    waypoint.position.x = 0;
                    waypoint.position.y = 0;
                    waypoint.position.z = 2;
                    state_data.ltstar_reply.waypoints.push_back(waypoint);

                    waypoint.position.x = 5;
                    waypoint.position.y = 0;
                    waypoint.position.z = 2;
                    state_data.ltstar_reply.waypoints.push_back(waypoint);
                    waypoint.position.x = -5;
                    waypoint.position.y = 1;
                    waypoint.position.z = 2;
                    state_data.ltstar_reply.waypoints.push_back(waypoint);
                    waypoint.position.x = 0;
                    waypoint.position.y = 0;
                    waypoint.position.z = 2;
                    state_data.ltstar_reply.waypoints.push_back(waypoint);
                    waypoint.position.x = 0;
                    waypoint.position.y = 5;
                    waypoint.position.z = 2;
                    state_data.ltstar_reply.waypoints.push_back(waypoint);
                    waypoint.position.x = 0;
                    waypoint.position.y = 0;
                    waypoint.position.z = 2;
                    state_data.ltstar_reply.waypoints.push_back(waypoint);
                    Eigen::Vector3d fake_frontier_e (waypoint.position.x, waypoint.position.y, waypoint.position.z);
                    state_data.frontiers_msg.frontiers_found = 1;
                    state_data.ltstar_reply.waypoint_amount = 14;
                    frontiers_msgs::VoxelMsg fake_frontier;
                    fake_frontier.xyz_m = current_position;
                    state_data.frontiers_msg.frontiers.push_back(fake_frontier);
                    state_data.frontier_index = 0;
                    state_data.goal_state_machine->NewFrontiers(state_data.frontiers_msg);

                    #ifdef SAVE_LOG
                    log_file << "[State manager][Exploration] visit_waypoints 2" << std::endl;
                    log_file << "[State manager]            [Follow path] init" << std::endl;
                    #endif
                }

                break;
            }
            case exploration_start:
            {
                log_file << "[State Manager] [exploration_start] " << std::endl;
                state_data.exploration_maneuver_started = false;
                state_data.waypoint_index = -1;
                frontiers_msgs::FrontierNodeStatus srv;
                if (frontier_status_client.call(srv))
                {
                    if((bool)srv.response.is_accepting_requests)
                    {
                        ROS_INFO_STREAM("[State manager] Asking for frontiers.");
                        askForFrontiers(state_data.frontier_request_count, geofence_min, geofence_max, frontier_request_pub);
                    }
                }
                else
                {
                    ROS_WARN("[State manager] Frontier node not accepting requests.");
                }
                break;
            }
            case generating_path:
            {
                log_file << "[State Manager] [generating_path] " << std::endl;
                lazy_theta_star_msgs::LTStarNodeStatus srv;
                if(ltstar_status_cliente.call(srv))
                {
                    if((bool)srv.response.is_accepting_requests)
                    {
                        geometry_msgs::Point current_position;
                        if(getUavPositionServiceCall(current_position))
                        {
                            Eigen::Vector3d current_position_e (current_position.x, current_position.y, current_position.z);
                            octomath::Vector3 start(current_position.x, current_position.y, current_position.z);
                            bool existsNext = state_data.goal_state_machine->NextGoal(current_position_e);
                            if(!existsNext)
                            {
                                state_data.exploration_state = exploration_start;
                            }
                            else
                            {
                                Eigen::Vector3d oppair_start;
                                state_data.goal_state_machine->getFlybyStart(oppair_start);
                                octomath::Vector3 goal (oppair_start(0), oppair_start(1), oppair_start(2));
                                askForObstacleAvoidingPath(start, goal, ltstar_request_pub);
                                state_data.exploration_state = waiting_path_response;
                                publishGoalToRviz(current_position);
                            }
                        }
                    }
                }
                else
                {
                    ROS_WARN("[State manager] Lazy Theta Star node not accepting requests.");
                }
                break;
            }
            case waiting_path_response:{break;}
            case visit_waypoints:
            {
                updateWaypointSequenceStateMachine();

                if (state_data.follow_path_state == finished_sequence)
                {
                    if(state_data.initial_maneuver)
                    {
                        state_data.exploration_state = exploration_start;
                        state_data.initial_maneuver = false;
                    }
                    else
                    {
                        state_data.exploration_state = gather_data_maneuver;
                        state_data.exploration_maneuver_started = false;
                    }
                    #ifdef SAVE_LOG
                    log_file << "[State manager][Exploration] gather_data_maneuver" << std::endl;
                    #endif
                }
                break;
            }
            case gather_data_maneuver:
            {
                geometry_msgs::Pose flyby_end;
                state_data.goal_state_machine->getFlybyEnd(flyby_end.position);
                if (!state_data.exploration_maneuver_started && !state_data.initial_maneuver)
                {
                    Eigen::Vector2d flyby_2d_start, flyby_2d_end;
                    state_data.goal_state_machine->get2DFlybyStart(flyby_2d_start);
                    state_data.goal_state_machine->get2DFlybyEnd  (flyby_2d_end);
                    flyby_end.orientation = tf::createQuaternionMsgFromYaw( architecture_math::calculateOrientation(flyby_2d_start, flyby_2d_end)+ M_PI);
                    #ifdef SAVE_LOG
                    log_file <<"[State manager] Flyby from " << flyby_2d_start << " to " << flyby_2d_end << std::endl;
                    #endif
                    state_data.exploration_maneuver_started = askPositionServiceCall(flyby_end);
                }
                else
                {
                    if(hasArrived(flyby_end.position))
                    {
                        state_data.exploration_state = exploration_start;
                        state_data.frontier_index = 0;
                        bool is_frontier = askIsFrontierServiceCall(get_current_frontier());
                        if(is_frontier)
                        {
                            ROS_ERROR_STREAM("[State manager] " << get_current_frontier() << " is still a frontier.");
                            #ifdef SAVE_LOG
                            log_file << "[State manager] " << get_current_frontier() << " is still a frontier." << std::endl;
                            #endif
                        }
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

    void main_loop(const ros::TimerEvent&)
    {
        if( state_data.exploration_state != finished_exploring) 
        {
            visualization_msgs::MarkerArray marker_array;
            rviz_interface::publish_geofence(geofence_min, geofence_max, marker_array);
            marker_pub.publish(marker_array);
            update_state(geofence_min, geofence_max);
        }
        else
        {
            timer.stop();
            #ifdef SAVE_CSV
            // TIME
            std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
            std::chrono::milliseconds millis = std::chrono::duration_cast<std::chrono::milliseconds>(time_span);
            // VOLUME
            double x = geofence_max.x() - geofence_min.x();
            double y = geofence_max.y() - geofence_min.y();
            double z = geofence_max.z() - geofence_min.z();
            double volume_meters = x * y * z;
            // WRITE
            std::ofstream csv_file;
            csv_file.open (folder_name+"/exploration_time.csv", std::ofstream::app);
            csv_file << millis.count() << "," << volume_meters << "," << is_successfull_exploration << std::endl; 
            csv_file.close();
            #endif

            #ifdef SAVE_LOG
            state_manager_node::log_file.close();
            #endif
        }
    }
}

int main(int argc, char **argv)
{
    auto timestamp_chrono = std::chrono::high_resolution_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(timestamp_chrono - std::chrono::hours(24));
    // std::string timestamp (std::put_time(std::localtime(&now_c), "%F %T") );
    std::stringstream folder_name_stream;
    folder_name_stream << state_manager_node::folder_name+"/" << (std::put_time(std::localtime(&now_c), "%F %T") );
    std::string sym_link_name = state_manager_node::folder_name+"/current";

    boost::filesystem::create_directories(folder_name_stream.str());
    boost::filesystem::create_directory_symlink(folder_name_stream.str(), sym_link_name);


    ros::init(argc, argv, "state_manager");
    ros::NodeHandle nh;
    state_manager_node::init_param_variables(nh);
    // Service client
    ros::ServiceClient check_flightCorridor_client = nh.serviceClient<lazy_theta_star_msgs::CheckFlightCorridor>("is_fligh_corridor_free");
    state_manager_node::ltstar_status_cliente = nh.serviceClient<lazy_theta_star_msgs::LTStarNodeStatus>("ltstar_status");
    state_manager_node::frontier_status_client = nh.serviceClient<frontiers_msgs::FrontierNodeStatus>("frontier_status");
    state_manager_node::is_frontier_client = nh.serviceClient<frontiers_msgs::CheckIsFrontier>("is_frontier");
    state_manager_node::current_position_client = nh.serviceClient<architecture_msgs::PositionMiddleMan>("get_current_position");
    state_manager_node::yaw_spin_client = nh.serviceClient<architecture_msgs::YawSpin>("yaw_spin");
    state_manager_node::target_position_client = nh.serviceClient<architecture_msgs::PositionRequest>("target_position");
    // Topic subscribers 
    ros::Subscriber frontiers_reply_sub = nh.subscribe<frontiers_msgs::FrontierReply>("frontiers_reply", 5, state_manager_node::frontier_cb);
    ros::Subscriber ltstar_reply_sub = nh.subscribe<lazy_theta_star_msgs::LTStarReply>("ltstar_reply", 5, state_manager_node::ltstar_cb);
    // Topic publishers
    state_manager_node::ltstar_request_pub = nh.advertise<lazy_theta_star_msgs::LTStarRequest>("ltstar_request", 10);
    state_manager_node::frontier_request_pub = nh.advertise<frontiers_msgs::FrontierRequest>("frontiers_request", 10);
    state_manager_node::marker_pub = nh.advertise<visualization_msgs::MarkerArray>("state_manager_viz", 1);

    #ifdef SAVE_LOG
    state_manager_node::log_file.open (state_manager_node::folder_name+"/current/state_manager.log", std::ofstream::app);
    #endif
    state_manager_node::init_state_variables(state_manager_node::state_data, nh);

    rviz_interface::PublishingInput pi(state_manager_node::marker_pub, true, "oppairs" );

    geometry_msgs::Point geofence_min_point, geofence_max_point;
    geofence_min_point.x = state_manager_node::geofence_min.x();
    geofence_min_point.y = state_manager_node::geofence_min.y();
    geofence_min_point.z = state_manager_node::geofence_min.z();
    geofence_max_point.x = state_manager_node::geofence_max.x();
    geofence_max_point.y = state_manager_node::geofence_max.y();
    geofence_max_point.z = state_manager_node::geofence_max.z();
    goal_state_machine::GoalStateMachine goal_state_machine (state_manager_node::state_data.frontiers_msg, state_manager_node::distance_inFront, state_manager_node::distance_behind, state_manager_node::circle_divisions, geofence_min_point, geofence_max_point, pi, check_flightCorridor_client, state_manager_node::ltstar_safety_margin, state_manager_node::safety_margin);
    state_manager_node::state_data.goal_state_machine = std::make_shared<goal_state_machine::GoalStateMachine>(goal_state_machine);

    #ifdef SAVE_CSV
    std::ofstream csv_file;
    csv_file.open (state_manager_node::folder_name+"/exploration_time.csv", std::ofstream::app);
    // csv_file << "timestamp,computation_time_millis,volume_cubic_meters" << std::endl;
    csv_file << std::put_time(std::localtime(&now_c), "%F %T") << ",";
    csv_file.close();
    // state_manager_node::start = std::chrono::high_resolution_clock::now();
    #endif
    state_manager_node::timer = nh.createTimer(ros::Duration(0.5), state_manager_node::main_loop);
    ros::spin();
    return 0;
}
