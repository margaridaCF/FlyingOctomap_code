
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
    std::string folder_name = "/home/hector/Flying_Octomap_code/src/data";

    struct Vector3Hash
    {
        std::size_t operator()(const octomath::Vector3 & v) const 
        {
            int scale = 0.00001;
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
    ros::ServiceClient check_flightCorridor_client;

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
    ros::Duration exploration_maneuver_duration_secs;
    int max_time_secs =5000;
    int max_cycles_waited_for_path = 3;
    double ltstar_safety_margin;
    octomath::Vector3 geofence_min (-5, -5, 1);
    octomath::Vector3 geofence_max (5, 5, 10);
    enum follow_path_state_t{init, on_route, arrived_at_waypoint, finished_sequence};
    enum exploration_state_t {clear_from_ground, exploration_start, generating_path, waiting_path_response, visit_waypoints, finished_exploring, gather_data_maneuver};
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
        frontiers_msgs::FrontierRequest frontiers_request;
        frontiers_msgs::FrontierReply frontiers_msg;
        lazy_theta_star_msgs::LTStarRequest ltstar_request;
        lazy_theta_star_msgs::LTStarReply ltstar_reply;
        std::unordered_set<octomath::Vector3, Vector3Hash> unobservable_set; 
        observation_lib::OPPairs oppairs;
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
        log_file << "[State manager] Requesting position " << state_data.waypoint_index << " = " << get_current_waypoint() << std::endl;
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
        request.frontier_amount = state_data.unobservable_set.size()+10;
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

    void initializeOPPairsNewFrontier()
    {
        geometry_msgs::Point curr_frontier_geom = get_current_frontier();
        Eigen::Vector3d new_frontier(curr_frontier_geom.x, curr_frontier_geom.y, curr_frontier_geom.z);
        geometry_msgs::Point point;
        while(!getUavPositionServiceCall(point))
        {
            ROS_ERROR_STREAM("[State Manager] Cannot get uav position to initialize observations point pairs for new frontier.");
        }
        Eigen::Vector3d uav_position (point.x, point.y, point.z);
        rviz_interface::PublishingInput pi(marker_pub, true, "oppairs" );
        state_data.oppairs.NewFrontier(new_frontier, uav_position, pi);
        log_file << "[State Manager] [oppairs] 1. New frontier. Reset all." << std::endl;
    }


    bool is_flightCorridor_free()
    {
        lazy_theta_star_msgs::CheckFlightCorridor srv;
        Eigen::Vector3d start_eigen = state_data.oppairs.get_current_start();
        Eigen::Vector3d end_eigen = state_data.oppairs.get_current_end();
        
        srv.request.start.x = start_eigen(0);
        srv.request.start.y = start_eigen(1);
        srv.request.start.z = start_eigen(2);
        srv.request.end.x   = end_eigen(0);
        srv.request.end.y   = end_eigen(1);
        srv.request.end.z   = end_eigen(2);
        srv.request.flight_corridor_width = ltstar_safety_margin;

        bool check = true;
        while(check)
        {
            check = check_flightCorridor_client.call(srv);
            if(!check)
            {
                ROS_ERROR("[State Manager] Cannot place request to check flight corridor for flyby.");
            }
        }
        return srv.response.free;
    }

    bool find_next_safe_oppair()
    {
        while(state_data.oppairs.Next())
        {
            log_file << "[State Manager] [oppairs] 2. Next oppair" << std::endl;
            if(is_flightCorridor_free()) return true;
        }
        log_file << "[State Manager] [oppairs] 3. Analyzed all oppairs, none suitable." << std::endl;
        return false;
    }

    void dealUnreachableGoal(std::string log_id)
    {
        octomath::Vector3 unreachable (get_current_frontier().x, get_current_frontier().y, get_current_frontier().z);
        state_data.unobservable_set.insert(unreachable);
        // If there are more ways to observe the frontier search for it
        if(find_next_safe_oppair())
        {
            state_data.exploration_state = generating_path; 
            state_data.waypoint_index = -1;
        }
        else 
        {
            // Need new frontier.
            // Now figure out if it will be from the batch of frontiers I got from frontier node or if I need to ask for new frontier batch
            if (state_data.frontier_index >= state_data.frontiers_msg.frontiers_found-1)
            {
                state_data.exploration_state = exploration_start;
            }
            else
            {
                initializeOPPairsNewFrontier();
                log_file << "[State Manager] [oppairs] 1. New frontier from cache. Reset all." << std::endl;
                state_data.frontier_index = state_data.frontier_index +1;
                state_data.exploration_state = generating_path; 
                state_data.waypoint_index = -1;
            }
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
                log_file << "[State manager][Exploration] visit_waypoints 3" << std::endl;
                log_file << "[State manager]            [Follow path] init" << std::endl;
#endif
            }
            else
            {
                ROS_WARN_STREAM (     "[State manager] Path reply failed!");
                dealUnreachableGoal("ltstar_cb");
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
            state_data.frontier_request_id = msg->request_id;
            state_data.exploration_state = generating_path;
            // state_data.frontier_index = 0;
            state_data.frontiers_msg = *msg;
            initializeOPPairsNewFrontier();
            log_file << "[State Manager] [oppairs] 1. New frontier arrived from node. Reset all." << std::endl;
#ifdef SAVE_LOG
            log_file << "[State manager]Frontier reply " << *msg << std::endl;
#endif
            ROS_INFO_STREAM (     "[State manager] found " << msg->frontiers_found << " frontiers." );
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

    bool updateWaypointSequenceStateMachine()
    {
        switch(state_data.follow_path_state)
        {
            case init:
            {
                // ROS_WARN_STREAM("[State manager]            [Path follow] updateWaypointSequenceStateMachine at init");
                if(askPositionServiceCall(get_current_waypoint()))
                {
                    state_data.follow_path_state = on_route;
#ifdef SAVE_LOG
            log_file << "[State manager]            [Path follow] on_route to " << get_current_waypoint() << std::endl;
#endif
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
                geometry_msgs::Point target_waypoint = get_current_waypoint().position;
                if(hasArrived(target_waypoint))
                {
                    state_data.follow_path_state = arrived_at_waypoint;
                }
                break;
            }
            case arrived_at_waypoint:
            {
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

    void init_state_variables(state_manager_node::StateData& state_data)
    {
        state_data.exploration_maneuver_started = false;
        state_data.handbrake_enabled = false;
        state_data.ltstar_request_id = 0;
        state_data.frontier_request_count = 0;
        state_data.exploration_state = clear_from_ground;
#ifdef SAVE_LOG
        log_file << "[State manager][Exploration] clear_from_ground" << std::endl;
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
        double laser_angle;
        nh.getParam("laser_angle", laser_angle);
        laser_range_xy = std::cos(laser_angle)*laser_range;
        ROS_INFO_STREAM("[architecture] Laser xy range: " << laser_range_xy);
        nh.getParam("odometry_error", odometry_error);
        nh.getParam("frontier/safety_margin", safety_margin);
        error_margin = std::max(px4_loiter_radius, odometry_error);
        nh.getParam("path/max_time_secs", max_time_secs);
        nh.getParam("path/max_cycles_waited_for_path", max_cycles_waited_for_path);
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
    }

    void init_oppairs (ros::NodeHandle& nh)
    {
        double distance_inFront, distance_behind;
        int circle_divisions = 12;
        nh.getParam("oppairs/distance_inFront", distance_inFront);
        nh.getParam("oppairs/distance_behind",  distance_behind);
        nh.getParam("oppairs/circle_divisions",  circle_divisions);
        state_data.oppairs = observation_lib::OPPairs(circle_divisions, safety_margin, distance_inFront, distance_behind);
        log_file << "[State Manager] [oppairs] 0. Precalculation." << std::endl;

    }

    double calculateOrientation(Eigen::Vector3d start, Eigen::Vector3d end)
    {
        Eigen::Vector3d d = end - start;
        d.normalize();
        return std::acos (1/d.norm());
    }

    void update_state(octomath::Vector3 const& geofence_min, octomath::Vector3 const& geofence_max)
    {
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
                    waypoint.position.x = current_position.x;
                    waypoint.position.y = current_position.y;
                    waypoint.position.z = std::min(geofence_max.z(), 10.f);
                    waypoint.orientation = tf::createQuaternionMsgFromYaw(0);
                    Eigen::Vector3d fake_uav_position (waypoint.position.x, waypoint.position.y, waypoint.position.z);
                    state_data.ltstar_reply.waypoints.push_back(waypoint);
                    waypoint.position.x = current_position.x;
                    waypoint.position.y = current_position.y;
                    waypoint.position.z = geofence_min.z();
                    waypoint.orientation = tf::createQuaternionMsgFromYaw(180  * 0.0174532925);
                    Eigen::Vector3d fake_frontier_e (waypoint.position.x, waypoint.position.y, waypoint.position.z);
                    state_data.ltstar_reply.waypoints.push_back(waypoint);
                    state_data.frontiers_msg.frontiers_found = 1;
                    state_data.ltstar_reply.waypoint_amount = 2;

                    rviz_interface::PublishingInput pi(marker_pub, true, "oppairs" );
                    state_data.oppairs.NewFrontier(fake_frontier_e, fake_uav_position, pi);

#ifdef SAVE_LOG
                    log_file << "[State manager][Exploration] visit_waypoints 2" << std::endl;
                    log_file << "[State manager]            [Follow path] init" << std::endl;
#endif
                    frontiers_msgs::VoxelMsg fake_frontier;
                    fake_frontier.xyz_m = current_position;
                    state_data.frontiers_msg.frontiers.push_back(fake_frontier);
                    state_data.frontier_index = 0;
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
            case generating_path:
            {
                lazy_theta_star_msgs::LTStarNodeStatus srv;
                if(ltstar_status_cliente.call(srv))
                {
                    if((bool)srv.response.is_accepting_requests)
                    {
                        geometry_msgs::Point current_position;
                        if(getUavPositionServiceCall(current_position))
                        {
                            visualization_msgs::MarkerArray marker_array;
                            octomath::Vector3 current_position_v (current_position.x, current_position.y, current_position.z);
                            octomath::Vector3 start(current_position.x, current_position.y, current_position.z);
                            Eigen::Vector3d oppair_start = state_data.oppairs.get_current_start();
                            octomath::Vector3 goal (oppair_start(0), oppair_start(1), oppair_start(2));
                            askForObstacleAvoidingPath(start, goal, ltstar_request_pub);
                            state_data.exploration_state = waiting_path_response;
                            state_data.cycles_waited_for_path = 0;

                            geometry_msgs::Point frontier_geom = get_current_frontier();
                            geometry_msgs::Point start_geom;
                            start_geom.x = current_position.x;
                            start_geom.y = current_position.y;
                            start_geom.z = current_position.z;
                            geometry_msgs::Point oppair_start_geom;
                            oppair_start_geom.x = oppair_start(0); 
                            oppair_start_geom.y = oppair_start(1);
                            oppair_start_geom.z = oppair_start(2);
                            geometry_msgs::Point oppair_end_geom;
                            oppair_end_geom.x = state_data.oppairs.get_current_end()(0); 
                            oppair_end_geom.y = state_data.oppairs.get_current_end()(1);
                            oppair_end_geom.z = state_data.oppairs.get_current_end()(2);
                            rviz_interface::build_stateManager(frontier_geom, oppair_start_geom, oppair_end_geom, start_geom, state_data.frontiers_request.safety_margin, marker_array);
                            marker_pub.publish(marker_array);
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

                break;
            }
            case visit_waypoints:
            {
                updateWaypointSequenceStateMachine();

                if (state_data.follow_path_state == finished_sequence)
                {
                    state_data.exploration_state = exploration_start;
#ifdef SAVE_LOG
                    log_file << "[State manager][Exploration] gather_data_maneuver" << std::endl;
#endif
                }
                break;
            }
            case gather_data_maneuver:
            {
                geometry_msgs::Pose flyby_end;
                flyby_end.position.x = state_data.oppairs.get_current_end()(0);
                flyby_end.position.y = state_data.oppairs.get_current_end()(1);
                flyby_end.position.z = state_data.oppairs.get_current_end()(2);
                if (!state_data.exploration_maneuver_started)
                {
                    flyby_end.orientation = tf::createQuaternionMsgFromYaw(calculateOrientation(state_data.oppairs.get_current_start(), state_data.oppairs.get_current_end()));
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
    state_manager_node::check_flightCorridor_client = nh.serviceClient<lazy_theta_star_msgs::CheckFlightCorridor>("is_fligh_corridor_free");
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
    state_manager_node::init_oppairs(nh);
    state_manager_node::init_state_variables(state_manager_node::state_data);
#ifdef SAVE_CSV
    std::ofstream csv_file;
    csv_file.open (state_manager_node::folder_name+"/exploration_time.csv", std::ofstream::app);
    // csv_file << "timestamp,computation_time_millis,volume_cubic_meters" << std::endl;
    csv_file << std::put_time(std::localtime(&now_c), "%F %T") << ",";
    csv_file.close();
    // state_manager_node::start = std::chrono::high_resolution_clock::now();
#endif
    state_manager_node::timer = nh.createTimer(ros::Duration(1), state_manager_node::main_loop);
    ros::spin();
    return 0;
}
