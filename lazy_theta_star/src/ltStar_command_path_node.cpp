
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <octomap/math/Vector3.h>
#include <visualization_msgs/Marker.h>

#include <sstream>
#include <fstream>
#include <string>
#include <chrono>
#include <boost/filesystem.hpp>

#include <marker_publishing_utils.h>

#include <architecture_msgs/PositionRequest.h>
#include <architecture_msgs/PositionMiddleMan.h>

#include <lazy_theta_star_msgs/LTStarReply.h>

#define SAVE_CSV 1
#define SAVE_LOG 1


namespace ltStar_command_path_node
{
    ros::Publisher marker_pub;
    ros::ServiceClient target_position_client;
    ros::ServiceClient current_position_client;

    ros::Timer timer;
    std::chrono::high_resolution_clock::time_point start;
    std::ofstream log_file;

    double odometry_error;
    double px4_loiter_radius;
    double error_margin;
    enum follow_path_state_t{init, on_route, arrived_at_waypoint, finished_sequence};
    enum exploration_state_t {clear_from_ground, waiting_path_response, visit_waypoints};
    struct StateData { 
        int waypoint_index;  // id of the waypoint that is currently the waypoint
        int ltstar_request_id;
        exploration_state_t exploration_state;
        follow_path_state_t follow_path_state;
        lazy_theta_star_msgs::LTStarReply ltstar_reply;
    };  
    ltStar_command_path_node::StateData state_data;

    // TODO when thre is generation of path these two will be different
    geometry_msgs::Pose& get_current_waypoint()
    {
        return state_data.ltstar_reply.waypoints[state_data.waypoint_index];
    }

    bool getUavPositionServiceCall(geometry_msgs::Point& current_position)
    {
        architecture_msgs::PositionMiddleMan srv;
        if(current_position_client.call(srv))
        {
            current_position = srv.response.current_position;

            // ROS_INFO_STREAM("[Command path] 1 Current (" << current_position.x << ", " << current_position.y << ", " << current_position.z << ");");
            return true;
        }
        else
        {
            ROS_WARN("[Command path] Current position middle man node not accepting requests.");
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
        log_file << "[Command path] Requesting position " << state_data.waypoint_index << " = " << get_current_waypoint() << std::endl;
#endif
            return position_request_srv.response.is_going_to_position;
        }
        else
        {
            ROS_WARN("[Command path] In YawSpin, node not accepting position requests.");
            return false;
        }
    }

    void ltstar_cb(const lazy_theta_star_msgs::LTStarReply::ConstPtr& msg)
    {
        if(msg->success)
        {
            state_data.ltstar_reply = *msg;
            state_data.follow_path_state = init;
            state_data.exploration_state = visit_waypoints;
            state_data.waypoint_index = 1;
#ifdef SAVE_LOG
            log_file << "[Command path] Path reply " << *msg << std::endl;
            log_file << "[Command path][Exploration] visit_waypoints 3" << std::endl;
            log_file << "[Command path]            [Follow path] init" << std::endl;
#endif
        }
        else
        {
            ROS_WARN_STREAM (     "[Command path] Path reply failed!");
        }
         
    }

    bool is_in_target_position(geometry_msgs::Point const& target_waypoint, 
        geometry_msgs::Point & current_position, double error_margin )
    {
        // ROS_INFO_STREAM("[Command path] Target position " << target_waypoint );
        // ROS_INFO_STREAM("[Command path] Current position " << current_position );
        // ROS_INFO_STREAM("[Command path] 3 Position offset");
        // ROS_INFO_STREAM("[Command path] Current (" << current_position.x << ", " << current_position.y << ", " << current_position.z << ");");
        // ROS_INFO_STREAM("[Command path]  Target (" << target_waypoint.x << ", " << target_waypoint.y << ", " << target_waypoint.z << ");");
        // ROS_INFO_STREAM("[Command path] Position offset (" << std::abs(target_waypoint.x - current_position.x) << ", "
        //     << std::abs(target_waypoint.y - current_position.y) << ", "
        //     << std::abs(target_waypoint.z - current_position.z) << ") ");

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
                // ROS_WARN_STREAM("[Command path]            [Path follow] updateWaypointSequenceStateMachine at init");
                if(askPositionServiceCall(get_current_waypoint()))
                {
                    state_data.follow_path_state = on_route;
#ifdef SAVE_LOG
            log_file << "[Command path]            [Path follow] on_route to " << get_current_waypoint() << std::endl;
#endif
                }
                else
                {
#ifdef SAVE_LOG
            log_file << "[Command path] Failed to set next position. Going to keep trying." << std::endl;
#endif
                }
                break;
            }
            case on_route:
            {
                // ROS_WARN_STREAM("[Command path]            [Path follow] updateWaypointSequenceStateMachine at on_route");
                geometry_msgs::Point current_position;
                if(getUavPositionServiceCall(current_position))
                {
                    // compare target with postition allowing for error margin 
                    // ROS_INFO_STREAM("[Command path] 2 Current (" << current_position.x << ", " << current_position.y << ", " << current_position.z << ");");
                    geometry_msgs::Point target_waypoint;
                    target_waypoint = get_current_waypoint().position;
                    if( is_in_target_position(target_waypoint, current_position, error_margin) )
                    {
                        state_data.follow_path_state = arrived_at_waypoint;
                    }
                }
                break;
            }
            case arrived_at_waypoint:
            {
                if(state_data.ltstar_reply.waypoint_amount == state_data.waypoint_index+1)
                {
                    // Reached Frontier
#ifdef SAVE_LOG
            log_file << "[Command path] Reached final waypoint (" << state_data.waypoint_index << ")" << std::endl;
            log_file << "[Command path]            [Path follow]  finished_sequence" << std::endl;
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

    void init_state_variables(ltStar_command_path_node::StateData& state_data)
    {
        state_data.ltstar_request_id = 0;
        state_data.exploration_state = clear_from_ground;
#ifdef SAVE_LOG
        log_file << "[Command path][Exploration] clear_from_ground" << std::endl;
#endif
    }

    void init_param_variables(ros::NodeHandle& nh)
    {
        nh.getParam("px4_loiter_radius", px4_loiter_radius);
        nh.getParam("odometry_error", odometry_error);
        error_margin = std::max(px4_loiter_radius, odometry_error);
    }

    void update_state(const ros::TimerEvent& event)
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
                    // state_data.frontier_request_id = 0;
                    state_data.waypoint_index = 0;
                    state_data.exploration_state = visit_waypoints;
                    state_data.follow_path_state = init;

                    geometry_msgs::Pose waypoint;
                    waypoint.position.x = 0;
                    waypoint.position.y = 0;
                    waypoint.position.z = 10;
                    waypoint.orientation = tf::createQuaternionMsgFromYaw(0);
                    state_data.ltstar_reply.waypoints.push_back(waypoint);
                    waypoint.position.x = -5;
                    waypoint.position.y = 5;
                    waypoint.position.z = 20;
                    waypoint.orientation = tf::createQuaternionMsgFromYaw(180  * 0.0174532925);
                    state_data.ltstar_reply.waypoints.push_back(waypoint);
                    waypoint.position.x = 10;
                    waypoint.position.y = 5;
                    waypoint.position.z = 20;
                    waypoint.orientation = tf::createQuaternionMsgFromYaw(0);
                    state_data.ltstar_reply.waypoints.push_back(waypoint);
                    waypoint.position.x = 10;
                    waypoint.position.y = 5;
                    waypoint.position.z = 2;
                    waypoint.orientation = tf::createQuaternionMsgFromYaw(180  * 0.0174532925);
                    state_data.ltstar_reply.waypoints.push_back(waypoint);
                    waypoint.position.x = 0;
                    waypoint.position.y = 0;
                    waypoint.position.z = 2;
                    waypoint.orientation = tf::createQuaternionMsgFromYaw(180  * 0.0174532925);
                    state_data.ltstar_reply.waypoints.push_back(waypoint);
                    // state_data.frontiers_msg.frontiers_found = 1;
                    state_data.ltstar_reply.waypoint_amount = 5;
#ifdef SAVE_LOG
                    log_file << "[Command path][Exploration] visit_waypoints 2" << std::endl;
                    log_file << "[Command path]            [Follow path] init" << std::endl;
#endif
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
                    state_data.exploration_state = waiting_path_response;
                }
                break;
            }
            default:
            {
                ROS_ERROR_STREAM("[Command path] Something went very wrong. State is unknown "<< state_data.exploration_state);
                break;
            }
        }
    }
}

int main(int argc, char **argv)
{
    auto timestamp_chrono = std::chrono::high_resolution_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(timestamp_chrono - std::chrono::hours(24));
    std::stringstream folder_name_stream;
    std::string folder_name = "/home/hector/Flying_Octomap_code/src/data/";
    folder_name_stream << folder_name << (std::put_time(std::localtime(&now_c), "%F %T") );
    std::string sym_link_name = folder_name+"/current";

    boost::filesystem::create_directories(folder_name_stream.str());
    boost::filesystem::create_directory_symlink(folder_name_stream.str(), sym_link_name);


    ros::init(argc, argv, "ltStar_command_path_node");
    ros::NodeHandle nh;
    ltStar_command_path_node::init_param_variables(nh);
    // Service client
    ltStar_command_path_node::current_position_client = nh.serviceClient<architecture_msgs::PositionMiddleMan>("get_current_position");
    ltStar_command_path_node::target_position_client = nh.serviceClient<architecture_msgs::PositionRequest>("target_position");
    // Topic subscribers 
    ros::Subscriber ltstar_reply_sub = nh.subscribe<lazy_theta_star_msgs::LTStarReply>("ltstar_reply", 5, ltStar_command_path_node::ltstar_cb);
    // Topic publishers
    ltStar_command_path_node::marker_pub = nh.advertise<visualization_msgs::Marker>("ltStar_command_path_node_viz", 1);

#ifdef SAVE_LOG
    ltStar_command_path_node::log_file.open (folder_name+"/current/ltStar_command_path_node.log", std::ofstream::app);
#endif
    ltStar_command_path_node::init_state_variables(ltStar_command_path_node::state_data);
    ltStar_command_path_node::timer = nh.createTimer(ros::Duration(1), ltStar_command_path_node::update_state);
    ros::spin();
    return 0;
}
