
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <octomap/math/Vector3.h>
#include <unordered_set>
#include <visualization_msgs/Marker.h>

#include <utility>   
#include <sstream>
#include <fstream>
#include <string>
#include <chrono>
#include <boost/filesystem.hpp>

#include <architecture_math.h>
#include <goal_state_machine.h>
#include <exploration_state_machine.h>
#include <observation_maneuver.h>
#include <marker_publishing_utils.h>

#include <architecture_msgs/PositionRequest.h>
#include <architecture_msgs/PositionMiddleMan.h>
#include <architecture_msgs/FindNextGoal.h>
#include <architecture_msgs/DeclareUnobservable.h>

#include <frontiers_msgs/CheckIsFrontier.h>
#include <frontiers_msgs/CheckIsExplored.h>
#include <frontiers_msgs/FrontierNodeStatus.h>
#include <frontiers_msgs/VoxelMsg.h>

#include <lazy_theta_star_msgs/LTStarReply.h>
#include <lazy_theta_star_msgs/LTStarRequest.h>
#include <lazy_theta_star_msgs/LTStarNodeStatus.h>



#define SAVE_CSV 1
#define SAVE_LOG 1


namespace state_manager_node
{
    std::stringstream aux_envvar_home (std::getenv("HOME"));
    std::string folder_name = aux_envvar_home.str() + "/Flying_Octomap_code/src/data";


    ros::Publisher frontier_request_pub;
    ros::Publisher ltstar_request_pub;
    ros::Publisher marker_pub;
    ros::Publisher flight_plan_pub;
    ros::ServiceClient ltstar_status_cliente;
    ros::ServiceClient ask_for_goal_client;

    ros::Timer timer;
    std::chrono::high_resolution_clock::time_point start;
    bool is_successfull_exploration = false;
    std::ofstream log_file;
    #ifdef SAVE_CSV
    std::ofstream csv_file, csv_file_success;
    std::chrono::high_resolution_clock::time_point operation_start, timeline_start;
    #endif

    
    double px4_loiter_radius;
    double odometry_error;
    double sensing_distance = 3;
    double error_margin;
    double distance_inFront, distance_behind;
    int circle_divisions = 12;
    int uav_id_= 1;
    int max_time_secs =5000;
    double ltstar_safety_margin;
    octomath::Vector3 geofence_min (-5, -5, 1);
    octomath::Vector3 geofence_max (5, 5, 10);
    struct StateData { 
        int frontier_request_id;    // id for the request in use
        int frontier_request_count; // generate id for new frontier requests
        int ltstar_request_id;
        bool new_map;
        architecture_msgs::FindNextGoal::Response next_goal_msg;
        lazy_theta_star_msgs::LTStarRequest ltstar_request;
        lazy_theta_star_msgs::LTStarReply ltstar_reply;
        exploration_sm::ExplorationStateMachine exploration_state;
    };  
    state_manager_node::StateData state_data;

    std::pair<double, double> calculateTime()
    {
        auto end_millis         = std::chrono::high_resolution_clock::now();
        
        auto time_span          = std::chrono::duration_cast<std::chrono::duration<double>>(end_millis - operation_start);
        double operation_millis = std::chrono::duration_cast<std::chrono::milliseconds>(time_span).count();

        time_span               = std::chrono::duration_cast<std::chrono::duration<double>>(end_millis - timeline_start);
        double timeline_millis  = std::chrono::duration_cast<std::chrono::milliseconds>(time_span).count();
        operation_start         = std::chrono::high_resolution_clock::now();
        return std::make_pair (timeline_millis, operation_millis);
    }

    void askForObstacleAvoidingPath()
    {
        lazy_theta_star_msgs::LTStarNodeStatus srv;
        if(ltstar_status_cliente.call(srv))
        {
            if((bool)srv.response.is_accepting_requests)
            {
                lazy_theta_star_msgs::LTStarRequest request;
                state_data.ltstar_request_id++;
                request.request_id = state_data.ltstar_request_id;
                request.header.frame_id = "world";
                request.start = state_data.ltstar_reply.waypoints[(state_data.ltstar_reply.waypoints.size()-1)].position;
                request.goal  = state_data.next_goal_msg.start_flyby;
                request.safety_margin = ltstar_safety_margin;
                if(state_data.next_goal_msg.global)
                {
                    request.max_time_secs = max_time_secs;
                }
                else
                {
                    request.max_time_secs = max_time_secs/4;
                }
                #ifdef SAVE_LOG
                log_file << "[State manager] Requesting path " << request << std::endl;
                #endif
                ROS_INFO_STREAM ("[State manager] Requesting path from " << request.start << " to " << request.goal);
                ltstar_request_pub.publish(request);
                state_data.ltstar_request = request;
                state_data.exploration_state.switchState(exploration_sm::waiting_path_response);
            }
        }
        else
        {
            ROS_WARN("[State manager] Lazy Theta Star node not accepting requests.");
        }
    }

    void publishGeofence()
    {
        visualization_msgs::MarkerArray marker_array;
        rviz_interface::publish_geofence(geofence_min, geofence_max, marker_array);
        marker_pub.publish(marker_array);
    }


    bool askForGoalServiceCall() 
    { 
        publishGeofence();
        architecture_msgs::FindNextGoal find_next_goal;
        find_next_goal.request.new_map = state_data.new_map;
        if(ask_for_goal_client.call(find_next_goal)) 
        { 
            state_data.next_goal_msg = find_next_goal.response;
            state_data.new_map = false;
            return true;
        } 
        else 
        { 
            ROS_WARN("[State manager] Goal SM node not accepting next goal requests."); 
            return false; 
        } 
    } 

    nav_msgs::Path generateFlightPlanRequest()
    {
        nav_msgs::Path flight_plan_request;
        geometry_msgs::PoseStamped pose_s;
        for (std::vector<geometry_msgs::Pose>::iterator i = state_data.ltstar_reply.waypoints.begin(); i != state_data.ltstar_reply.waypoints.end(); ++i)
        {
            pose_s.pose = *i;
            flight_plan_request.poses.push_back(pose_s);
        }
        pose_s.pose.position = state_data.next_goal_msg.end_flyby;
        flight_plan_request.poses.push_back(pose_s);

        std::ofstream pathWaypoints;
        pathWaypoints.open (folder_name + "/current/final_path_state_manager.txt", std::ofstream::out | std::ofstream::app);
        for (std::vector<geometry_msgs::PoseStamped>::iterator i = flight_plan_request.poses.begin(); i != flight_plan_request.poses.end(); ++i)
        {
            pathWaypoints << std::setprecision(5) << i->pose.position.x << ", " << i->pose.position.y << ", " << i->pose.position.z << std::endl;
            
        }
        pathWaypoints.close();
        return flight_plan_request;
    }

    void findTarget()
    {
        #ifdef SAVE_LOG
            log_file << "[State manager][Exploration] exploration_start. Asked for next goal." << std::endl;
        #endif

        while(!askForGoalServiceCall()){}

        if(!state_data.next_goal_msg.success)
        {
            ROS_INFO_STREAM("[State manager][Exploration] finished_exploring - no frontiers reported.");
            log_file << "[State manager][Exploration] finished_exploring - no frontiers reported." << std::endl;
            is_successfull_exploration = true;
            state_data.exploration_state.switchState(exploration_sm::finished_exploring);
        }
        else
        {
            state_data.exploration_state.switchState(exploration_sm::generating_path);
            askForObstacleAvoidingPath();
        }
    }

    void ltstar_cb(const lazy_theta_star_msgs::LTStarReply::ConstPtr& msg)
    {
        if(state_data.exploration_state.getState() != exploration_sm::waiting_path_response)
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
                nav_msgs::Path flight_plan_request = generateFlightPlanRequest();
                state_data.new_map = true;
                state_data.exploration_state.switchState(exploration_sm::visit_waypoints);
                flight_plan_pub.publish(flight_plan_request);
                #ifdef SAVE_LOG
                    log_file << "[State manager] Path reply " << *msg << std::endl;
                    log_file << "[State manager] Flight plan request " << flight_plan_request << std::endl;
                #endif
            }
            else
            {
                ROS_WARN_STREAM (     "[State manager] Path reply failed!");
                state_data.exploration_state.switchState(exploration_sm::exploration_start);
                #ifdef SAVE_LOG
                    log_file << "[State manager] Path reply failed!" << std::endl;
                #endif
                findTarget();

            }
            #ifdef SAVE_CSV
                std::pair <double, double> millis_count = calculateTime(); 
                operation_start = std::chrono::high_resolution_clock::now();
                int success = 2;
                if (msg->success) success = 1;
                else success = 0;
                csv_file_success << millis_count.first << "," << success << "," << std::endl;
            #endif
        }
    }

    void flighPlan_cb(const std_msgs::Empty::ConstPtr& msg)
    {
        state_data.exploration_state.switchState(exploration_sm::exploration_start);
        findTarget();
    }

    void init_state_variables(state_manager_node::StateData& state_data, ros::NodeHandle& nh)
    {
        state_data.new_map = true;
        state_data.ltstar_request_id = 0;
        state_data.frontier_request_count = 0;
        geometry_msgs::Point start_position;
        start_position.x = 0;
        start_position.y = -6;
        start_position.z = 3;
        geometry_msgs::Pose start_pose;
        start_pose.position = start_position;
        std::vector<geometry_msgs::Pose> initial (1, start_pose);
        state_data.ltstar_reply.waypoints = initial;
    }

    void init_param_variables(ros::NodeHandle& nh)
    {
        double temp;
        nh.getParam("px4_loiter_radius", px4_loiter_radius);
        nh.getParam("odometry_error", odometry_error);
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

        nh.getParam("uav_id", uav_id_);

        // Goal state machine
        nh.getParam("oppairs/sensing_distance", sensing_distance);
        nh.getParam("oppairs/distance_inFront", distance_inFront);
        nh.getParam("oppairs/distance_behind",  distance_behind);
        nh.getParam("oppairs/circle_divisions",  circle_divisions);
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
    state_manager_node::ltstar_status_cliente      = nh.serviceClient<lazy_theta_star_msgs::LTStarNodeStatus>   ("ltstar_status");
    state_manager_node::ask_for_goal_client        = nh.serviceClient<architecture_msgs::FindNextGoal>          ("find_next_goal");
    // Topic subscribers 
    ros::Subscriber ltstar_reply_sub = nh.subscribe<lazy_theta_star_msgs::LTStarReply>("ltstar_reply", 5, state_manager_node::ltstar_cb);
    ros::Subscriber flighPlan_notifications_sub = nh.subscribe<std_msgs::Empty>("/uav_" + std::to_string(state_manager_node::uav_id_) + "/flight_plan_notifications", 5, state_manager_node::flighPlan_cb);
    // Topic publishers
    state_manager_node::ltstar_request_pub = nh.advertise<lazy_theta_star_msgs::LTStarRequest>("ltstar_request", 10);
    state_manager_node::marker_pub = nh.advertise<visualization_msgs::MarkerArray>("state_manager_viz", 1);
    state_manager_node::flight_plan_pub = nh.advertise<nav_msgs::Path>("/uav_" + std::to_string(state_manager_node::uav_id_) + "/flight_plan_requests", 1);


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

    #ifdef SAVE_CSV
    state_manager_node::csv_file_success.open (state_manager_node::folder_name+"/current/success_rate.csv", std::ofstream::app);
    state_manager_node::csv_file_success << "timeline,path_planner,sampling" << std::endl;
    state_manager_node::operation_start = std::chrono::high_resolution_clock::now();
    state_manager_node::timeline_start = std::chrono::high_resolution_clock::now();
    #endif
    // state_manager_node::timer = nh.createTimer(ros::Duration(30), state_manager_node::main_loop);
    ros::spin();
    return 0;
}
