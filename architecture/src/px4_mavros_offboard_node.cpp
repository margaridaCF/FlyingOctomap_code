/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/TwistStamped.h>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped new_frontier;
geometry_msgs::PoseStamped next_waypoint;
ros::Publisher stop_position_pub;
bool enable_stop, has_goal;

void askObstacleAvoidingPath ()
{
    // Make Lazy Theta Star request
    // TODO - this bit will go into the callback when there is one.
    has_waypoint_sequence = true;
    // change code to reflect a waypoint sequence instead of just one point
    next_waypoint = new_frontier;
}

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

bool notOffboardMode(ros::Time& last_request)
{
    return current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0));
}

bool disarmed(ros::Time& last_request)
{
    return !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0));
}

bool reachedGoal(geometry_msgs::PoseStamped const& pose)
{
    return pose.pose.position.x == new_frontier.pose.position.x 
        && pose.pose.position.y == new_frontier.pose.position.y 
        && pose.pose.position.z == new_frontier.pose.position.z;
}

void stopUAV(ros::Publisher const& stop_position_pub)
{
    // FLAGS
    enable_stop = true;
    finished_waypoint_sequence = true;
    has_waypoint_sequence = false;
    is_flying_to_next_goal = false;
    // COMMAND
    mavros_msgs::PositionTarget stop_position;
    stop_position.type_mask = 0b0000101111000111;
    stop_position.header.stamp = ros::Time::now();
    stop_position.header.seq=1;
    stop_position_pub.publish(stop_position);
    ROS_INFO("STOP msg sent!");
}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
        current_state = *msg;
}

void stop_cb(const std_msgs::Empty::ConstPtr& msg)
{
    stopUAV();
}

void frontier_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
        new_frontier = *msg;
        has_goal = true;
        ROS_INFO_STREAM("[Offboard node] New frontier ("
            <<new_frontier.pose.position.x << ", "
            <<new_frontier.pose.position.y << ", "
            <<new_frontier.pose.position.z << ") ");
}

bool set_mavros_param(std::string param_name, double param_value)
{
    ros::ServiceClient param_set_client = n.serviceClient<mavros_msgs::ParamSet>("mavros/param/set");
    mavros_msgs::ParamSet set_MPC_XY_CRUISE_srv;
    set_MPC_XY_CRUISE_srv.request.param_id = param_name;
    set_MPC_XY_CRUISE_srv.request.value.integer = param_value;
    return param_set_client.call(set_MPC_XY_CRUISE_srv);
}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "offb_node");
        ros::NodeHandle nh;

        ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
        ros::Subscriber stop_sub = nh.subscribe<std_msgs::Empty>("/stop_uav", 10, stop_cb);
        ros::Subscriber waypoints_sub = nh.subscribe<geometry_msgs::PoseStamped>("new_frontier", 10, frontier_cb);

        stop_position_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
        ros::Publisher frontier_request_pub = nh.advertise<frontiers_msgs::FrontierRequest>("frontiers_request", 10);
        ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

        ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
        ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

        int request_count = 1;
        int current_goal_request_id = 0;
        has_goal = false;
        bool finished_waypoint_sequence = true;
        bool has_waypoint_sequence = false;
        bool is_flying_to_next_goal = false;
        octomath::Vector3 geofence_min (0, 0, 0);
        octomath::Vector3 geofence_max (6, 2, 2);


        //the setpoint publishing rate MUST be faster than 2Hz
        ros::Rate rate(20);

        // === FCU CONNECTION ===
        while(ros::ok() && current_state.connected) {
                ros::spinOnce();
                rate.sleep();
        }

        // === VELOCITY ===
        // LNDMC_XY_VEL_MAX
        // MPC_ACC_DOWN_MAX
        // MPC_XY_CRUISE
        // MPC_XY_VEL_MAX
        // // param show MPC_XY_CRUISE
        // //     Symbols: x = used, + = saved, * = unsaved
        // //     x + MPC_XY_CRUISE [292,580] : 7.0000
        // //      1055 parameters total, 654 used.
        // rosservice call /mavros/param/set "param_id: 'MPC_XY_CRUISE'
        //     value:
        //       integer: 0
        //       real: 7.0" 
        double const flight_speed = 0.01;
        velocity_set =  false;
        while(!velocity_set)
        {
            velocity_set = set_mavros_param("MPC_XY_CRUISE", flight_speed);
        }
        ROS_INFO_STREAM("Velocity set to " << flight_speed << "(MPC_XY_CRUISE)");


        // === SET WAYPOINTS ===
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 1;
        //send a few setpoints before starting
        for(int i = 20; ros::ok() && i > 0; --i) {
                local_pos_pub.publish(pose);
                ros::spinOnce();
                rate.sleep();
        }

        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;
        ros::Time last_request = ros::Time::now();
        while(ros::ok()) 
        {
            if( notOffboardMode(last_request)) 
            {
                if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                        ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            } 
            else 
            {
                if( disarmed(last_request)) 
                {
                    if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success) 
                    {
                        ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                } 
                else if (current_state.armed) 
                {
                    // === ToDo - Is all space explored? ===
                    // === Is the goal defined? ===
                    if (!has_goal)
                    {
                        current_goal_request_id = request_count;
                        askForGoal(request_count, geofence_min, geofence_max, frontier_request_pub);
                    }
                    else if(!has_waypoint_sequence)
                    {
                        askObstacleAvoidingPath();
                    }
                    else if(!is_flying_to_next_goal)
                    {
                        // send waypoint to start flying
                        local_pos_pub.publish(next_waypoint);
                    }
                }
            }
            ros::spinOnce();
            rate.sleep();
        }

        return 0;
}
