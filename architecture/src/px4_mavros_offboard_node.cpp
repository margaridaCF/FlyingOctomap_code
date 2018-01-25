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
bool enable_stop;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
        current_state = *msg;
}

void stop_cb(const std_msgs::Empty::ConstPtr& msg){
        enable_stop = true;
}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "offb_node");
        ros::NodeHandle nh;

        ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                                            ("mavros/state", 10, state_cb);
        ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                               ("mavros/setpoint_position/local", 10);
        ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                                   ("mavros/cmd/arming");
        ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                                     ("mavros/set_mode");

        ros::Subscriber stop_sub = nh.subscribe<std_msgs::Empty>("/stop_uav", 10, stop_cb);
        ros::Publisher stop__velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
        ros::Publisher stop_position_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

        //the setpoint publishing rate MUST be faster than 2Hz
        ros::Rate rate(20);

        // wait for FCU connection
        while(ros::ok() && current_state.connected) {
                ros::spinOnce();
                rate.sleep();
        }




        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 2;

        geometry_msgs::TwistStamped stop_velocity;
        // stop_velocity.twist.linear.x = 0.0;
        // stop_velocity.twist.linear.y = 0.0;
        // stop_velocity.twist.linear.z = 0.0;
        // stop_velocity.twist.angular.x = 0.0;
        // stop_velocity.twist.angular.y = 0.0;
        // stop_velocity.twist.angular.z = 0.0;

        mavros_msgs::PositionTarget stop_position;
        stop_position.type_mask = 0b0000101111000111;


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


        while(ros::ok()) {
                if( current_state.mode != "OFFBOARD" &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))) {
                        if( set_mode_client.call(offb_set_mode) &&
                            offb_set_mode.response.mode_sent) {
                                ROS_INFO("Offboard enabled");
                        }
                        last_request = ros::Time::now();
                } else {
                        if( !current_state.armed &&
                            (ros::Time::now() - last_request > ros::Duration(5.0))) {
                                if( arming_client.call(arm_cmd) &&
                                    arm_cmd.response.success) {
                                        ROS_INFO("Vehicle armed");
                                }
                                last_request = ros::Time::now();
                        } else if (current_state.armed) {
                                if (enable_stop == false) {
                                        pose.pose.position.x +=0.03;
                                        ROS_INFO("UAV proceeding in straight line");
                                } else {
                                        // pose.pose.position.x = 0;
                                        // pose.pose.position.y = 0;
                                        // pose.pose.position.z = 0;
                                        // NOTE: to stop the UAV, you can just send always the current pose, meaning you don't change the message sent
                                        stop_velocity.header.stamp = ros::Time::now();
                                        stop_velocity.header.seq=1;
                                        // stop_velocity_pub.publish(stop_velocity);
                                        stop_position.header.stamp = ros::Time::now();
                                        stop_position.header.seq=1;
                                        stop_position_pub.publish(stop_position);
                                        ROS_INFO("STOP msg sent!");
                                        // Allow the UAV to start again moving
                                        enable_stop = false;
                                }
                        }
                }
                local_pos_pub.publish(pose);
                // Publish new waypoints only once the UAV is armed
                // if(current_state.armed){
                //   if (enable_stop == false){
                //     pose.pose.position.x +=0.05;
                //     ROS_INFO("UAV proceeding in straight line");
                //   } else {
                //     // pose.pose.position.x = 0;
                //     // pose.pose.position.y = 0;
                //     // pose.pose.position.z = 0;
                //     // NOTE: to stop the UAV, you can just send always the current pose, meaning you don't change the message sent
                //     ROS_INFO("STOP msg sent!");
                //     // Allow the UAV to start again moving
                //     enable_stop = false;
                //   }
                //   local_pos_pub.publish(pose);
                //
                // }

                ros::spinOnce();
                rate.sleep();
        }

        return 0;
}
