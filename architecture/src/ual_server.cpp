//----------------------------------------------------------------------------------------------------------------------
// GRVC UAL
//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2016 GRVC University of Seville
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------
#include <architecture_msgs/PositionRequest.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <uav_abstraction_layer/State.h>
#include <uav_abstraction_layer/ual.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <architecture_msgs/PositionMiddleMan.h>
#include <tf/transform_datatypes.h>

namespace ual_server
{
ros::ServiceClient current_position_client;
ros::Publisher pub_current_path, pub_target_path;

int uav_id;
bool offboard_enabled;

nav_msgs::Path uav_current_path, uav_target_path;
Eigen::Vector3f target_point, current_point, fix_pose_point;
geometry_msgs::PoseStamped target_pose, current_pose, fix_pose_pose;
geometry_msgs::TwistStamped velocity_to_pub;
Eigen::Quaterniond q_current, q_target;
uav_abstraction_layer::State uav_state;
double last_yaw, requested_yaw;
double px4_orientation_confusion_limit, px4_orientation_confusion_limit_degrees;
bool new_target = false;
bool new_orientation = false;
bool taking_off = false;
double position_tolerance;
double flight_level;
double distance_switch_wp_control;
double max_acceptance_orientation;
double min_acceptance_orientation;
enum movement_state_t { hover,
                        velocity,
                        fix_pose,
                        take_off };
movement_state_t movement_state;

void state_cb(const uav_abstraction_layer::State msg) {
    uav_state.state = msg.state;
    return;
}

void update_current_variables(geometry_msgs::PoseStamped ual_pose) {
    current_pose = ual_pose;
    current_point = Eigen::Vector3f(current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
    q_current.x() = current_pose.pose.orientation.x;
    q_current.y() = current_pose.pose.orientation.y;
    q_current.z() = current_pose.pose.orientation.z;
    q_current.w() = current_pose.pose.orientation.w;
    return;
}

void update_target_fix_variables(geometry_msgs::Pose fix_pose) {
    fix_pose_pose.pose.orientation = fix_pose.orientation;
    fix_pose_pose.pose.position = uav_target_path.poses.at(uav_target_path.poses.size() - 2).pose.position;  // Take previous target_pose.position to fix orientation before next target
    fix_pose_point = Eigen::Vector3f(fix_pose_pose.pose.position.x, fix_pose_pose.pose.position.y, fix_pose_pose.pose.position.z);
    return;
}

void update_yaw(double yaw)
{
    target_pose.pose.orientation   = tf::createQuaternionMsgFromYaw(yaw);
    fix_pose_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    q_target.x() = target_pose.pose.orientation.x;
    q_target.y() = target_pose.pose.orientation.y;
    q_target.z() = target_pose.pose.orientation.z;
    q_target.w() = target_pose.pose.orientation.w;
}

double yawDiff(double last_yaw, double requested_yaw)
{
    double yawDiff = 0;
    if (std::isnan(last_yaw))
    {
        yawDiff = requested_yaw;
    }
    else if (last_yaw >= 0 && requested_yaw >= 0)
    {
        yawDiff = std::abs(last_yaw - requested_yaw);
    }
    else if (last_yaw < 0 && requested_yaw < 0)
    { 
        yawDiff = std::abs(last_yaw - requested_yaw);
    }
    else if (last_yaw >= 0 && requested_yaw < 0)
    { 
        yawDiff = std::abs(requested_yaw) + last_yaw;
    }
    else if (last_yaw < 0 && requested_yaw >= 0)
    { 
        yawDiff = std::abs(last_yaw) + requested_yaw;
    }
    else
    {
        ROS_ERROR_STREAM("[ual] 1 There is a major bug in yawDiff. last_yaw " << last_yaw << " - requested_yaw " << requested_yaw );
    }
    return yawDiff;
}

double toDegrees(double radians)
{
    return (radians*(180/M_PI));
}

bool target_position_cb(architecture_msgs::PositionRequest::Request &req,
                        architecture_msgs::PositionRequest::Response &res) {
    if (uav_state.state == 4) {
        if (!new_target) 
        {
            movement_state = fix_pose;
            target_pose.pose = req.pose;
            target_pose.header.frame_id = "uav_1_home";
            uav_target_path.poses.push_back(target_pose);
            Eigen::Quaterniond q(req.pose.orientation.w, req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z);
            auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
            last_yaw = tf::getYaw(fix_pose_pose.pose.orientation);
            requested_yaw = tf::getYaw(req.pose.orientation);
            double amplitude = yawDiff(last_yaw, requested_yaw);
            if (amplitude > px4_orientation_confusion_limit)
            {
                last_yaw = px4_orientation_confusion_limit;
            }   
            else if(amplitude < -px4_orientation_confusion_limit)
            {
                last_yaw = -px4_orientation_confusion_limit;
            }   
            else
            {
                last_yaw = requested_yaw;
            }
            update_yaw(last_yaw);
            target_point = Eigen::Vector3f(target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
            update_target_fix_variables(target_pose.pose);
            res.is_going_to_position = true;
        }else{
            res.is_going_to_position = false;
        }
        new_target = true;
        new_orientation = true;
    }else{
        res.is_going_to_position = false;
    }
    return true;
}

geometry_msgs::TwistStamped calculateVelocity(Eigen::Vector3f x0, Eigen::Vector3f x2, double d) {
    double cruising_speed = 1.0;
    geometry_msgs::TwistStamped output_vel;
    Eigen::Vector3f unit_vec = (x2 - x0) / d;
    unit_vec = unit_vec / unit_vec.norm();
    output_vel.twist.linear.x = unit_vec(0) * cruising_speed;
    output_vel.twist.linear.y = unit_vec(1) * cruising_speed;
    output_vel.twist.linear.z = unit_vec(2) * cruising_speed;
    output_vel.header.frame_id = "uav_1_home";
    return output_vel;
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
            ROS_WARN("[UAL Node] Current position middle man node not accepting requests.");
            return false;
        }
    }

void initialization(ros::NodeHandle nh)
{   
        
    px4_orientation_confusion_limit_degrees = 120;
    px4_orientation_confusion_limit = (px4_orientation_confusion_limit_degrees*(M_PI/180));
    double px4_loiter_radius, odometry_error;
    nh.getParam("/px4_loiter_radius", px4_loiter_radius);
    nh.getParam("/odometry_error", odometry_error);
    position_tolerance = std::max(px4_loiter_radius, odometry_error);
    distance_switch_wp_control = 2;
    max_acceptance_orientation = 3.0;
    min_acceptance_orientation = 0.14;
    ros::param::param<int>("~uav_id", uav_id, 1);
    ros::param::param<bool>("~offboard_enabled", offboard_enabled, false);
    ROS_WARN_STREAM("[UAL Node] offboard_enabled: " << offboard_enabled);


    flight_level = 5.0;
    if (offboard_enabled) {
        ROS_INFO("[UAL Node] Take off height: %f", flight_level);
        movement_state = take_off;
    }

    velocity_to_pub.header.frame_id = "uav_1_home";
    uav_current_path.header.frame_id = "uav_1_home";
    uav_target_path.header.frame_id = "uav_1_home";

    geometry_msgs::Point current_position;
    while(!getUavPositionServiceCall(current_position));

    current_pose.pose.position = current_position;
    current_pose.pose.position.z = flight_level;
    current_pose.pose.orientation.x = 0;
    current_pose.pose.orientation.y = 0;
    current_pose.pose.orientation.z = 0;
    current_pose.pose.orientation.w = 1;
    uav_target_path.poses.push_back(current_pose);
}

void main_loop(grvc::ual::UAL& ual)
{
    while (ros::ok()) {
        update_current_variables(ual.pose());
        switch (movement_state) {
            case take_off:
                switch (uav_state.state) {
                    case 2:  // Landed armed
                        if (!taking_off) {
                            ROS_INFO("[UAL Node] Taking off ");
                            ual.takeOff(flight_level, false);
                            taking_off = true;
                        }
                        break;
                    case 4:  // Flying auto
                        ual.goToWaypoint(uav_target_path.poses.at(0), false);
                        break;
                }
                break;
            case hover:
                if (new_target) {
                    ual.goToWaypoint(target_pose, false);
                    new_target = false;
                }
                break;
            case velocity:
                if ((target_point - current_point).norm() > distance_switch_wp_control && new_target) {
                    velocity_to_pub = calculateVelocity(current_point, target_point, (target_point - current_point).norm());
                    ual.setVelocity(velocity_to_pub);
                } else {
                    ROS_INFO("[UAL Node] Hovering");
                    movement_state = hover;
                }
                break;
            case fix_pose:
                bool changedOrientation = (max_acceptance_orientation > q_current.angularDistance(q_target) && q_current.angularDistance(q_target) > min_acceptance_orientation);
                if (changedOrientation) {
                    // Eigen::Quaterniond q(fix_pose_pose.pose.orientation.w, fix_pose_pose.pose.orientation.x, fix_pose_pose.pose.orientation.y, fix_pose_pose.pose.orientation.z);
                    // auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
                    // ROS_INFO_STREAM( "[Ual Node] fix pose Yaw " << euler[2] << " = " << (euler[2]*(180/M_PI)));
                    ual.goToWaypoint(fix_pose_pose, false);
                    new_orientation = false;
                } 
                else 
                {
                    if(last_yaw != requested_yaw)
                    {
                        update_yaw(requested_yaw);
                        last_yaw = requested_yaw;
                    }

                    bool command_in_position = (max_acceptance_orientation > q_current.angularDistance(q_target) && q_current.angularDistance(q_target) > min_acceptance_orientation) 
                        || (fix_pose_point - current_point).norm() > position_tolerance;
                    if (command_in_position)
                    {
                        // ROS_INFO("[UAL Node] Waiting until position arrival.");
                    }
                    else 
                    {
                        ROS_INFO("[UAL Node] Arrived, start to command in velocity.");
                        movement_state = velocity;
                    }
                }
                break;
        }
        uav_current_path.poses.push_back(ual.pose());
        pub_current_path.publish(uav_current_path);
        pub_target_path.publish(uav_target_path);
        ros::spinOnce();
        sleep(0.1);
    }
}

}

int main(int _argc, char **_argv) {
    grvc::ual::UAL ual(_argc, _argv);

    ros::NodeHandle nh;
    ros::Subscriber sub_state = nh.subscribe<uav_abstraction_layer::State>("/uav_1/ual/state", 10, ual_server::state_cb);
    ros::ServiceServer target_position_service = nh.advertiseService("/target_position", ual_server::target_position_cb);
    ual_server::pub_current_path = nh.advertise<nav_msgs::Path>("/ual/current_path", 10);
    ual_server::pub_target_path = nh.advertise<nav_msgs::Path>("/ual/target_path", 10);

    ual_server::current_position_client = nh.serviceClient<architecture_msgs::PositionMiddleMan>("/get_current_position");


    ual_server::initialization(nh);

    while (!ual.isReady() && ros::ok()) {
        ROS_WARN("UAL %d not ready!", ual_server::uav_id);
        sleep(1);
    }
    ROS_INFO("UAL %d ready!", ual_server::uav_id);
    

    ual_server::main_loop(ual);

    return 0;
}

