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

nav_msgs::Path uav_current_path, uav_target_path;
Eigen::Vector3f target_point, current_point, fix_pose_point;
geometry_msgs::PoseStamped target_pose, current_pose, fix_pose_pose;
Eigen::Quaterniond q_current, q_target, q_target_fix;
uav_abstraction_layer::State uav_state;
bool new_target = false;
bool taking_off = false;
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
    q_target_fix.x() = fix_pose_pose.pose.orientation.x;
    q_target_fix.y() = fix_pose_pose.pose.orientation.y;
    q_target_fix.z() = fix_pose_pose.pose.orientation.z;
    q_target_fix.w() = fix_pose_pose.pose.orientation.w;
    return;
}

bool target_position_cb(architecture_msgs::PositionRequest::Request &req,
                        architecture_msgs::PositionRequest::Response &res) {
    if (uav_state.state == 4) {
        if (!new_target) {
            fix_pose_pose.pose.orientation = req.pose.orientation;
            fix_pose_pose.pose.position = target_pose.pose.position;
            movement_state = fix_pose;
            target_pose.pose = req.pose;
            target_pose.header.frame_id = "uav_1_home";
            uav_target_path.poses.push_back(target_pose);
            target_point = Eigen::Vector3f(target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
            q_target.x() = target_pose.pose.orientation.x;
            q_target.y() = target_pose.pose.orientation.y;
            q_target.z() = target_pose.pose.orientation.z;
            q_target.w() = target_pose.pose.orientation.w;
            update_current_variables(current_pose);
            ROS_INFO("[UAL] New target -> P: %f, %f, %f", target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
            ROS_INFO("                    O: %f, %f, %f, %f", target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w);
            ROS_INFO("[UAL] Fixing Pose");
            update_target_fix_variables(target_pose.pose);
            res.is_going_to_position = true;
        }else{
            res.is_going_to_position = false;
        }
        new_target = true;
    }else{
        res.is_going_to_position = false;
    }
    return true;
}

geometry_msgs::TwistStamped calculateVelocity(Eigen::Vector3f x0, Eigen::Vector3f x2, double d) {
    double cruising_speed = 1.0;
    geometry_msgs::TwistStamped output_vel;
    Eigen::Vector3f unit_vec = (x2 - x0) / d;
    output_vel.twist.linear.x = unit_vec(0) * cruising_speed;
    output_vel.twist.linear.y = unit_vec(1) * cruising_speed;
    output_vel.twist.linear.z = unit_vec(2) * cruising_speed;
    output_vel.header.frame_id = "uav_1_home";
    return output_vel;
}

int main(int _argc, char **_argv) {
    grvc::ual::UAL ual(_argc, _argv);

    ros::NodeHandle nh;
    ros::Subscriber sub_state = nh.subscribe<uav_abstraction_layer::State>("/uav_1/ual/state", 10, state_cb);
    ros::Publisher pub_current_path = nh.advertise<nav_msgs::Path>("/ual/current_path", 10);
    ros::Publisher pub_target_path = nh.advertise<nav_msgs::Path>("/ual/target_path", 10);
    ros::ServiceServer target_position_service = nh.advertiseService("/target_position", target_position_cb);

    double position_tolerance = 0.2;
    double distance_switch_wp_control = 0.5;
    double max_acceptance_orientation = 3.0;
    double min_acceptance_orientation = 0.14;
    int uav_id;
    bool offboard_enabled;
    ros::param::param<int>("~uav_id", uav_id, 1);
    ros::param::param<bool>("~offboard_enabled", offboard_enabled, false);
    ROS_WARN_STREAM("[UAL] offboard_enabled: " << offboard_enabled);

    while (!ual.isReady() && ros::ok()) {
        ROS_WARN("UAL %d not ready!", uav_id);
        sleep(1);
    }
    ROS_INFO("UAL %d ready!", uav_id);

    double flight_level = 5.0;
    if (offboard_enabled) {
        ROS_INFO("[UAL] Take off height: %f", flight_level);
        movement_state = take_off;
    }

    geometry_msgs::TwistStamped velocity_to_pub;
    velocity_to_pub.header.frame_id = "uav_1_home";
    uav_current_path.header.frame_id = "uav_1_home";
    uav_target_path.header.frame_id = "uav_1_home";
    current_pose.pose.position.x = 0;
    current_pose.pose.position.y = 0;
    current_pose.pose.position.z = flight_level;
    current_pose.pose.orientation.x = 0;
    current_pose.pose.orientation.y = 0;
    current_pose.pose.orientation.z = 0;
    current_pose.pose.orientation.w = 1;
    uav_target_path.poses.push_back(current_pose);

    while (ros::ok()) {
        update_current_variables(ual.pose());
        switch (movement_state) {
            case take_off:
                switch (uav_state.state) {
                    case 2:  // Landed armed
                        if (!taking_off) {
                            ROS_INFO("[UAL] Taking off ");
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
                    ROS_INFO("[UAL] Hovering");
                    movement_state = hover;
                }
                break;
            case fix_pose:
                update_target_fix_variables(fix_pose_pose.pose);
                if ((max_acceptance_orientation > q_current.angularDistance(q_target) && q_current.angularDistance(q_target) > min_acceptance_orientation) || (fix_pose_point - current_point).norm() > position_tolerance) {
                    ual.goToWaypoint(fix_pose_pose, false);
                } else {
                    ROS_INFO("[UAL] Going to target");
                    movement_state = velocity;
                }
                break;
        }
        uav_current_path.poses.push_back(ual.pose());
        pub_current_path.publish(uav_current_path);
        pub_target_path.publish(uav_target_path);
        ros::spinOnce();
        sleep(0.1);
    }

    return 0;
}