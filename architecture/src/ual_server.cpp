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
Eigen::Vector3f target_point, current_point, fix_orientation_point;
geometry_msgs::PoseStamped target_pose, current_pose, fix_orientation_pose;
Eigen::Quaterniond q_current, q_target, q_target_fix;
uav_abstraction_layer::State uav_state;
bool new_target = false;
int cont_init_d_to_target = 0;
float init_d_to_target = 0;
int cont_smooth_vel = 1;
int max_velocity_portions = 50;
enum movement_state_t {hover, velocity, fix_orientation};
movement_state_t movement_state;

void update_current_variables(geometry_msgs::PoseStamped ual_pose){
    current_pose = ual_pose;
    current_point = Eigen::Vector3f(current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
    q_current.x() = current_pose.pose.orientation.x;
    q_current.y() = current_pose.pose.orientation.y;
    q_current.z() = current_pose.pose.orientation.z;
    q_current.w() = current_pose.pose.orientation.w;
    return;
}

void update_target_fix_variables(geometry_msgs::Pose fix_orientation){
    // fix_orientation_pose = uav_target_path.poses.back(); 
    fix_orientation_pose.pose.orientation = fix_orientation.orientation;
    fix_orientation_pose.pose.position = uav_target_path.poses.at(uav_target_path.poses.size()-1).pose.position; // Take previous target_pose.position to fix orientation before next target
    fix_orientation_point = Eigen::Vector3f(fix_orientation_pose.pose.position.x, fix_orientation_pose.pose.position.y, fix_orientation_pose.pose.position.z);
    q_target_fix.x() = fix_orientation_pose.pose.orientation.x;
    q_target_fix.y() = fix_orientation_pose.pose.orientation.y;
    q_target_fix.z() = fix_orientation_pose.pose.orientation.z;
    q_target_fix.w() = fix_orientation_pose.pose.orientation.w;
    std::cout << "q_target_fix: " << q_target_fix.x() << " " << q_target_fix.y() << " " << q_target_fix.z() << " " << q_target_fix.w() << std::endl;
    return;
}

bool target_position_cb(architecture_msgs::PositionRequest::Request &req,
                        architecture_msgs::PositionRequest::Response &res) {
    if (!new_target) {
        target_pose.pose = req.pose;
        // std::cout << "UAV path: " << std::endl;
        // for (int i = 0; i < uav_target_path.poses.size(); i++){
        //     std::cout << uav_target_path.poses.at(i).pose << std::endl;
        // }
        // std::cout << "----------" << std::endl;
        target_point = Eigen::Vector3f(target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
        // Eigen::Quaterniond q_target(target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w); 
        q_target.x() = target_pose.pose.orientation.x;
        q_target.y() = target_pose.pose.orientation.y;
        q_target.z() = target_pose.pose.orientation.z;
        q_target.w() = target_pose.pose.orientation.w;
        update_current_variables(current_pose);
        std::cout << "o_target  -> x: " << target_pose.pose.orientation.x << ", y: " << target_pose.pose.orientation.y << ", z: " << target_pose.pose.orientation.z << ", w: " << target_pose.pose.orientation.w << std::endl; 
        std::cout << "q_target  -> x: " << q_target.x() << ", y: " << q_target.y() << ", z: " <<  q_target.z() << ", w: " << q_target.w() << std::endl;
        std::cout << "q_current -> x: " << q_current.x() << ", y: " << q_current.y() << ", z: " <<  q_current.z() << ", w: " << q_current.w() << std::endl;
        std::cout << "Angular distance  1: " << q_current.angularDistance(q_target) << std::endl;
        Eigen::Quaterniond q_result, q_result2;
        q_result  = q_target*q_current.inverse();
        q_result2 = q_current*q_target.inverse();
        std::cout << "q_result   -> x: " << q_result.x() << ", " << q_result.y() << ", " <<  q_result.z() << ", " << q_result.w() << std::endl;
        std::cout << "q_result2  -> x: " << q_result2.x() << ", " << q_result2.y() << ", " <<  q_result2.z() << ", " << q_result2.w() << std::endl;
        // std::cout << "normalized : " << q_result.norm() << std::endl;
        // std::cout << "normalized2: " << q_result2.norm() << std::endl;
        // std::cout << "Angular distance -2: " <<  << std::endl;
        // std::cout << "Angular distance  2: " <<  << std::endl;
        if (q_current.angularDistance(q_target) > 0){
            update_target_fix_variables(target_pose.pose);
            movement_state = fix_orientation;
            // std::cout << target_pose.pose.orientation << std::endl << fix_orientation_pose.pose.orientation << std::endl;
        }else{
            movement_state = velocity;
        }
        uav_target_path.poses.push_back(target_pose);
    }
    new_target = true;
    return true;
}

void state_cb(const uav_abstraction_layer::State msg) {
    uav_state = msg;
    return;
}


geometry_msgs::TwistStamped calculateSmoothVelocity(Eigen::Vector3f x0, Eigen::Vector3f x2, double d) {
    double cruising_speed = 1.0;
    geometry_msgs::TwistStamped output_vel;

    Eigen::Vector3f unit_vec = (x2 - x0) / d;
    double velocity_portion = cruising_speed / max_velocity_portions;

    if (cont_init_d_to_target == 0) {
        init_d_to_target = d;
        cont_init_d_to_target++;
    }

    output_vel.twist.linear.x = unit_vec(0) * velocity_portion * cont_smooth_vel;
    output_vel.twist.linear.y = unit_vec(1) * velocity_portion * cont_smooth_vel;
    output_vel.twist.linear.z = unit_vec(2) * velocity_portion * cont_smooth_vel;

    if (d >= 3) {
        if (cont_smooth_vel < max_velocity_portions) cont_smooth_vel++;
    } else {
        if (cont_smooth_vel > 0) cont_smooth_vel--;
    }

    if (d < 1) {
        cont_smooth_vel = 0;
        init_d_to_target = 0;
        cont_init_d_to_target = 0;
    }

    output_vel.header.frame_id = "uav_1_home";
    return output_vel;
}

geometry_msgs::TwistStamped calculateVelocity(Eigen::Vector3f x0, Eigen::Vector3f x2, double d) {
    double cruising_speed = 1.0;
    geometry_msgs::TwistStamped output_vel;

    Eigen::Vector3f unit_vec = (x2 - x0) / d;

    if (d >= 1.0) {
        output_vel.twist.linear.x = unit_vec(0) * cruising_speed;
        output_vel.twist.linear.y = unit_vec(1) * cruising_speed;
        output_vel.twist.linear.z = unit_vec(2) * cruising_speed;
    } else {
        output_vel.twist.linear.x = 0;
        output_vel.twist.linear.y = 0;
        output_vel.twist.linear.z = 0;
    }
    output_vel.header.frame_id = "uav_1_home";
    return output_vel;
}

int main(int _argc, char **_argv) {
    grvc::ual::UAL ual(_argc, _argv);

    ros::NodeHandle nh;
    ros::Subscriber sub_state = nh.subscribe<uav_abstraction_layer::State>("/uav_1/ual/State", 10, state_cb);
    ros::Publisher pub_current_path = nh.advertise<nav_msgs::Path>("/ual/current_path", 10);
    ros::Publisher pub_target_path = nh.advertise<nav_msgs::Path>("/ual/target_path", 10);
    ros::ServiceServer target_position_service = nh.advertiseService("/uav_1/ual/target_position", target_position_cb);

    int uav_id;
    ros::param::param<int>("~uav_id", uav_id, 1);

    while (!ual.isReady() && ros::ok()) {
        ROS_WARN("UAL %d not ready!", uav_id);
        sleep(1);
    }
    ROS_INFO("UAL %d ready!", uav_id);
    double flight_level = 5.0;
    ual.takeOff(flight_level);

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

    update_current_variables(ual.pose());

    while (ros::ok()) {
        update_current_variables(ual.pose());
        double d_to_target = (target_point - current_point).norm();
        std::cout << "Movement state: [";
        switch(movement_state) {
            case hover:
                std::cout << "Hover]" << std::endl;
                if (new_target) {
                    ual.goToWaypoint(target_pose, false);
                }
                new_target = false;
                break;
            case velocity:
                std::cout << "Velocity]" << std::endl;
                while (d_to_target > 0.8 && new_target) {
                    velocity_to_pub = calculateVelocity(current_point, target_point, d_to_target);
                    ual.setVelocity(velocity_to_pub);
                    uav_current_path.poses.push_back(ual.pose());
                    pub_current_path.publish(uav_current_path);
                    update_current_variables(ual.pose());
                    d_to_target = (target_point - current_point).norm();
                    pub_target_path.publish(uav_target_path);
                    sleep(0.1);
                }
                movement_state = hover;
                break;
            case fix_orientation:
            std::cout << "Fix orientation]" << std::endl;
                update_current_variables(ual.pose());
                update_target_fix_variables(fix_orientation_pose.pose);
                std::cout << "[0] Angular: " << q_current.angularDistance(q_target_fix) << " Distance: " << (fix_orientation_point - current_point).norm() << std::endl;
                while (q_current.angularDistance(q_target_fix) > 0.2 || (fix_orientation_point - current_point).norm() > 0.2){
                    ual.goToWaypoint(fix_orientation_pose, false);
                    update_current_variables(ual.pose());
                    update_target_fix_variables(fix_orientation_pose.pose);
                    std::cout << "[1]  Angular: " << q_current.angularDistance(q_target_fix) << " Distance: " << (fix_orientation_point - current_point).norm() << std::endl;
                    std::cout << "[1]  Angular: " << q_target_fix.angularDistance(q_current) << " Distance: " << (fix_orientation_point - current_point).norm() << std::endl;
                    sleep(1);
                }
                std::cout << "[2] Angular: " << q_current.angularDistance(q_target_fix) << " Distance: " << (fix_orientation_point - current_point).norm() << std::endl;
                movement_state = velocity;
                break;
        }
        uav_current_path.poses.push_back(ual.pose());
        pub_current_path.publish(uav_current_path);
        sleep(1);
    }

    return 0;
}