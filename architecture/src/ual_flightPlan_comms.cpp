//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 Hector Perez Leon
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

#include <ual_flightPlan_comms.h>

namespace flight_plan_comms {

UALCommunication::UALCommunication() : nh_(), pnh_("~") {
    // Parameters
    pnh_.getParam("uav_id", uav_id_);
    pnh_.getParam("initial_maneuver", init_path_name_);
    pnh_.getParam("pkg_name", pkg_name_);
    pnh_.getParam("reach_tolerance", reach_tolerance_);
    pnh_.getParam("generator_mode", generator_mode_);
    pnh_.getParam("take_off_height", take_off_height);

    // === Final data flow ===
    follower_ = upat_follower::Follower(uav_id_);
    sub_flight_plan_ = nh_.subscribe("/uav_" + std::to_string(uav_id_) + "/flight_plan", 0, &UALCommunication::flightPlanCallback, this);
    switchState(init_flight);
    // =======================




    // Subscriptions
    sub_pose_ = nh_.subscribe("/uav_" + std::to_string(uav_id_) + "/ual/pose", 0, &UALCommunication::ualPoseCallback, this);
    sub_state_ = nh_.subscribe("/uav_" + std::to_string(uav_id_) + "/ual/state", 0, &UALCommunication::ualStateCallback, this);
    sub_velocity_ = nh_.subscribe("/upat_follower/follower/uav_" + std::to_string(uav_id_) + "/output_vel", 0, &UALCommunication::velocityCallback, this);
    // Publishers
    pub_set_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/uav_" + std::to_string(uav_id_) + "/ual/set_pose", 1000);
    pub_set_velocity_ = nh_.advertise<geometry_msgs::TwistStamped>("/uav_" + std::to_string(uav_id_) + "/ual/set_velocity", 1000);
    // Services
    client_take_off_ = nh_.serviceClient<uav_abstraction_layer::TakeOff>("/uav_" + std::to_string(uav_id_) + "/ual/take_off");
    client_land_ = nh_.serviceClient<uav_abstraction_layer::Land>("/uav_" + std::to_string(uav_id_) + "/ual/land");
    client_visualize_ = nh_.serviceClient<upat_follower::Visualize>("/upat_follower/visualization/uav_" + std::to_string(uav_id_) + "/visualize");
    }

UALCommunication::~UALCommunication() {
}

void UALCommunication::switchState(comms_state_t new_comms_state)
{
    comms_state = new_comms_state;
    switch(comms_state)
    {
        case wait_for_flight:
            ROS_INFO("[UAL COMMS] wait_for_flight");
            break;
        case init_flight:
            ROS_INFO("[UAL COMMS] init_flight");
            break;
        case execute_flight:
            ROS_INFO("[UAL COMMS] execute_flight");
            break;
    }
}

nav_msgs::Path UALCommunication::csvToPath(std::string _file_name) 
{
    nav_msgs::Path out_path;
    out_path.header.frame_id = "uav_" + std::to_string(uav_id_) + "_home";
    std::string pkg_name_path = ros::package::getPath(pkg_name_);
    std::string folder_name = pkg_name_path + "/data/" + _file_name + ".csv";
    std::fstream read_csv;
    read_csv.open(folder_name);
    int waypoint_amount;
    read_csv >> waypoint_amount;
    if (read_csv.fail() || read_csv.eof()) 
    {
        ROS_ERROR_STREAM("[UAL COMMS] Could not read file " << folder_name);
        switchState(wait_for_flight);
        return out_path;
    }
    ROS_INFO_STREAM("[UAL COMMS] Initial maneuver has " << waypoint_amount << " points");
    read_csv.ignore(100, '\n');
    std::vector<geometry_msgs::PoseStamped> poses(waypoint_amount);
    ROS_INFO("[UAL COMMS] Reading initial_maneuver");
    if (read_csv.is_open()) {
        char comma;
        for (int i = 0; i < waypoint_amount; ++i)
        {
            read_csv >> poses.at(i).pose.position.x  >> comma >> poses.at(i).pose.position.y >> comma >> poses.at(i).pose.position.z;
            poses.at(i).pose.orientation.x = 0;
            poses.at(i).pose.orientation.y = 0;
            poses.at(i).pose.orientation.z = 0;
            poses.at(i).pose.orientation.w = 1;
            // ROS_INFO_STREAM("(" << poses.at(i).x << ", " << poses.at(i).y ", "<<poses.at(i).z<<")");
            ROS_INFO_STREAM("[UAL COMMS] [" << i << "] " << poses.at(i).pose.position);
        }
    }
    else
    {
        ROS_ERROR_STREAM("read_csv is someshow closed...");
    }
    out_path.poses = poses;
    return out_path;
}

void UALCommunication::flightPlanCallback(const nav_msgs::Path::ConstPtr &_flight_plan) {
    init_path_ = *_flight_plan;
    switchState(init_flight);
}

void UALCommunication::ualPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &_ual_pose) {
    ual_pose_ = *_ual_pose;
}

void UALCommunication::ualStateCallback(const uav_abstraction_layer::State &_ual_state) {
    ual_state_.state = _ual_state.state;
}

void UALCommunication::velocityCallback(const geometry_msgs::TwistStamped &_velocity) {
    velocity_ = _velocity;
}


void UALCommunication::callVisualization() {
    upat_follower::Visualize visualize;
    visualize.request.init_path = init_path_;
    visualize.request.generated_path = target_path_;
    visualize.request.current_path = current_path_;
    client_visualize_.call(visualize);
}

bool UALCommunication::prepare()
{
    // Initialize path
    init_path_ = csvToPath(init_path_name_);
    if(comms_state == wait_for_flight) return false;
    // Save data
    if (save_test_) {
        std::string pkg_name_path = ros::package::getPath(pkg_name_);
        folder_data_name_ = pkg_name_path + "/tests/splines";
    }

    // Flags
    on_path_ = false;
    end_path_ = false;
    upat_follower::PreparePath prepare_path;
    upat_follower::PrepareTrajectory prepare_trajectory;
    if (target_path_.poses.size() < 1) {
        prepare_path.request.init_path = init_path_;
        prepare_path.request.generator_mode.data = 2;
        prepare_path.request.look_ahead.data = 1.2;
        prepare_path.request.cruising_speed.data = 1.0;
        target_path_ = follower_.preparePath(init_path_, generator_mode_, 0.4, 1.0);
    }
    return true;
}

void UALCommunication::runFlightPlan()
{
    switch(comms_state)
    {
        case init_flight:
            if(prepare())   switchState(execute_flight);
            else            switchState(wait_for_flight);
            break;
        case execute_flight:
            followFlightPlan();
            if(end_path_)   switchState(wait_for_flight);
            break;
    }
}

void UALCommunication::runMission_try2() {
    switch (ual_state_.state) {
        case 2:  // Landed armed
            if (!end_path_) {
                uav_abstraction_layer::TakeOff take_off;
                take_off.request.height = take_off_height;
                take_off.request.blocking = true;
                client_take_off_.call(take_off);
            }
            break;
        case 3:  // Taking of
            break;
        case 4:  // Flying auto
            runFlightPlan();
            break;
        case 5:  // Landing
            break;
    }
}

void UALCommunication::followFlightPlan()
{
    Eigen::Vector3f current_p, path0_p, path_end_p;
    current_p = Eigen::Vector3f(ual_pose_.pose.position.x, ual_pose_.pose.position.y, ual_pose_.pose.position.z);
    path0_p = Eigen::Vector3f(target_path_.poses.front().pose.position.x, target_path_.poses.front().pose.position.y, target_path_.poses.front().pose.position.z);
    path_end_p = Eigen::Vector3f(target_path_.poses.back().pose.position.x, target_path_.poses.back().pose.position.y, target_path_.poses.back().pose.position.z);
    if (!end_path_) {
        if (!on_path_) {
            if ((current_p - path0_p).norm() > reach_tolerance_ * 2) {
                pub_set_pose_.publish(target_path_.poses.at(0));
            } else if (reach_tolerance_ > (current_p - path0_p).norm() && !flag_hover_) {
                pub_set_pose_.publish(target_path_.poses.front());
                on_path_ = true;
            }
        } else {
            if (reach_tolerance_ * 2 > (current_p - path_end_p).norm()) {
                pub_set_pose_.publish(target_path_.poses.back());
                on_path_ = false;
                end_path_ = true;
            } else {
                follower_.updatePose(ual_pose_);
                velocity_ = follower_.getVelocity();
                velocity_.twist.angular.z = 1;
                pub_set_velocity_.publish(velocity_);
                current_path_.header.frame_id = ual_pose_.header.frame_id;
                current_path_.poses.push_back(ual_pose_);
            }
        }
    } else {
        if (reach_tolerance_ * 2 > (current_p - path_end_p).norm() && (current_p - path_end_p).norm() > reach_tolerance_) {
            pub_set_pose_.publish(target_path_.poses.back());
        } else {
            uav_abstraction_layer::Land land;
            land.request.blocking = true;
            client_land_.call(land);
        }
    }
}

}  // namespace upat_follower