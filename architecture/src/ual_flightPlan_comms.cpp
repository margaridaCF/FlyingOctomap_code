//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2019 Margarida Faria
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
    sub_flight_plan_ = nh_.subscribe("/uav_" + std::to_string(uav_id_) + "/flight_plan_requests", 0, &UALCommunication::flightPlanCallback, this);
    flight_plan_state_ = nh_.advertise<std_msgs::Empty>("/uav_" + std::to_string(uav_id_) + "/flight_plan_notifications", 1000);
    flight_plan = csvToPath(init_path_name_);
    flight_plan.header.frame_id = "uav_" + std::to_string(uav_id_) + "_home";
    current_target = 1;
    switchState(init_segment);
    // =======================




    // Subscriptions
    sub_pose_ = nh_.subscribe("/uav_" + std::to_string(uav_id_) + "/ual/pose", 0, &UALCommunication::ualPoseCallback, this);
    sub_state_ = nh_.subscribe("/uav_" + std::to_string(uav_id_) + "/ual/state", 0, &UALCommunication::ualStateCallback, this);
    sub_velocity_ = nh_.subscribe("/upat_follower/follower/uav_" + std::to_string(uav_id_) + "/output_vel", 0, &UALCommunication::velocityCallback, this);
    // Publishers
    pub_set_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/uav_" + std::to_string(uav_id_) + "/ual/set_pose", 1000);
    pub_set_velocity_ = nh_.advertise<geometry_msgs::TwistStamped>("/uav_" + std::to_string(uav_id_) + "/ual/set_velocity", 1000);
    pub_flight_plan_ = nh_.advertise<nav_msgs::Path>("/uav_" + std::to_string(uav_id_) + "/flight_plan", 1);
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
            ROS_WARN("[UAL COMMS] wait_for_flight");
            break;
        case init_segment:
            ROS_WARN("[UAL COMMS] init_segment");
            break;
        case execute_position:
            ROS_WARN("[UAL COMMS] execute_position");
            break;
    }
}

nav_msgs::Path UALCommunication::csvToPath(std::string _file_name) 
{
    nav_msgs::Path out_path;
    out_path.header.frame_id = "uav_" + std::to_string(uav_id_) + "_home";
    std::string pkg_name_path = ros::package::getPath(pkg_name_);
    std::string folder_name = pkg_name_path + "/maneuvers/" + _file_name + ".csv";
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
    std::vector<geometry_msgs::PoseStamped> poses(waypoint_amount);
    if (read_csv.is_open()) {
        char comma;
        double yaw;
        tf2::Quaternion q_yaw;
        for (int i = 0; i < waypoint_amount; ++i)
        {
            read_csv >> poses.at(i).pose.position.x  >> comma >> poses.at(i).pose.position.y >> comma >> poses.at(i).pose.position.z;
        }
    }
    else ROS_ERROR_STREAM("read_csv is someshow closed...");
    out_path.poses = poses;
    return out_path;
}

bool generateYaw(nav_msgs::Path & path)
{
    tf2::Quaternion q_yaw;
    double yaw;
    for (int i = 0; i < path.poses.size(); ++i)
    {
        if(i > 0)
        {
            yaw = architecture_math::calculateOrientation(Eigen::Vector2d(path.poses.at(i-1).pose.position.x, path.poses.at(i-1).pose.position.y), Eigen::Vector2d(path.poses.at(i).pose.position.x, path.poses.at(i).pose.position.y)) ;
            q_yaw.setRPY( 0, 0, yaw );
            q_yaw.normalize();
            path.poses.at(i-1).pose.orientation = tf2::toMsg(q_yaw);
        }
    }
    if(path.poses.size() > 1)
    {
        path.poses.at(path.poses.size()-1).pose.orientation = tf2::toMsg(q_yaw);
    }
    else if(path.poses.size() == 1)
    {
        ROS_WARN("[UAL COMMS] The received path with one waypoint. The yaw is zero because to face forwards two points are needed to create a line to align with.");
        q_yaw.setRPY( 0, 0, 0 );
        path.poses.at(0).pose.orientation = tf2::toMsg(q_yaw);
    }
    else
    {
        ROS_ERROR_STREAM("[UAL COMMS] The received path has no waypoints.");
        return false;
    }
    return true;
}

void UALCommunication::flightPlanCallback(const nav_msgs::Path::ConstPtr &_flight_plan) {
    flight_plan = *_flight_plan;
    flight_plan.header.frame_id = "uav_" + std::to_string(uav_id_) + "_home";
    current_target = 1;
    switchState(init_segment);
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
    pub_flight_plan_.publish(flight_plan);
}

bool UALCommunication::prepare()
{
    // Add current position to flight plan
    std::vector<geometry_msgs::PoseStamped>::iterator it;
    geometry_msgs::PoseStamped current_pose;
    current_pose.pose.position = ual_pose_.pose.position;
    it = init_path_.poses.begin();
    it = init_path_.poses.insert ( it , current_pose );
    // Initialize path
    if( !generateYaw(init_path_) ) return false;
    // Save data
    if (save_test_) {
        std::string pkg_name_path = ros::package::getPath(pkg_name_);
        folder_data_name_ = pkg_name_path + "/tests/splines";
    }

    // Flags
    on_path_ = false;
    end_path_ = false;
    double look_ahead = 0.8;
    double cruising_speed = 1.0;
    target_path_ = follower_.preparePath(init_path_, generator_mode_, look_ahead, cruising_speed);
    // ROS_ERROR_STREAM("init_path_" );
    // for (std::vector<geometry_msgs::PoseStamped>::iterator i = init_path_.poses.begin(); i != init_path_.poses.end(); ++i)
    // {
    //     ROS_ERROR_STREAM(i->pose.position.x << "," << i->pose.position.y << "," << i->pose.position.z << "," << i->pose.orientation.x  << "," << i->pose.orientation.y  << "," << i->pose.orientation.z  << "," << i->pose.orientation.w );
    // }
    return true;
}

bool UALCommunication::prepare_yaw()
{
    nav_msgs::Path segment;
    init_path_.header.frame_id = "uav_" + std::to_string(uav_id_) + "_home";
    init_path_.poses = {flight_plan.poses.at(current_target-1), flight_plan.poses.at(current_target)};
    if( !generateYaw(init_path_) ) return false;
    temp_segment_path = init_path_;
    return true;
}

bool UALCommunication::prepare_position()
{
    nav_msgs::Path segment;
    init_path_.header.frame_id = "uav_" + std::to_string(uav_id_) + "_home";
    init_path_.poses = {flight_plan.poses.at(current_target-1), flight_plan.poses.at(current_target)};
    // Flags
    on_path_ = false;
    end_path_ = false;
    double look_ahead = 0.8;
    double cruising_speed = 1.0;
    target_path_ = follower_.preparePath(init_path_, generator_mode_, look_ahead, cruising_speed);
    return true;
}

double calculateYawRate(double current_yaw, double desired_yaw)
{
    double yaw_rate_dt = 2;
    double dx = desired_yaw - current_yaw;
    if      ( dx > M_PI )  { dx =  -(dx - M_PI);}
    else if ( dx < -M_PI ) { dx = -(dx + M_PI); }
    double yaw_rate = dx / yaw_rate_dt; 
    // ROS_WARN_STREAM("");
    // ROS_WARN_STREAM("[UAL COMMS] std::min(" << yaw_rate << ", 0.5) = " << std::min(yaw_rate, 0.5));
    yaw_rate = std::min(yaw_rate, 0.5);
    // ROS_WARN_STREAM("[UAL COMMS] std::max(" << yaw_rate << ", -0.5) = " << std::max(yaw_rate, -0.5));
    yaw_rate = std::max(yaw_rate, -0.5);
    // ROS_WARN_STREAM("[UAL COMMS] returning " << yaw_rate);
    return yaw_rate;

}

double UALCommunication::checkYaw()
{
    double desired_yaw = tf::getYaw(temp_segment_path.poses.at(1).pose.orientation);
    double current_yaw = tf::getYaw(ual_pose_.pose.orientation);
    velocity_.twist.angular.z = calculateYawRate(current_yaw, desired_yaw);
    return velocity_.twist.angular.z;
}

void UALCommunication::runFlightPlan_segments()
{
    switch(comms_state)
    {
        case init_segment:
            if(prepare_yaw())            desired_quaternion = temp_segment_path.poses.at(1).pose.orientation;
            if(prepare_position())                          switchState(execute_position);
            else                                            switchState(wait_for_flight);
            break;
        case execute_position:
            followFlightPlan();
            if(end_path_)
            {
                current_target++;
                if(current_target < flight_plan.poses.size())  switchState(init_segment);
                else
                {                                            
                                                                switchState(wait_for_flight);
                    std_msgs::Empty done;
                    flight_plan_state_.publish(done);
                }
            }
            break;
        case wait_for_flight: break;
        default:
            ROS_WARN_STREAM("[UAL COMMS] Something went very wrong. comms_state = " << comms_state);
            break;
    }
}

void UALCommunication::runMission_try2() {
    // ROS_WARN_STREAM("ual_state_.state = " << ual_state_);
    switch (ual_state_.state) {
        case 2:  // Landed armed
            // if (!end_path_) 
            {
                uav_abstraction_layer::TakeOff take_off;
                take_off.request.height = take_off_height;
                take_off.request.blocking = true;
                client_take_off_.call(take_off);
            }
            break;
        case 3:  // Taking of
            break;
        case 4:  // Flying auto
            runFlightPlan_segments();
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
            geometry_msgs::PoseStamped temp_target;
            temp_target = target_path_.poses.at(0);
            temp_target.pose.orientation = desired_quaternion;
            if ((current_p - path0_p).norm() > reach_tolerance_ * 2) {
                pub_set_pose_.publish(temp_target);
            } else if (reach_tolerance_ > (current_p - path0_p).norm() && !flag_hover_ && std::abs(checkYaw()) < 0.01) {
                pub_set_pose_.publish(temp_target);
                on_path_ = true;
            } else {
                pub_set_pose_.publish(temp_target);
            }
        } else {
            if (reach_tolerance_ * 2 > (current_p - path_end_p).norm()) {
                pub_set_pose_.publish(target_path_.poses.back());
                on_path_ = false;
                end_path_ = true;
            } else {
                follower_.updatePose(ual_pose_);
                double current_yaw = tf::getYaw(ual_pose_.pose.orientation);
                velocity_ = follower_.getVelocity();
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