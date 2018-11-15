#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/ParamSet.h>
#include <architecture_msgs/PositionRequest.h>
#include <architecture_msgs/PositionMiddleMan.h>
#include <architecture_msgs/YawSpin.h>

namespace mav_comms
{
bool offboard_enabled;
ros::Duration exploration_maneuver_phases_duration_secs;
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
// ros::Publisher setpoint_raw_pub;
ros::Publisher local_pos_pub;
ros::Publisher local_velocity_pub;
ros::Publisher target_wp_pub;
ros::ServiceServer yaw_spin_service;
ros::ServiceServer target_position_service;
ros::ServiceServer target_position_vel_service;
ros::ServiceServer current_position_service;
enum movement_state_t
{
    take_off,
    hover,
    position,
    yaw_then_velocity,
    velocity,
    stop,
    yaw_spin
};
enum yaw_spin_maneuver_state_t
{
    front_facing = 0,
    yaw_1 = 120,
    yaw_2 = 240,
    yaw_3 = 360
};
struct Position
{
    movement_state_t movement_state;
    int waypoint_sequence;
    geometry_msgs::Pose pose;
    yaw_spin_maneuver_state_t yaw_spin_state;
    ros::Time yaw_spin_last_sent;
};
// mavros_msgs::PositionTarget stop_position;
geometry_msgs::PoseStamped target_orientation;
Position position_state, current_position;
ros::ServiceClient param_set_client;

bool flying = false;
bool flag_next_wp;
std::vector<double> poseListX, poseListY, poseListZ;

template <typename Real>
int nearestNeighbourIndex(std::vector<Real> &x, Real &value)
{
    Real dist = std::numeric_limits<Real>::max();
    Real newDist = dist;
    size_t idx = 0;

    for (size_t i = 0; i < x.size(); ++i)
    {
        newDist = std::abs(value - x[i]);
        if (newDist <= dist)
        {
            dist = newDist;
            idx = i;
        }
    }

    return idx;
}

template <typename Real>
std::vector<Real> interp1(std::vector<Real> &x, std::vector<Real> &y, std::vector<Real> &x_new)
{
    std::vector<Real> y_new;
    Real dx, dy, m, b;
    size_t x_max_idx = x.size() - 1;
    size_t x_new_size = x_new.size();

    y_new.reserve(x_new_size);

    for (size_t i = 0; i < x_new_size; ++i)
    {
        size_t idx = nearestNeighbourIndex(x, x_new[i]);

        if (x[idx] > x_new[i])
        {
            dx = idx > 0 ? (x[idx] - x[idx - 1]) : (x[idx + 1] - x[idx]);
            dy = idx > 0 ? (y[idx] - y[idx - 1]) : (y[idx + 1] - y[idx]);
        }
        else
        {
            dx = idx < x_max_idx ? (x[idx + 1] - x[idx]) : (x[idx] - x[idx - 1]);
            dy = idx < x_max_idx ? (y[idx + 1] - y[idx]) : (y[idx] - y[idx - 1]);
        }

        m = dy / dx;
        b = y[idx] - x[idx] * m;

        y_new.push_back(x_new[i] * m + b);
    }

    return y_new;
}

std::vector<double> interpolation(double target, double actual)
{
    double num_interp = 100;
    std::vector<double> x = {0.0, num_interp};
    std::vector<double> y = {actual, target};
    std::vector<double> newx;
    for (int i = 0; i < num_interp; i++)
    {
        newx.push_back(i);
    }
    auto res = interp1(x, y, newx);
    return res;
}

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pose = *msg;
}

bool set_mavros_param(std::string param_name, double param_value)
{
    mavros_msgs::ParamSet set_param_srv;
    set_param_srv.request.param_id = param_name;
    set_param_srv.request.value.integer = param_value;
    return param_set_client.call(set_param_srv);
}

bool yaw_spin_cb(architecture_msgs::YawSpin::Request &req,
                 architecture_msgs::YawSpin::Response &res)
{
    position_state.pose.position = req.position;
    position_state.movement_state = yaw_spin;
    // ROS_INFO_STREAM("[mav_comms] Receiving yaw spin request " << req);
    return true;
}

bool current_position_cb(architecture_msgs::PositionMiddleMan::Request &req,
                         architecture_msgs::PositionMiddleMan::Response &res)
{
    ROS_WARN_STREAM("[mav_comms] current_position_cb");
    current_position.pose.position.x = res.current_position.x;
    current_position.pose.position.y = res.current_position.y;
    current_position.pose.position.z = res.current_position.z;
    return true;
}

bool target_position_cb(architecture_msgs::PositionRequest::Request &req,
                        architecture_msgs::PositionRequest::Response &res)
{
    ROS_WARN_STREAM("[mav_comms] target_position_cb");
    if (position_state.movement_state != hover)
    {
        ROS_WARN_STREAM("[mav_comms] Rejecting position " << req.pose
                                                          << ", wrong movement state");
        res.is_going_to_position = false;
    }
    else if (req.waypoint_sequence_id < position_state.waypoint_sequence)
    {
        ROS_WARN_STREAM("[mav_comms] Rejecting position " << req.pose
                                                          << ", wrong movement state or request is from aborted waypoint sequence (id "
                                                          << req.waypoint_sequence_id << ")");
        res.is_going_to_position = false;
    }
    else
    {
        position_state.movement_state = position;
        position_state.waypoint_sequence = req.waypoint_sequence_id;
        position_state.pose = req.pose;
        res.is_going_to_position = true;
    }
    geometry_msgs::PoseStamped wp_to_pub;
    wp_to_pub.pose = position_state.pose;
    target_wp_pub.publish(wp_to_pub);
    return true;
}

bool target_position_vel_cb(architecture_msgs::PositionRequest::Request &req,
                            architecture_msgs::PositionRequest::Response &res)
{
    ROS_WARN_STREAM("[mav_comms] target_position_vel_cb");
    if (position_state.movement_state != hover)
    {
        ROS_WARN_STREAM("[mav_comms] Rejecting position " << req.pose
                                                          << ", wrong movement state");
        res.is_going_to_position = false;
    }
    else
    {
        if (target_orientation.pose.orientation.x != req.pose.orientation.x ||
            target_orientation.pose.orientation.y != req.pose.orientation.y ||
            target_orientation.pose.orientation.z != req.pose.orientation.z ||
            target_orientation.pose.orientation.w != req.pose.orientation.w)
        {
            target_orientation.pose = req.pose;
            target_orientation.pose.orientation = req.pose.orientation;
            target_orientation.pose.position = current_pose.pose.position;
            position_state.movement_state = yaw_then_velocity;
            position_state.pose = req.pose;
            res.is_going_to_position = true;
        }
        else
        {
            position_state.movement_state = velocity;
            position_state.pose.position = req.pose.position;
            res.is_going_to_position = true;
        }
        poseListX = interpolation(position_state.pose.position.x, current_pose.pose.position.x);
        poseListY = interpolation(position_state.pose.position.y, current_pose.pose.position.y);
        poseListZ = interpolation(position_state.pose.position.z, current_pose.pose.position.z);
        poseListX.push_back(position_state.pose.position.x);
        poseListY.push_back(position_state.pose.position.y);
        poseListZ.push_back(position_state.pose.position.z);
    }
    geometry_msgs::PoseStamped wp_to_pub;
    wp_to_pub.pose = position_state.pose;
    target_wp_pub.publish(wp_to_pub);
    return true;
}

void state_variables_init(ros::NodeHandle &nh)
{
    position_state.movement_state = position;
    position_state.yaw_spin_state = front_facing;

    int temp;
    nh.getParam("exploration_maneuver_phases_duration_secs", temp);
    exploration_maneuver_phases_duration_secs = ros::Duration(temp);

    offboard_enabled = false;
    nh.getParam("offboard_enabled", offboard_enabled);
    ROS_WARN_STREAM("[mav_comms] offboard_enabled " << offboard_enabled);
}

geometry_msgs::TwistStamped calculateVelocity(geometry_msgs::PoseStamped target, bool last)
{
    geometry_msgs::TwistStamped velocity_vector, unit_vector;
    double cruising_speed = 1.0;
    double magnitude_vec_to_target = sqrt(pow(target.pose.position.x - current_pose.pose.position.x, 2) +
                                          pow(target.pose.position.y - current_pose.pose.position.y, 2) +
                                          pow(target.pose.position.z - current_pose.pose.position.z, 2));
    unit_vector.twist.linear.x = (target.pose.position.x - current_pose.pose.position.x) / magnitude_vec_to_target;
    unit_vector.twist.linear.y = (target.pose.position.y - current_pose.pose.position.y) / magnitude_vec_to_target;
    unit_vector.twist.linear.z = (target.pose.position.z - current_pose.pose.position.z) / magnitude_vec_to_target;
    velocity_vector.twist.linear.x = unit_vector.twist.linear.x * cruising_speed;
    velocity_vector.twist.linear.y = unit_vector.twist.linear.y * cruising_speed;
    velocity_vector.twist.linear.z = unit_vector.twist.linear.z * cruising_speed;
    if (0.2 >= magnitude_vec_to_target)
    {
        flag_next_wp = true;
    }
    if (last == true)
    {
        velocity_vector.twist.linear.x = 0;
        velocity_vector.twist.linear.y = 0;
        velocity_vector.twist.linear.z = 0;
        position_state.movement_state = hover;
    }
    return velocity_vector;
}

void send_msg_to_px4()
{
    double take_off_altitude = 1.0;
    if (flying == false)
    {
        if (position_state.pose.position.z == 0)
        {
            position_state.pose.position.z = take_off_altitude;
        }
        if (current_pose.pose.position.z < take_off_altitude * 0.8)
        {
            ROS_WARN_STREAM("Taking Off");
            position_state.movement_state = take_off;
            flying = false;
        }
        else
        {
            flying = true;
        }
    }
    switch (position_state.movement_state)
    {
    case movement_state_t::take_off:
    {
        geometry_msgs::PoseStamped point_to_pub;
        point_to_pub.pose.position.z = take_off_altitude;
        local_pos_pub.publish(point_to_pub);
        position_state.movement_state = hover;
        break;
    }
    case movement_state_t::hover:
    {
        geometry_msgs::PoseStamped point_to_pub;
        point_to_pub.pose = position_state.pose;
        local_pos_pub.publish(point_to_pub);
        break;
    }
    case movement_state_t::position:
    {
        geometry_msgs::PoseStamped point_to_pub;
        point_to_pub.pose = position_state.pose;
        local_pos_pub.publish(point_to_pub);
        position_state.movement_state = hover;
        break;
    }
    case movement_state_t::yaw_then_velocity:
    {
        geometry_msgs::PoseStamped point_to_pub;
        point_to_pub.pose = target_orientation.pose;
        local_pos_pub.publish(point_to_pub);
        ros::Duration(4.0).sleep();
        position_state.movement_state = velocity;
        break;
    }
    case movement_state_t::velocity:
    {
        for (int i = 1; i < poseListX.size(); i++)
        {
            while (flag_next_wp == false)
            {
                geometry_msgs::PoseStamped interpolated_target;
                double dist_to_target = sqrt(pow(poseListX[i] - current_pose.pose.position.x, 2) +
                                             pow(poseListY[i] - current_pose.pose.position.y, 2) +
                                             pow(poseListZ[i] - current_pose.pose.position.z, 2));
                while (dist_to_target < 0.5 && i != poseListX.size() - 1)
                {
                    i++;
                    dist_to_target = sqrt(pow(poseListX[i] - current_pose.pose.position.x, 2) +
                                          pow(poseListY[i] - current_pose.pose.position.y, 2) +
                                          pow(poseListZ[i] - current_pose.pose.position.z, 2));
                    // ROS_WARN_STREAM("dist: " << dist_to_target << " m to [" << i << "]");
                    ros::Duration(0.1).sleep();
                }
                // ROS_WARN_STREAM("[" << i << "]" << poseListX[i] << " " << poseListY[i] << " " << poseListZ[i]);
                interpolated_target.pose.position.x = poseListX[i];
                interpolated_target.pose.position.y = poseListY[i];
                interpolated_target.pose.position.z = poseListZ[i];
                geometry_msgs::TwistStamped velocity_to_pub = calculateVelocity(interpolated_target, false);
                local_velocity_pub.publish(velocity_to_pub);
                ros::spinOnce();
                ros::Duration(0.1).sleep();
                if (flag_next_wp == true && i != poseListX.size())
                {
                    std::cout << "Next waypoint" << std::endl;
                }
            }
            flag_next_wp = false;
        }
        geometry_msgs::PoseStamped last_target;
        last_target.pose = position_state.pose;
        geometry_msgs::TwistStamped velocity_to_pub = calculateVelocity(last_target, true);
        break;
    }
    case movement_state_t::yaw_spin:
    {
        geometry_msgs::PoseStamped point_to_pub;
        point_to_pub.pose = position_state.pose;
        ros::Duration time_lapse = ros::Time::now() - position_state.yaw_spin_last_sent;
        switch (position_state.yaw_spin_state)
        {
        case front_facing:
        {
            position_state.yaw_spin_last_sent = ros::Time::now();
            position_state.yaw_spin_state = yaw_1;
            ROS_WARN_STREAM("[mav_comms] yaw_1");
            break;
        }
        case yaw_1:
        {
            if (time_lapse > exploration_maneuver_phases_duration_secs)
            {
                position_state.yaw_spin_last_sent = ros::Time::now();
                position_state.yaw_spin_state = yaw_2;
                ROS_WARN_STREAM("[mav_comms] yaw_2");
            }
            break;
        }
        case yaw_2:
        {
            if (time_lapse > exploration_maneuver_phases_duration_secs)
            {
                position_state.yaw_spin_last_sent = ros::Time::now();
                position_state.yaw_spin_state = yaw_3;
                ROS_WARN_STREAM("[mav_comms] yaw_3");
            }
            break;
        }
        case yaw_3:
        {
            if (time_lapse > exploration_maneuver_phases_duration_secs)
            {
                position_state.yaw_spin_last_sent = ros::Time::now();
                position_state.yaw_spin_state = front_facing;
                position_state.movement_state = position;
                ROS_WARN_STREAM("[mav_comms] front_facing");
            }
            break;
        }
        }
        point_to_pub.pose.orientation = tf::createQuaternionMsgFromYaw(position_state.yaw_spin_state * 0.0174532925);
        local_pos_pub.publish(point_to_pub);
        break;
    }
    }
}
} // namespace mav_comms

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mav_comms");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, mav_comms::state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, mav_comms::pose_cb);
    // ros::Subscriber stop_sub = nh.subscribe<std_msgs::Empty>("stop_uav", 10, mav_comms::stopUAV_cb);

    mav_comms::local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    mav_comms::local_velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    mav_comms::target_wp_pub = nh.advertise<geometry_msgs::PoseStamped>("target_wp", 10);
    // mav_comms::setpoint_raw_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);

    mav_comms::yaw_spin_service = nh.advertiseService("yaw_spin", mav_comms::yaw_spin_cb);
    mav_comms::target_position_service = nh.advertiseService("target_position", mav_comms::target_position_cb);
    mav_comms::target_position_vel_service = nh.advertiseService("target_position_vel", mav_comms::target_position_vel_cb);
    mav_comms::current_position_service = nh.advertiseService("current_position", mav_comms::current_position_cb);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    mav_comms::param_set_client = nh.serviceClient<mavros_msgs::ParamSet>("mavros/param/set");

    mav_comms::state_variables_init(nh);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(10);
    // === FCU CONNECTION ===
    while (ros::ok() && mav_comms::current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    // === px4 PARAM ===
    // param set MPC_XY_VEL_MAX 2.00
    // param set MPC_Z_VEL_MAX_DN 2.00
    // param set MPC_Z_VEL_MAX_UP 2.00

    // double flight_speed = 3;
    // nh.getParam("px4/flight_speed", flight_speed);
    // while(!mav_comms::set_mavros_param("MPC_XY_CRUISE", flight_speed)) { }
    // ROS_INFO_STREAM("[mav_comms] Velocity set to " << flight_speed << " m/s (MPC_XY_CRUISE)");
    // while(!mav_comms::set_mavros_param("MPC_XY_CRUISE", flight_speed)) { }
    // ROS_INFO_STREAM("[mav_comms] Velocity set to " << flight_speed << " m/s (MPC_XY_CRUISE)");
    // while(!mav_comms::set_mavros_param("MPC_Z_VEL_MAX_DN", flight_speed)) { }
    // ROS_INFO_STREAM("[mav_comms] MPC_Z_VEL_MAX_DN set to " << flight_speed);
    // while(!mav_comms::set_mavros_param("MPC_Z_VEL_MAX_UP", flight_speed)) { }
    // ROS_INFO_STREAM("[mav_comms] MPC_Z_VEL_MAX_UP set to " << flight_speed);

    // double MPC_ACC_HOR = 2; // minimum 1
    // while(!mav_comms::set_mavros_param("MPC_ACC_HOR", MPC_ACC_HOR)) { }
    // ROS_INFO_STREAM("[mav_comms] Acceleration set to " << MPC_ACC_HOR << " m/s (MPC_ACC_HOR)");
    // // NAV_ACC_RAD - Acceptance Radius
    // double NAV_ACC_RAD = 2; // minimum 0.01
    // while(!mav_comms::set_mavros_param("NAV_ACC_RAD", NAV_ACC_RAD)) { }
    // ROS_INFO_STREAM("[mav_comms] Acceptance Radius set to " << NAV_ACC_RAD << " m (NAV_ACC_RAD)");
    // // MPC_CRUISE_90 - Cruise speed when angle prev-current/current-next setpoint is 90 degrees.
    // double MPC_CRUISE_90 = 1;   // minimum 1
    // while(!mav_comms::set_mavros_param("MPC_CRUISE_90", MPC_CRUISE_90)) { }
    // ROS_INFO_STREAM("[mav_comms] Acceptance Radius set to " << MPC_CRUISE_90 << " m/s (MPC_CRUISE_90)");

    // double const offboard_mode_timeout_sec = 20;
    // while(!mav_comms::set_mavros_param("COM_OF_LOSS_T", offboard_mode_timeout_sec)) { }
    // ROS_INFO_STREAM("[mav_comms] Timeout from OFFBOARD mode set to " << offboard_mode_timeout_sec << " sec (COM_OF_LOSS_T)");
    // while(!mav_comms::set_mavros_param("COM_RC_IN_MODE", 1)) { }
    // ROS_INFO_STREAM("[mav_comms] COM_RC_IN_MODE set to " << 1);

    // while(!mav_comms::set_mavros_param("MPC_XY_P", 0.5)) { }
    // ROS_INFO_STREAM("[mav_comms] MPC_XY_P set to " << 0.5);

    // === SET POSITIONS ===
    /*  PROBLEMS AT STARTING NODE
        geometry_msgs::PoseStamped point_to_pub;
        point_to_pub.pose.position.x = 0;
        point_to_pub.pose.position.y = 0;
        point_to_pub.pose.position.z = 2;
        for (int i = 20; ros::ok() && i > 0; --i)
        { //send a few setpoints before starting
            mav_comms::local_pos_pub.publish(point_to_pub);
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO_STREAM("[mav_comms] Safety waypoints set to  " << point_to_pub.pose.position);
    */
    // === MAIN LOOP ===
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    bool armed = false;
    bool offboard_on = false;
    while (ros::ok())
    { // Position is always sent regardeless of the state to keep vehicle in offboard mode
        mav_comms::send_msg_to_px4();

        if (mav_comms::offboard_enabled)
        {
            if (mav_comms::current_state.mode != "OFFBOARD")
            {
                if (offboard_on)
                {
                    ROS_INFO_STREAM("[mav_comms] mode " << mav_comms::current_state.mode);
                }
                offboard_on = false;
                set_mode_client.call(offb_set_mode);
            }
            else
            {
                if (!offboard_on)
                {
                    ROS_INFO("[mav_comms] mode OFFBOARD");
                    offboard_on = true;
                }
                if (!mav_comms::current_state.armed)
                {
                    if (armed)
                    {
                        ROS_INFO("[mav_comms] Vehicle DISarmed");
                    }
                    armed = false;
                    arming_client.call(arm_cmd);
                }
                else
                {
                    if (!armed)
                    {
                        ROS_INFO("[mav_comms] Vehicle ARMED");
                        armed = true;
                    }
                }
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
}
