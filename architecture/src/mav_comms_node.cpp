#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/ParamSet.h>
#include <architecture_msgs/PositionRequest.h>
#include <architecture_msgs/YawSpin.h>

namespace mav_comms
{
    bool offboard_enabled;
    ros::Duration exploration_maneuver_phases_duration_secs;
	mavros_msgs::State current_state;
	// ros::Publisher setpoint_raw_pub;
    ros::Publisher local_pos_pub;
    ros::ServiceServer yaw_spin_service;
    ros::ServiceServer target_position_service;
    enum movement_state_t {position, stop, yaw_spin};
    enum yaw_spin_maneuver_state_t {front_facing = 0, yaw_1 = 120, yaw_2 = 240, yaw_3 = 360};
    struct Position { movement_state_t movement_state; int waypoint_sequence;
        geometry_msgs::Pose pose; 
        yaw_spin_maneuver_state_t yaw_spin_state; 
        ros::Time yaw_spin_last_sent;};
    // mavros_msgs::PositionTarget stop_position;
    Position position_state;
    ros::ServiceClient param_set_client; 


	void state_cb(const mavros_msgs::State::ConstPtr& msg){
	    current_state = *msg;
	}

	bool set_mavros_param(std::string param_name, double param_value)
	{
	    mavros_msgs::ParamSet set_param_srv;
	    set_param_srv.request.param_id = param_name;
	    set_param_srv.request.value.integer = param_value;
	    return param_set_client.call(set_param_srv);
	}

    bool yaw_spin_cb(architecture_msgs::YawSpin::Request  &req, 
        architecture_msgs::YawSpin::Response &res)
    {
        position_state.pose.position = req.position;
        position_state.movement_state = yaw_spin;
        // ROS_INFO_STREAM("[mav_comms] Receiving yaw spin request " << req);
        return true;
    }

	// void stopUAV_cb(const std_msgs::Empty::ConstPtr& msg)
	// {
 //        if(position_state.movement_state == position)
 //        {
 //    		position_state.movement_state = stop;
 //    		// Command in velocity
 //    	    mavros_msgs::PositionTarget stop_position;
 //    	    stop_position.type_mask = 0b0000101111000111;
 //    	    stop_position.header.stamp = ros::Time::now();
 //    	    stop_position.header.seq = 1;
 //    	    setpoint_raw_pub.publish(stop_position);
 //    	    // ROS_INFO("[mav_comms] STOP msg sent!");
 //        }
	// }

	bool target_position_cb(architecture_msgs::PositionRequest::Request  &req, 
        architecture_msgs::PositionRequest::Response &res)
	{
        ROS_WARN_STREAM("[mav_comms] target_position_cb");
		if(position_state.movement_state != position) 
        { 
            ROS_WARN_STREAM("[mav_comms] Rejecting position " << req.pose  
                << ", wrong movement state"); 
            res.is_going_to_position = false;
        } 
        else if( req.waypoint_sequence_id < position_state.waypoint_sequence) 
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
        return true;
	}

    void state_variables_init(ros::NodeHandle& nh)
    {
        position_state.movement_state = position;
        position_state.yaw_spin_state = front_facing;

        int temp;
        nh.getParam("exploration_maneuver_phases_duration_secs", temp);
        exploration_maneuver_phases_duration_secs = ros::Duration(temp);

        offboard_enabled = false;
        nh.getParam("offboard_enabled", offboard_enabled);
        ROS_WARN_STREAM("[mav_comms] offboard_enabled" << offboard_enabled);
    }

    void send_msg_to_px4()
    {
        switch(position_state.movement_state)
        {
            case movement_state_t::position:
            {
                geometry_msgs::PoseStamped point_to_pub;
                point_to_pub.pose = position_state.pose;
                local_pos_pub.publish(point_to_pub);
                // ROS_INFO_STREAM("[mav_comms] Sending position " << point_to_pub.pose.position);
                break;
            }
            // case movement_state_t::stop:
            // {
            //     stop_position.header.stamp = ros::Time::now();
            //     setpoint_raw_pub.publish(stop_position);
            //     // ROS_INFO("[mav_comms] STOP msg sent!");
            //     break;
            // }
            case movement_state_t::yaw_spin:
            {
                geometry_msgs::PoseStamped point_to_pub;
                point_to_pub.pose = position_state.pose;
                ros::Duration time_lapse = ros::Time::now() - position_state.yaw_spin_last_sent;
                switch(position_state.yaw_spin_state)
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
                        if(time_lapse > exploration_maneuver_phases_duration_secs)
                        {
                            position_state.yaw_spin_last_sent = ros::Time::now();
                            position_state.yaw_spin_state = yaw_2;
                            ROS_WARN_STREAM("[mav_comms] yaw_2");
                        }
                        break;
                    }
                    case yaw_2:
                    {
                        if(time_lapse > exploration_maneuver_phases_duration_secs)
                        {
                            position_state.yaw_spin_last_sent = ros::Time::now();
                            position_state.yaw_spin_state = yaw_3;
                            ROS_WARN_STREAM("[mav_comms] yaw_3");

                        }
                        break;
                    }
                    case yaw_3:
                    {
                        if(time_lapse > exploration_maneuver_phases_duration_secs)
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
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mav_comms");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, mav_comms::state_cb);
    // ros::Subscriber stop_sub = nh.subscribe<std_msgs::Empty>("stop_uav", 10, mav_comms::stopUAV_cb);

    mav_comms::local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    // mav_comms::setpoint_raw_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    
    mav_comms::yaw_spin_service = nh.advertiseService("yaw_spin", mav_comms::yaw_spin_cb);
    mav_comms::target_position_service = nh.advertiseService("target_position", mav_comms::target_position_cb);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    mav_comms::param_set_client = nh.serviceClient<mavros_msgs::ParamSet>("mavros/param/set");


    mav_comms::state_variables_init(nh);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(10);
    // === FCU CONNECTION ===
    while(ros::ok() && mav_comms::current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    // === px4 PARAM ===
    double flight_speed = 3;
    nh.getParam("px4/flight_speed", flight_speed);
    while(!mav_comms::set_mavros_param("MPC_XY_CRUISE", flight_speed)) { }
    ROS_INFO_STREAM("[mav_comms] Velocity set to " << flight_speed << " m/s (MPC_XY_CRUISE)");
    double MPC_ACC_HOR = 2; // minimum 1
    while(!mav_comms::set_mavros_param("MPC_ACC_HOR", MPC_ACC_HOR)) { }
    ROS_INFO_STREAM("[mav_comms] Acceleration set to " << MPC_ACC_HOR << " m/s (MPC_ACC_HOR)"); 
    // NAV_ACC_RAD - Acceptance Radius
    double NAV_ACC_RAD = 2; // minimum 0.01
    while(!mav_comms::set_mavros_param("NAV_ACC_RAD", NAV_ACC_RAD)) { }
    ROS_INFO_STREAM("[mav_comms] Acceptance Radius set to " << NAV_ACC_RAD << " m (NAV_ACC_RAD)"); 
    // MPC_CRUISE_90 - Cruise speed when angle prev-current/current-next setpoint is 90 degrees.
    double MPC_CRUISE_90 = 1;   // minimum 1
    while(!mav_comms::set_mavros_param("MPC_CRUISE_90", MPC_CRUISE_90)) { }
    ROS_INFO_STREAM("[mav_comms] Acceptance Radius set to " << MPC_CRUISE_90 << " m/s (MPC_CRUISE_90)");


    double const offboard_mode_timeout_sec = 20;
    while(!mav_comms::set_mavros_param("COM_OF_LOSS_T", offboard_mode_timeout_sec)) { }
    ROS_INFO_STREAM("[mav_comms] Timeout from OFFBOARD mode set to " << offboard_mode_timeout_sec << " sec (COM_OF_LOSS_T)");
    while(!mav_comms::set_mavros_param("COM_RC_IN_MODE", 1)) { }
    ROS_INFO_STREAM("[mav_comms] COM_RC_IN_MODE set to " << 1);


    while(!mav_comms::set_mavros_param("MPC_Z_VEL_MAX_DN", 3)) { }
    ROS_INFO_STREAM("[mav_comms] MPC_Z_VEL_MAX_DN set to " << 3);

    // while(!mav_comms::set_mavros_param("MPC_XY_P", 0.5)) { }
    // ROS_INFO_STREAM("[mav_comms] MPC_XY_P set to " << 0.5);


    // === SET POSITIONS ===
    geometry_msgs::PoseStamped point_to_pub;
    point_to_pub.pose.position.x = 0;
    point_to_pub.pose.position.y = 0;
    point_to_pub.pose.position.z = 2;
    for(int i = 20; ros::ok() && i > 0; --i) {  //send a few setpoints before starting
            mav_comms::local_pos_pub.publish(point_to_pub);
            ros::spinOnce();
            rate.sleep();
    }
    ROS_INFO_STREAM("[mav_comms] Safety waypoints set to  " << point_to_pub.pose.position);
    // === MAIN LOOP ===
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    bool armed = false;
    bool offboard_on = false;
    while(ros::ok()) 
    {// Position is always sent regardeless of the state to keep vehicle in offboard mode
        mav_comms::send_msg_to_px4();

        if(mav_comms::offboard_enabled)
        {
            if( mav_comms::current_state.mode != "OFFBOARD") 
            {
                if(offboard_on)
                {
                    ROS_INFO_STREAM("[mav_comms] mode " <<  mav_comms::current_state.mode);
                }
                offboard_on = false;
                set_mode_client.call(offb_set_mode);
            } 
            else 
            {
                if(!offboard_on)
                {
                    ROS_INFO("[mav_comms] mode OFFBOARD");
                    offboard_on = true;
                }
                if( !mav_comms::current_state.armed) 
                {
                    if(armed)
                    {
                        ROS_INFO("[mav_comms] Vehicle DISarmed");
                    }
                    armed = false;
                    arming_client.call(arm_cmd);
                } 
                else
                {
                    if(!armed)
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
