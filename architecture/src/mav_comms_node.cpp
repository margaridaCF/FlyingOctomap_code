#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <architecture_msgs/PositionRequest.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/ParamSet.h>
#include <geometry_msgs/PoseStamped.h>
#include <architecture_msgs/YawSpin.h>
#include <tf/transform_datatypes.h>

namespace mav_comms
{
	mavros_msgs::State current_state;
	ros::Publisher setpoint_raw_pub;
    ros::Publisher local_pos_pub;
    ros::ServiceServer yaw_spin_service;
    enum movement_state_t {position, stop, yaw_spin};
	struct Position { movement_state_t movement_state; int waypoint_sequence;
        double x; double y; double z; };
    mavros_msgs::PositionTarget stop_position;
	Position position_state;
    ros::ServiceClient param_set_client;
    geometry_msgs::PoseStamped point_to_pub; // just to avoid creating a variable each time


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
        position_state.x = req.position.x;
        position_state.y = req.position.y;
        position_state.z = req.position.z;
        position_state.movement_state = yaw_spin;
    }

	void stopUAV_cb(const std_msgs::Empty::ConstPtr& msg)
	{
		position_state.movement_state = stop;
		// Command in velocity
	    // mavros_msgs::PositionTarget stop_position;
	    // stop_position.type_mask = 0b0000101111000111;
	    stop_position.header.stamp = ros::Time::now();
	    // stop_position.header.seq = 1;
	    setpoint_raw_pub.publish(stop_position);
	    // ROS_INFO("[mav_comms] STOP msg sent!");
	}

	void target_position_cb(const architecture_msgs::PositionRequest::ConstPtr& msg)
	{
		if(position_state.movement_state != position) 
        { 
            ROS_WARN_STREAM("[mav_comms] Rejecting position " << msg->position  
                << ", wrong movement state"); 
        } 
        else if( msg->waypoint_sequence_id != position_state.waypoint_sequence) 
		{
			ROS_WARN_STREAM("[mav_comms] Rejecting position " << msg->position 
				<< ", wrong movement state or request is from aborted waypoint sequence (id "
				<< msg->waypoint_sequence_id << ")");
		}
		else
		{
			position_state.movement_state = position;
			position_state.waypoint_sequence = msg->waypoint_sequence_id;
			position_state.x = msg->position.x;
			position_state.y = msg->position.y;
			position_state.z = msg->position.z;
		}
	}

    void state_variables_init()
    {
        position_state.movement_state = position;

        position_state.x = 0;
        position_state.y = 0;
        position_state.z = 1;

        stop_position.type_mask = 0b0000101111000111;
        stop_position.header.seq = 1;
    }

    void send_msg_to_px4()
    {
        switch(position_state.movement_state)
        {
            case movement_state_t::position:
            {
                point_to_pub.pose.position.x = position_state.x;
                point_to_pub.pose.position.y = position_state.y;
                point_to_pub.pose.position.z = position_state.z;
                local_pos_pub.publish(point_to_pub);
                break;
            }
            case movement_state_t::stop:
            {
                stop_position.header.stamp = ros::Time::now();
                setpoint_raw_pub.publish(stop_position);
                break;
            }
            case movement_state_t::yaw_spin:
            {

                geometry_msgs::PoseStamped msg;
                msg.header.stamp = ros::Time::now();
                msg.pose.position.x = position_state.x;
                msg.pose.position.y = position_state.y;
                msg.pose.position.z = position_state.z;
                msg.pose.orientation = tf::createQuaternionMsgFromYaw(0.174533);
                local_pos_pub.publish(msg);

                position_state.movement_state = position;
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
    ros::Subscriber stop_sub = nh.subscribe<std_msgs::Empty>("stop_uav", 10, mav_comms::stopUAV_cb);
    ros::Subscriber target_position_sub = nh.subscribe<architecture_msgs::PositionRequest>("target_position", 10, mav_comms::target_position_cb);

    mav_comms::local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    mav_comms::setpoint_raw_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    
    mav_comms::yaw_spin_service = nh.advertiseService("yaw_spin", mav_comms::yaw_spin_cb);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    mav_comms::param_set_client = nh.serviceClient<mavros_msgs::ParamSet>("mavros/param/set");

    mav_comms::state_variables_init();
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(100);
    // === FCU CONNECTION ===
    while(ros::ok() && mav_comms::current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    // === px4 PARAM ===
    double const flight_speed = 3;
    while(!mav_comms::set_mavros_param("MPC_XY_CRUISE", flight_speed)) { }
    ROS_INFO_STREAM("[mav_comms] Velocity set to " << flight_speed << " m/s (MPC_XY_CRUISE)");
    double const offboard_mode_timeout_sec = 20;
    while(!mav_comms::set_mavros_param("COM_OF_LOSS_T", offboard_mode_timeout_sec)) { }
    ROS_INFO_STREAM("[mav_comms] Timeout from OFFBOARD mode set to " << offboard_mode_timeout_sec << " sec (COM_OF_LOSS_T)");
    while(!mav_comms::set_mavros_param("COM_RC_IN_MODE", 1)) { }
    ROS_INFO_STREAM("[mav_comms] COM_RC_IN_MODE set to " << 1);


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
        ros::spinOnce();
        rate.sleep();
    }
}