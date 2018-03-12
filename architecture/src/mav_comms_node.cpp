#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <architecture_msgs/PositionRequest.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/ParamSet.h>

namespace mav_comms
{
	mavros_msgs::State current_state;
	ros::Publisher stop_position_pub;
	struct Position { bool send; int waypoint_sequence; double x; double y; double z;};
	Position position_state;
    ros::ServiceClient param_set_client;

	bool disarmed(ros::Time& last_request)
	{
	    return !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0));
	}

	bool notOffboardMode(ros::Time& last_request)
	{
	    return current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0));
	}

	void state_cb(const mavros_msgs::State::ConstPtr& msg){
	    current_state = *msg;
	}

	bool set_mavros_param(std::string param_name, double param_value)
	{
	    mavros_msgs::ParamSet set_MPC_XY_CRUISE_srv;
	    set_MPC_XY_CRUISE_srv.request.param_id = param_name;
	    set_MPC_XY_CRUISE_srv.request.value.integer = param_value;
	    return param_set_client.call(set_MPC_XY_CRUISE_srv);
	}

	void stopUAV_cb(const std_msgs::Empty::ConstPtr& msg)
	{
		position_state.send = false;
		// Command in velocity
	    mavros_msgs::PositionTarget stop_position;
	    stop_position.type_mask = 0b0000101111000111;
	    stop_position.header.stamp = ros::Time::now();
	    stop_position.header.seq = 1;
	    stop_position_pub.publish(stop_position);
	    ROS_INFO("[mav_comms] STOP msg sent!");
	}

	void target_position_cb(const architecture_msgs::PositionRequest::ConstPtr& msg)
	{
		if(!position_state.send && msg.waypoint_sequence_id <= position_state.waypoint_sequence)
		{
			ROS_WARN_STREAM("[mav_comms] Rejecting position " << msg->point 
				<< ", seems to be from aborted waypoint sequence (id "
				<< msg->waypoint_sequence_id << ")");
		}
		else
		{
			position_state.send = true;
			position_state.waypoint_sequence_id = msg->waypoint_sequence_id;
			position_state.x = msg->point.x;
			position_state.y = msg->point.y;
			position_state.z = msg->point.z;
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mav_comms");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, mav_comms_node::state_cb);
    ros::Subscriber stop_sub = nh.subscribe<std_msgs::Empty>("stop_uav", 10, mav_comms_node::stopUAV_cb);
    ros::Subscriber target_position_sub = nh.subscribe<architecture_msgs::PositionRequest>("target_position", 10, target_position_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    stop_position_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    param_set_client = nh.serviceClient<mavros_msgs::ParamSet>("mavros/param/set");
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
    while(!mav_comms_node::set_mavros_param("MPC_XY_CRUISE", flight_speed))
    {
        
    }
    ROS_INFO_STREAM("Velocity set to " << flight_speed << " m/s (MPC_XY_CRUISE)");


    // === SET POSITIONS ===
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1;
    //send a few setpoints before starting
    for(int i = 20; ros::ok() && i > 0; --i) {
            mav_comms_node::local_pos_pub.publish(pose);
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
    	if( mav_comms_node::notOffboardMode(last_request)) 
        {
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent) {
                    ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } 
        else 
        {
            if( mav_comms_node::disarmed(last_request)) 
            {
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success) 
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            } 
            else
            { 
            	if (mav_comms_node::current_state.armed && position_state.send) 
	            {
	                // keep sending position
	                geometry_msgs::PoseStamped point_to_pub;
	                point_to_pub.pose.position.x = position_state.x;
	                point_to_pub.pose.position.y = position_state.y;
	                point_to_pub.pose.position.z = position_state.z;
	                local_pos_pub.publish();
	            }
	        }
        }
        ros::spinOnce();
        rate.sleep();
    }
}