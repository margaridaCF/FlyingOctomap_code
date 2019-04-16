#include <goal_state_machine.h>
#include <iostream>
#include <fstream>

#define RUNNING_ROS 1

namespace goal_state_machine
{
    GoalStateMachine::GoalStateMachine(frontiers_msgs::FrontierReply & frontiers_msg, double distance_inFront, double distance_behind, int circle_divisions, geometry_msgs::Point& geofence_min, geometry_msgs::Point& geofence_max, rviz_interface::PublishingInput pi, ros::ServiceClient& check_flightCorridor_client, double path_safety_margin)
		: frontiers_msg(frontiers_msg), has_more_goals(false), frontier_index(0), geofence_min(geofence_min), geofence_max(geofence_max), pi(pi), path_safety_margin(path_safety_margin), check_flightCorridor_client(check_flightCorridor_client), sensing_distance(path_safety_margin), oppair_id(0)
	{
		// sensing_distance = std::round(    (path_safety_margin/2 ) * 10   )  / 10;
		sensing_distance = path_safety_margin/2 + 1;
		oppairs_side  = observation_lib::OPPairs(circle_divisions, sensing_distance, distance_inFront, distance_behind);
        unobservable_set = unobservable_pair_set(); 

		double distance_behind_under, distance_inFront_under;
		double sensor_shape_offset = sensing_distance / std::tan(0.872665);
		double flyby_distance      = distance_behind + distance_inFront;
		distance_behind_under      = distance_behind + sensor_shape_offset;
		double diference           = flyby_distance - distance_behind_under;
		if(diference < 0)
		{
			distance_inFront_under = 0;
		}
		else
		{
			distance_inFront_under = diference; 	
		}
		ROS_INFO_STREAM("[Goal SM] diference = distance_behind + sensor_shape_offset = " << distance_behind << " + " << sensor_shape_offset << " = " << distance_behind_under);
		ROS_INFO_STREAM("[Goal SM] Under      circle_divisions:" << circle_divisions/2 << ", distance from unknown: " << 0.1 << ", distance_inFront_under: " << distance_inFront_under << ", distance_behind_under: " << distance_behind_under);

		oppairs_under = observation_lib::OPPairs(circle_divisions/2, 1, distance_inFront_under, distance_behind_under);
	}

	observation_lib::OPPairs& GoalStateMachine::getCurrentOPPairs()
	{
		if(is_oppairs_side) return oppairs_side;
		else 				return oppairs_under;	
	}

	bool GoalStateMachine::is_flightCorridor_free() 
    {
        lazy_theta_star_msgs::CheckFlightCorridor srv;
        Eigen::Vector3d start_eigen = getCurrentOPPairs().get_current_start();
        Eigen::Vector3d end_eigen = getCurrentOPPairs().get_current_end();
        
        srv.request.start.x = start_eigen(0);
        srv.request.start.y = start_eigen(1);
        srv.request.start.z = start_eigen(2);
        srv.request.end.x   = end_eigen(0);
        srv.request.end.y   = end_eigen(1);
        srv.request.end.z   = end_eigen(2);
        // Inflated the space required to be free around the flyby.
        srv.request.flight_corridor_width = path_safety_margin + 1; // This magic number is to inflate the safe space around the flyby
        // Without it frequently the path planner was asked for impossible goals because the start of the line of sight was just outside a particular voxel
        // With this we guarentee the flyby to stay away from obstacles and unknown space.

        bool check = false;
        while(!check)
        {
            check = check_flightCorridor_client.call(srv);
            if(!check)
            {
                ROS_ERROR("[Goal SM] Cannot place request to check flight corridor for flyby.");
            }
        }
        return srv.response.free;
    }

	bool GoalStateMachine::IsOPPairValid() 
    {
		geometry_msgs::Point start, end;
		start.x = getCurrentOPPairs().get_current_start().x();
		start.y = getCurrentOPPairs().get_current_start().y();
		start.z = getCurrentOPPairs().get_current_start().z();
		end.x = getCurrentOPPairs().get_current_end().x();
		end.y = getCurrentOPPairs().get_current_end().y();
		end.z = getCurrentOPPairs().get_current_end().z();
    	// int sleep_seconds = 0;
    	bool start_inside_geofence = is_inside_geofence(getCurrentOPPairs().get_current_start());
    	if(start_inside_geofence)
    	{
			bool end_inside_geofence = is_inside_geofence(getCurrentOPPairs().get_current_end());
    		if(end_inside_geofence)
	    	{
	    		bool fc_free = is_flightCorridor_free();
				if(!fc_free)
				{
    				// ros::Duration(sleep_seconds).sleep();
					// ROS_INFO_STREAM("[Goal SM] Flight corridor had something.");

					#ifdef RUNNING_ROS
					if(pi.publish)
					{
						rviz_interface::publish_arrow_straight_line(start, end, pi.marker_pub, false, oppair_id);
					}
					#endif
				}
				return fc_free;
	    	}
	    	else
	    	{
	    		geometry_msgs::Point outsider;
    			outsider.x = getCurrentOPPairs().get_current_end().x();
    			outsider.y = getCurrentOPPairs().get_current_end().y();
    			outsider.z = getCurrentOPPairs().get_current_end().z();
				#ifdef RUNNING_ROS
				if(pi.publish)
				{
					rviz_interface::publish_arrow_straight_line(start, end, pi.marker_pub, false, oppair_id);
				}
		    	rviz_interface::build_endOPP_outsideGeofence(outsider, pi.waypoint_array, oppair_id);
				#endif
    			oppair_id++;
    			pi.marker_pub.publish(pi.waypoint_array);
    			// ROS_INFO_STREAM("End outside geofence.");
    			// ros::Duration(sleep_seconds).sleep();
    			return false;
	    	}
    	}
    	else
    	{
    		geometry_msgs::Point outsider;
			outsider.x = getCurrentOPPairs().get_current_start().x();
			outsider.y = getCurrentOPPairs().get_current_start().y();
			outsider.z = getCurrentOPPairs().get_current_start().z();
			#ifdef RUNNING_ROS
			rviz_interface::build_startOPP_outsideGeofence(outsider, pi.waypoint_array, oppair_id);
			if(pi.publish)
				rviz_interface::publish_arrow_straight_line(start, end, pi.marker_pub, false, oppair_id);
			}
			#endif
			oppair_id++;
			pi.marker_pub.publish(pi.waypoint_array);
			// ROS_INFO_STREAM("Start outside geofence.");
			// ros::Duration(sleep_seconds).sleep();
			return false;
    	}
    }

	geometry_msgs::Point GoalStateMachine::get_current_frontier() const
    {
        return frontiers_msg.frontiers[frontier_index].xyz_m;
    }

	bool GoalStateMachine::is_inside_geofence(Eigen::Vector3d target) const
	{
		if(        target.x() < geofence_min.x 
                || target.y() < geofence_min.y 
                || target.z() < geofence_min.z 

                || target.x() > geofence_max.x 
                || target.y() > geofence_max.y  
                || target.z() > geofence_max.z)
        {
            return false;
        }
        else
        {
        	return true;
        }
	}

	bool GoalStateMachine::hasNextFrontier() const
	{
		return frontier_index < frontiers_msg.frontiers_found-1;
	}

	void GoalStateMachine::resetOPPair(Eigen::Vector3d& uav_position)
	{
		is_oppairs_side = true;
		geometry_msgs::Point curr_frontier_geom = get_current_frontier();
		
        Eigen::Vector3d new_frontier(curr_frontier_geom.x, curr_frontier_geom.y, curr_frontier_geom.z);
		oppairs_side.NewFrontier(new_frontier, uav_position, pi);
	    visualization_msgs::MarkerArray marker_array;
	    rviz_interface::build_sphere_basic(curr_frontier_geom, marker_array, "unknown_point", 0, 0, 1);

		// // To generate the points to pass under, the sensing_distance is used
		curr_frontier_geom.z = curr_frontier_geom.z-sensing_distance;
        Eigen::Vector3d new_frontier_under(curr_frontier_geom.x, curr_frontier_geom.y, curr_frontier_geom.z);
		oppairs_under.NewFrontier(new_frontier_under, uav_position, pi);
	    rviz_interface::build_sphere_basic(curr_frontier_geom, marker_array, "under_unknown_point", 0.5, 0.5, 1);
	    pi.marker_pub.publish(marker_array);
	}

	bool GoalStateMachine::pointToNextGoal(Eigen::Vector3d& uav_position)
	{	
		// #ifdef SAVE_LOG	
		// std::ofstream log_file;
		// log_file.open ("/home/mfaria/Flying_Octomap_code/src/data/current/oppair.log", std::ofstream::app);
		// #endif
		bool search = true;
		has_more_goals = false;
		while(search)
		{
			for(bool existsNextOPPair = oppairs_side.Next();
				existsNextOPPair;
				existsNextOPPair = oppairs_side.Next())
			{
				if( !IsUnobservable(uav_position) && IsOPPairValid() )
				{
					// ROS_INFO_STREAM("[Goal SM] Found goal side by side.");
					has_more_goals = true;
					return true;
				}		
			}
			// ROS_INFO_STREAM("[Goal SM] Starting underneath search.");
			is_oppairs_side = false;
			for(bool existsNextOPPair = oppairs_under.Next();
				existsNextOPPair;
				existsNextOPPair = oppairs_under.Next())
			{
				if( !IsUnobservable(uav_position) && IsOPPairValid() )
				{
					ROS_INFO_STREAM("[Goal SM] Found goal side underneath.");
					has_more_goals = true;
					return true;
				}		
			}
			ROS_INFO_STREAM("[Goal SM] frontier (" << get_current_frontier().x << ", " << get_current_frontier().y << ", " << get_current_frontier().z << ") is unreachable.");
			ros::Duration(5).sleep();
			// None of the remaining OPPairs were usable to inspect
			if(hasNextFrontier())
			{
				frontier_index++;
				// ROS_INFO_STREAM("[Goal SM] Next frontier " << get_current_frontier());
				resetOPPair(uav_position);
			}
			else
			{
				ROS_INFO_STREAM("[Goal SM] No more frontier in cache.");
				search = false;
			}
		}

		// #ifdef SAVE_LOG	
		// log_file.close();	
		// #endif
		return has_more_goals;
	}

	void GoalStateMachine::DeclareUnobservable(Eigen::Vector3d const&  unobservable, Eigen::Vector3d const& viewpoint)
	{
		// The frontier is unobservable
        unobservable_set.insert(std::make_pair(unobservable, viewpoint));
	}

	bool GoalStateMachine::IsUnobservable(Eigen::Vector3d const& viewpoint)
	{
		return IsUnobservable(getCurrentOPPairs().get_current_start(), viewpoint);
	}

	bool GoalStateMachine::IsUnobservable(Eigen::Vector3d const& unobservable, Eigen::Vector3d const& viewpoint)
	{
		bool is_unobservable = ! (unobservable_set.find(std::make_pair(unobservable, viewpoint)) ==  unobservable_set.end() );
		if (is_unobservable) ros::Duration(60).sleep();
		return is_unobservable;
	}

	void GoalStateMachine::NewFrontiers(frontiers_msgs::FrontierReply & new_frontiers_msg)
	{
		frontiers_msg = new_frontiers_msg;
		frontier_index = 0;
		resetOPPair_flag = true;
	}


	bool GoalStateMachine::NextGoal(Eigen::Vector3d& uav_position)
	{
		if(resetOPPair_flag)
		{
			resetOPPair(uav_position);
			resetOPPair_flag = false;
		}
		return pointToNextGoal(uav_position);
	}
}