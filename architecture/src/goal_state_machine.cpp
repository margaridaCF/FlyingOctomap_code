#include <goal_state_machine.h>

#define RUNNING_ROS 1

namespace goal_state_machine
{
    GoalStateMachine::GoalStateMachine(frontiers_msgs::FrontierReply & frontiers_msg, double distance_inFront, double distance_behind, int circle_divisions, geometry_msgs::Point& geofence_min, geometry_msgs::Point& geofence_max, rviz_interface::PublishingInput pi, ros::ServiceClient& check_flightCorridor_client, double path_safety_margin, double sensing_distance)
		: frontiers_msg(frontiers_msg), has_more_goals(false), frontier_index(0), geofence_min(geofence_min), geofence_max(geofence_max), pi(pi), path_safety_margin(path_safety_margin), check_flightCorridor_client(check_flightCorridor_client), sensing_distance(sensing_distance), oppair_id(0)
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
					#ifdef RUNNING_ROS
					if(pi.publish)
					{
						rviz_interface::publish_arrow_straight_line(start, end, pi.marker_pub, false, oppair_id);
    					oppair_id++;
					}
					#endif
    				// ros::Duration(sleep_seconds).sleep();
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
			{
				rviz_interface::publish_arrow_straight_line(start, end, pi.marker_pub, false, oppair_id);
			}
			#endif
			oppair_id++;
			pi.marker_pub.publish(pi.waypoint_array);
			// ROS_INFO_STREAM("Start outside geofence.");
			// ros::Duration(s).sleep();
			return false;
    	}
    }

	geometry_msgs::Point GoalStateMachine::get_current_frontier() const
    {
        return frontiers_msg.frontiers[frontier_index].xyz_m;
    }

	void GoalStateMachine::get_current_frontier(Eigen::Vector3d & frontier) const
    {
		frontier << frontiers_msg.frontiers[frontier_index].xyz_m.x, frontiers_msg.frontiers[frontier_index].xyz_m.y, frontiers_msg.frontiers[frontier_index].xyz_m.z;
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
        Eigen::Vector3d new_frontier;
        get_current_frontier(new_frontier);
		
		oppairs_side.NewFrontier(new_frontier, uav_position, pi);
	    visualization_msgs::MarkerArray marker_array;
	    geometry_msgs::Point curr_frontier_geom;
	    curr_frontier_geom.x = new_frontier.x();
	    curr_frontier_geom.y = new_frontier.y();
	    curr_frontier_geom.z = new_frontier.z();

	    rviz_interface::build_sphere_basic(curr_frontier_geom, marker_array, "unknown_point", 0, 0, 1);

		// // To generate the points to pass under, the sensing_distance is used
		curr_frontier_geom.z = curr_frontier_geom.z-sensing_distance;
        Eigen::Vector3d new_frontier_under(new_frontier.x(), new_frontier.y(), curr_frontier_geom.z);
		oppairs_under.NewFrontier(new_frontier_under, uav_position, pi);
	    rviz_interface::build_sphere_basic(curr_frontier_geom, marker_array, "under_unknown_point", 0.5, 0.5, 1);
	    pi.marker_pub.publish(marker_array);
	}

	bool GoalStateMachine::pointToNextGoal(Eigen::Vector3d& uav_position)
	{	
	    std::stringstream aux_envvar_home (std::getenv("HOME"));
	    std::string folder_name = aux_envvar_home.str() + "/Flying_Octomap_code/src/data";
		std::ofstream log_file;
		log_file.open (folder_name+"/current/state_manager.log", std::ofstream::app);
        Eigen::Vector3d unknown;
        get_current_frontier(unknown);
		bool search = true;
		has_more_goals = false;
		while(search)
		{
			Eigen::Vector3d viewpoint;
			for(bool existsNextOPPair = oppairs_side.Next();
				existsNextOPPair;
				existsNextOPPair = oppairs_side.Next())
			{
				if( !IsUnobservable(unknown) && IsOPPairValid() )
				{
					has_more_goals = true;
					getFlybyStart(viewpoint);
					// ROS_INFO_STREAM("[Goal SM] Found goal side by side.");
					log_file << "[Goal SM] Ok for (" << unknown.x() << ", " << unknown.y() << ", " << unknown.z() << ") from (" << viewpoint.x() << ", " << viewpoint.y() << ", " << viewpoint.z() << ")" << std::endl;
					log_file.close();
					return true;
				}		
			}
			// log_file << "[Goal SM] Starting underneath search." << std::endl;
			is_oppairs_side = false;
			for(bool existsNextOPPair = oppairs_under.Next();
				existsNextOPPair;
				existsNextOPPair = oppairs_under.Next())
			{
				if( !IsUnobservable(unknown) && IsOPPairValid() )
				{
					has_more_goals = true;
					getFlybyStart(viewpoint);
					// ROS_INFO_STREAM("[Goal SM] Found goal side underneath.");
					log_file << "[Goal SM] Ok for (" << unknown.x() << ", " << unknown.y() << ", " << unknown.z() << ") from (" << viewpoint.x() << ", " << viewpoint.y() << ", " << viewpoint.z() << ")" << std::endl;
					log_file.close();
					return true;
				}		
			}
			// ROS_INFO_STREAM("[Goal SM] frontier (" << get_current_frontier().x << ", " << get_current_frontier().y << ", " << get_current_frontier().z << ") is unreachable.");
			// ros::Duration(5).sleep();
			// None of the remaining OPPairs were usable to inspect
			if(hasNextFrontier())
			{
				// log_file << "[Goal SM] Increment frontier index " << std::endl;
				frontier_index++;
        		get_current_frontier(unknown);
				resetOPPair(uav_position);
			}
			else
			{
				// ROS_INFO_STREAM("[Goal SM] No more frontier in cache.");
				search = false;
			}
		}

		// #ifdef SAVE_LOG	
		log_file.close();	
		// #endif
		return has_more_goals;
	}

	void GoalStateMachine::DeclareUnobservable()
	{
		// The frontier is unobser
        // Eigen::Vector3d unknown (get_current_frontier().x, get_current_frontier().y, get_current_frontier().z);
		Eigen::Vector3d unknown;
		get_current_frontier(unknown);

        unobservable_set.insert(std::make_pair(unknown, getCurrentOPPairs().get_current_start()));

		// #ifdef SAVE_LOG	
	    std::stringstream aux_envvar_home (std::getenv("HOME"));
	    std::string folder_name = aux_envvar_home.str() + "/Flying_Octomap_code/src/data";
		std::ofstream log_file;
		log_file.open (folder_name+"/current/state_manager.log", std::ofstream::app);
		ROS_INFO_STREAM("[Goal SM] (" << unknown.x() << ", " << unknown.y() << ", " << unknown.z() << ") unobservable from (" << getCurrentOPPairs().get_current_start().x() << ", " << getCurrentOPPairs().get_current_start().y() << ", " << getCurrentOPPairs().get_current_start().z() << ")");
		log_file << "[Goal SM] (" << unknown.x() << ", " << unknown.y() << ", " << unknown.z() << ") unobservable from (" << getCurrentOPPairs().get_current_start().x() << ", " << getCurrentOPPairs().get_current_start().y() << ", " << getCurrentOPPairs().get_current_start().z() << ")" << std::endl;
		for (auto i = unobservable_set.begin(); i != unobservable_set.end(); ++i)
		{
			log_file << "(" << i->first.x() << ", " << i->first.y() << ", " << i->first.z() << ") - (" << i->second.x() << ", " << i->second.y() << ", " << i->second.z() << ")"	<< std::endl;
		}
		log_file.close();	
		// #endif
	}

	bool GoalStateMachine::IsUnobservable(Eigen::Vector3d const& unknown)
	{
		return IsUnobservable(unknown, getCurrentOPPairs().get_current_start());
	}

	bool GoalStateMachine::IsUnobservable(Eigen::Vector3d const& unobservable, Eigen::Vector3d const& viewpoint)
	{



		bool is_unobservable = ! (unobservable_set.find(std::make_pair(unobservable, viewpoint)) ==  unobservable_set.end() );
		// if (is_unobservable) ros::Duration(60).sleep();
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