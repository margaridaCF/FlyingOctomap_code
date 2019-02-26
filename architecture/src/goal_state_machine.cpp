#include <goal_state_machine.h>
#include <iostream>
#include <fstream>

namespace goal_state_machine
{
    GoalStateMachine::GoalStateMachine(frontiers_msgs::FrontierReply & frontiers_msg, double distance_inFront, double distance_behind, int circle_divisions, geometry_msgs::Point& geofence_min, geometry_msgs::Point& geofence_max, rviz_interface::PublishingInput pi, ros::ServiceClient& check_flightCorridor_client, double path_safety_margin, double frontier_safety_margin)
		: frontiers_msg(frontiers_msg), has_more_goals(false), frontier_index(0), geofence_min(geofence_min), geofence_max(geofence_max), pi(pi), path_safety_margin(path_safety_margin), check_flightCorridor_client(check_flightCorridor_client)
	{
		oppairs = observation_lib::OPPairs(circle_divisions, frontier_safety_margin, distance_inFront, distance_behind);
	}

	bool GoalStateMachine::is_flightCorridor_free() 
    {
        lazy_theta_star_msgs::CheckFlightCorridor srv;
        Eigen::Vector3d start_eigen = oppairs.get_current_start();
        Eigen::Vector3d end_eigen = oppairs.get_current_end();
        
        srv.request.start.x = start_eigen(0);
        srv.request.start.y = start_eigen(1);
        srv.request.start.z = start_eigen(2);
        srv.request.end.x   = end_eigen(0);
        srv.request.end.y   = end_eigen(1);
        srv.request.end.z   = end_eigen(2);
        srv.request.flight_corridor_width = path_safety_margin;

        bool check = false;
        while(!check)
        {
            check = check_flightCorridor_client.call(srv);
            if(!check)
            {
                ROS_ERROR("[Goal SM] Cannot place request to check flight corridor for flyby.");
            }
        }
		// #ifdef SAVE_LOG
		// std::ofstream log_file;
		// log_file.open ("/home/mfaria/Flying_Octomap_code/src/data/current/oppair.log", std::ofstream::app);
  //       if(!srv.response.free)
		// {
		// 	log_file << "[Goal SM] Path occupied between oppairs " << start_eigen << " and " << end_eigen << std::endl;
		// }
		// else
		// {
		// 	log_file << "[Goal SM] Path free between oppairs " << start_eigen << " and " << end_eigen << std::endl;
		// }
		// log_file.close();	
		// #endif
        return srv.response.free;
    }

	bool GoalStateMachine::IsOPPairValid() 
    {
    	if(is_inside_geofence(oppairs.get_current_start()) 
    		&& is_inside_geofence(oppairs.get_current_end()) 
    		&& is_flightCorridor_free()) 
		{
    		return true;
    	}
        return false;
    }

	geometry_msgs::Point GoalStateMachine::get_current_frontier() const
    {
        return frontiers_msg.frontiers[frontier_index].xyz_m;
    }

	bool GoalStateMachine::is_inside_geofence(Eigen::Vector3d target) const
	{
		if(target.x() < geofence_min.x 
                || target.y() < geofence_min.y 
                || target.x() < geofence_min.y 
                || target.x() > geofence_max.x 
                || target.y()> geofence_max.y  
                || target.z() > geofence_max.z)
        {
			#ifdef SAVE_LOG
			std::ofstream log_file;
			log_file.open ("/home/mfaria/Flying_Octomap_code/src/data/current/oppair.log", std::ofstream::app);
			log_file << "[Goal SM] " << target << " is outside geofence." << std::endl;
			log_file.close();	
			#endif
            return false;
        }
        else
        {
			#ifdef SAVE_LOG
			std::ofstream log_file;
			log_file.open ("/home/mfaria/Flying_Octomap_code/src/data/current/oppair.log", std::ofstream::app);
			// log_file << "[Goal SM] geofence ok." << std::endl;
			log_file.close();	
			#endif
        	return true;
        }
	}

	bool GoalStateMachine::hasNextFrontier() const
	{
		return frontier_index < frontiers_msg.frontiers_found-1;
	}

	void GoalStateMachine::resetOPPair(Eigen::Vector3d& uav_position)
	{
		geometry_msgs::Point curr_frontier_geom = get_current_frontier();
        Eigen::Vector3d new_frontier(curr_frontier_geom.x, curr_frontier_geom.y, curr_frontier_geom.z);
		oppairs.NewFrontier(new_frontier, uav_position, pi);
	}

	bool GoalStateMachine::pointToNextGoal(Eigen::Vector3d& uav_position)
	{	
		#ifdef SAVE_LOG	
		std::ofstream log_file;
		log_file.open ("/home/mfaria/Flying_Octomap_code/src/data/current/oppair.log", std::ofstream::app);
		#endif
		bool search = true;
		while(search)
		{
			// log_file << "[Goal SM] pointToNextGoal loop." << std::endl;
			for(bool existsNextOPPair = oppairs.Next();
				existsNextOPPair;
				existsNextOPPair = oppairs.Next())
			{
				// log_file << "[Goal SM] oppair loop." << std::endl;

				if(IsOPPairValid())
				{
					has_more_goals = true;
					#ifdef SAVE_LOG	
					log_file << "[Goal SM] found oppair." << std::endl;
					log_file.close();	
					#endif
					return true;
				}		
			}
			log_file << "[Goal SM] frontier " << get_current_frontier() << " is unreachable." << std::endl;
			// None of the remaining OPPairs were usable to inspect
			// The frontier is unreachable
            octomath::Vector3 unreachable (get_current_frontier().x, get_current_frontier().y, get_current_frontier().z);
            unobservable_set.insert(unreachable);
			if(hasNextFrontier())
			{
				frontier_index++;
				log_file << "[Goal SM] Next frontier " << get_current_frontier() << std::endl;
				resetOPPair(uav_position);
			}
			else
			{
				log_file << "[Goal SM] No more frontier in cache." << std::endl;
				search = false;
			}
		}
		has_more_goals = false;

		#ifdef SAVE_LOG	
		log_file.close();	
		#endif
		return false;
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