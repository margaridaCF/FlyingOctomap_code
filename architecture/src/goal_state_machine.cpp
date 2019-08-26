#include <goal_state_machine.h>
#include <ltStar_lib_ortho.h>
#include <chrono>
#include <utility>   
#include <sstream>
#include <fstream>
#include <string>
#include <chrono>
#include <boost/filesystem.hpp>

#define RUNNING_ROS 1
#define SAVE_CSV 1

namespace goal_state_machine
{
    std::ofstream log_file;

    GoalStateMachine::GoalStateMachine(ros::ServiceClient& find_frontiers_client, double distance_inFront, double distance_behind, int circle_divisions, geometry_msgs::Point& geofence_min, geometry_msgs::Point& geofence_max, rviz_interface::PublishingInput pi, double path_safety_margin, double sensing_distance, double local_fence_side)
		: find_frontiers_client(find_frontiers_client), has_more_goals(false), frontier_index(0), geofence_min(geofence_min), geofence_max(geofence_max), pi(pi), path_safety_margin(path_safety_margin), sensing_distance(sensing_distance), oppair_id(0), new_map(true), global(true), first_request(true), local_fence_side(local_fence_side), first_global_request(true), global_search_it(0)
	{
		oppairs_side  = observation_lib::OPPairs(circle_divisions, sensing_distance, distance_inFront, distance_behind, observation_lib::translateAdjustDirection);
        unobservable_set = unobservable_pair_set(); 

        frontier_srv.response.frontiers_found = 0;
        frontier_srv.response.success = false;
        frontier_request_count = 0;

		flyby_length = distance_inFront + distance_behind;

		double distance_from_unknown_under = 1;
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

		oppairs_under = observation_lib::OPPairs(circle_divisions/2, distance_from_unknown_under, distance_inFront_under, distance_behind_under, observation_lib::translate);

	}

	void GoalStateMachine::openCsv()
	{
		std::stringstream aux_envvar_home (std::getenv("HOME"));
	    std::string folder_name = aux_envvar_home.str() + "/Flying_Octomap_code/src/data";
		csv_file.open (folder_name+"/current/goal_sm.csv", std::ofstream::app);
		csv_file << "not_observable,not_visible,oppair_not_valid,start_not_reachable,outside_start,outside_end,obstacles_in_flight_corridor,is_oppairs_side" << std::endl;
	}

	void GoalStateMachine::NewMap()
	{
		if(!first_request) global = false;
		new_map = true;
		frontier_srv.response.success = false;
	}

	bool hasLineOfSight_UnknownAsFree(LazyThetaStarOctree::InputData const& input)
	{
		octomath::Vector3 dummy;
		octomath::Vector3 direction = input.goal - input.start;
		bool is_visible = !input.octree.castRay( input.start, direction, dummy, true, direction.norm());
		return is_visible;
	}

	observation_lib::OPPairs& GoalStateMachine::getCurrentOPPairs()
	{
		if(is_oppairs_side) return oppairs_side;
		else 				return oppairs_under;	
	}

	bool GoalStateMachine::checkFligthCorridor(double flight_corridor_width, Eigen::Vector3d& start, Eigen::Vector3d& end, ros::Publisher const& marker_pub)
	{
		octomath::Vector3 start_o(start.x(), start.y(), start.z());
		octomath::Vector3 end_o(end.x(), end.y(), end.z());
		LazyThetaStarOctree::InputData input (*octree, start_o, end_o, flight_corridor_width);
		
		return LazyThetaStarOctree::is_flight_corridor_free(input, rviz_interface::PublishingInput( marker_pub, false));
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
	    		Eigen::Vector3d start_e = getCurrentOPPairs().get_current_start();
	    		Eigen::Vector3d end_e = getCurrentOPPairs().get_current_end();

	    		bool fc_free = checkFligthCorridor(path_safety_margin + octree->getResolution(), start_e, end_e, pi.marker_pub);
				if(!fc_free)
				{
					#ifdef RUNNING_ROS
					if(pi.publish)
					{
						rviz_interface::publish_arrow_straight_line(start, end, pi.marker_pub, false, oppair_id);
    					oppair_id++;
					}
					#endif
    				// ros::Duration(3).sleep();
					csv_file << ",,1,,,,1," << is_oppairs_side << std::endl;
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
				csv_file << ",,1,,1,,," << std::endl;
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
			csv_file << ",,1,,1,,," << std::endl;
			return false;
    	}
    }

    bool GoalStateMachine::IsVisible(Eigen::Vector3d unknown)
    {
    	Eigen::Vector3d start_e = getCurrentOPPairs().get_current_start();
        octomath::Vector3 start(start_e.x(), start_e.y(), start_e.z());
		octomath::Vector3 end  (unknown.x(), unknown.y(), unknown.z());
		LazyThetaStarOctree::InputData input (*octree, start, end, 0);

		bool has_visibility_forwards = LazyThetaStarOctree::hasLineOfSight_UnknownAsFree(input, pi);
		bool has_visibility_backwards = LazyThetaStarOctree::hasLineOfSight_UnknownAsFree(LazyThetaStarOctree::InputData (*octree, end, start, 0), pi);
		bool has_visibility = has_visibility_forwards && has_visibility_backwards;
		if(!has_visibility)
		{
			csv_file << ",1,,,,,," << std::endl;
			// log_file << "[Goal] There is an obstacle between the start of the flyby and the unknown point." << std::endl;
			rviz_interface::publish_arrow_path_visibility(input.start, input.goal, pi.marker_pub, has_visibility, 58);
		}
    	// ros::Duration(3).sleep();
        return has_visibility;
    }

	geometry_msgs::Point GoalStateMachine::get_current_frontier() 
    {
    	if(!frontier_srv.response.success) 
    	{
    		log_file << "[Goal SM] A get_current_frontier without frontiers!" << std::endl;
    		ROS_ERROR("[Goal SM] A get_current_frontier without frontiers!");
    	}
        return frontier_srv.response.frontiers[frontier_index].xyz_m;
    }

	void GoalStateMachine::get_current_frontier(Eigen::Vector3d & frontier) 
    {
    	if(!frontier_srv.response.success) 
    	{
    		log_file << "[Goal SM] B get_current_frontier without frontiers!" << std::endl;
    		ROS_ERROR("[Goal SM] B get_current_frontier without frontiers!");
    	}
		frontier << frontier_srv.response.frontiers[frontier_index].xyz_m.x, frontier_srv.response.frontiers[frontier_index].xyz_m.y, frontier_srv.response.frontiers[frontier_index].xyz_m.z;
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
		if(!frontier_srv.response.success) return false;
		return frontier_index < frontier_srv.response.frontiers_found-1;
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

	bool GoalStateMachine::fillLocalGeofence()
	{
		Eigen::Vector3d start(success_flyby_start.x, success_flyby_start.y, success_flyby_start.z);
		Eigen::Vector3d end(success_flyby_end.x, success_flyby_end.y, success_flyby_end.z);
		
		Eigen::Vector3d direction = end - start;
        if(direction.norm() <= 0.01)
        {
                ROS_ERROR_STREAM("Still have to do this! start " << success_flyby_start << "; end " << success_flyby_end);
        }
        Eigen::Vector3d ortho = Eigen::Vector3d::UnitZ().cross(direction);
        ortho.normalize();
        direction.normalize();
        double extension  = local_fence_side - flyby_length;
        double range = local_fence_side * 2;
        Eigen::Vector3d e = start - (direction * extension/2); 
        Eigen::Vector3d a = e + (ortho     * range);
        Eigen::Vector3d b = a + (direction * local_fence_side);
        Eigen::Vector3d c = e - (ortho     * range);
        Eigen::Vector3d d = c + (direction * local_fence_side);
        // Geofence
        // Min
        frontier_srv.request.min.x = std::max(std::min({a.x(), b.x(), c.x(), d.x()}), geofence_min.x);
        frontier_srv.request.min.y = std::max(std::min({a.y(), b.y(), c.y(), d.y()}), geofence_min.y);
        frontier_srv.request.min.z = std::max(std::min({start.z(), end.z()}), geofence_min.z);
        // Max
        frontier_srv.request.max.x = std::min(std::max({a.x(), b.x(), c.x(), d.x()}), geofence_max.x);
        frontier_srv.request.max.y = std::min(std::max({a.y(), b.y(), c.y(), d.y()}), geofence_max.y);
        frontier_srv.request.max.z = std::min(frontier_srv.request.min.z+ std::abs(start.z() - end.z()) + range, geofence_max.z);
        // Guarentee minimum dimensions 
        frontier_srv.request.min.x = std::min( (frontier_srv.request.min.x ), (frontier_srv.request.max.x - local_fence_side) );
        frontier_srv.request.min.y = std::min( (frontier_srv.request.min.y ), (frontier_srv.request.max.y - local_fence_side) );
        frontier_srv.request.min.z = std::min( (frontier_srv.request.min.z ), (frontier_srv.request.max.z - local_fence_side) );

        if(pi.publish)
        {
	        visualization_msgs::MarkerArray marker_array;
	        // octomath::Vector3 a_o(a.x(), a.y(), a.z());
	        // octomath::Vector3 b_o(b.x(), b.y(), b.z());
	        // octomath::Vector3 c_o(c.x(), c.y(), c.z());
	        // octomath::Vector3 d_o(d.x(), d.y(), d.z());
	        // visualization_msgs::Marker marker;
	        // octomath::Vector3 start_o(start.x(), start.y(), start.z());
	        // rviz_interface::build_waypoint(start_o, 0.1, 0,   10, marker, 5);
	        // marker.ns = "start";
	        // marker_array.markers.push_back( marker );
	        // octomath::Vector3 end_o(end.x(), end.y(), end.z());
	        // rviz_interface::build_waypoint(end_o, 0.1, 0,   11, marker, 5);
	        // marker.ns = "end";
	        // marker_array.markers.push_back( marker );

	        // rviz_interface::build_waypoint(a_o, 0.5, 0,   1, marker, 1);
	        // marker.ns = "a";
	        // marker_array.markers.push_back( marker );
	        // rviz_interface::build_waypoint(b_o, 0.5, 250, 2, marker, 1);
	        // marker.ns = "b";
	        // marker_array.markers.push_back( marker );
	        // rviz_interface::build_waypoint(c_o, 0.5, 0  , 3, marker, 9);
	        // marker.ns = "c";
	        // marker_array.markers.push_back( marker );
	        // rviz_interface::build_waypoint(d_o, 0.5, 250, 4, marker, 9);
	        // marker.ns = "d";
	        // marker_array.markers.push_back( marker );
	        // rviz_interface::build_arrow_type(start_o, end_o, marker_array, 20, true);
	        // octomath::Vector3 end_ortho(start_o.x()+ortho.x(), start_o.y()+ortho.y(), start_o.z()+ortho.z());
	        // rviz_interface::build_arrow_type(start_o, end_ortho, marker_array, 21, false);
	        // ROS_INFO_STREAM("");
	        // ROS_INFO_STREAM("Extension = " << extension );
	        // ROS_INFO_STREAM("Start = (" << start.x() << ", " << start.y() << ", " << start.z() << ")");
	        // ROS_INFO_STREAM("End = (" << end.x() << ", " << end.y() << ", " << end.z() << ")");
	        // ROS_INFO_STREAM("End Ortho = (" << end_ortho.x() << ", " << end_ortho.y() << ", " << end_ortho.z() << ")");
	        
	        // ROS_INFO_STREAM("e = (" << e.x() << ", " << e.y() << ", " << e.z() << ")");

	        // ROS_INFO_STREAM("a = (" << a.x() << ", " << a.y() << ", " << a.z() << ")");
	        // ROS_INFO_STREAM("b = (" << b.x() << ", " << b.y() << ", " << b.z() << ")");

	        // ROS_INFO_STREAM("c = (" << c.x() << ", " << c.y() << ", " << c.z() << ")");
	        // ROS_INFO_STREAM("d = (" << d.x() << ", " << d.y() << ", " << d.z() << ")");

	        // ROS_INFO_STREAM("Direction = (" << direction.x() << ", " << direction.y() << ", " << direction.z() << ")");
	        // ROS_INFO_STREAM("Ortho = (" << ortho.x() << ", " << ortho.y() << ", " << ortho.z() << ")");
	        // ROS_INFO_STREAM("Request " << frontier_srv.request);
	        octomath::Vector3 min(frontier_srv.request.min.x, frontier_srv.request.min.y, frontier_srv.request.min.z);
	        octomath::Vector3 max(frontier_srv.request.max.x, frontier_srv.request.max.y, frontier_srv.request.max.z);
	        // ROS_INFO_STREAM("Min = (" << frontier_srv.request.min.x << ", " << frontier_srv.request.min.y << ", " << frontier_srv.request.min.z << ")");
	        // ROS_INFO_STREAM("Max = (" << frontier_srv.request.max.x << ", " << frontier_srv.request.max.y << ", " << frontier_srv.request.max.z<< ")");
	        rviz_interface::publish_geofence (min, max, marker_array);

	        pi.marker_pub.publish(marker_array);
        }
	}

	bool GoalStateMachine::findFrontiersAllMap(Eigen::Vector3d& uav_position)
	{
		first_request = false;
		if (global)
		{
			frontier_srv.request.min = geofence_min;
			frontier_srv.request.max = geofence_max;
			frontier_srv.request.global_search_it = global_search_it;
		}
		else
		{
			fillLocalGeofence();
			frontier_srv.request.global_search_it = 0;
		}
		frontier_srv.request.frontier_amount = 35;
		bool found_frontiers = findFrontiers_CallService(uav_position);
		first_global_request = false;
		if (found_frontiers)
		{
	        // ROS_INFO_STREAM("[Goal SM] Request " << frontier_srv.request);
	        if(global)
	        {
        		ROS_INFO_STREAM("[Goal SM] The global search still found a frontier.");
        		global_search_it = frontier_srv.response.global_search_it;
	        }
        	else
        		ROS_INFO_STREAM("[Goal SM] Just the local search was was enough to find a frontier");
			return true;
		} 
		// If it was a local search, there is still global search;
		if(!global)
		{
			global = true;
			ROS_INFO_STREAM("[Goal SM] Switch to global");
			first_global_request = true;
			return findFrontiersAllMap(uav_position);
		}
		else
		{
			return false;
		}
	}

	bool GoalStateMachine::findFrontiers_CallService(Eigen::Vector3d& uav_position)
	{
		
		frontier_srv.request.current_position.x = uav_position.x();
		frontier_srv.request.current_position.y = uav_position.y();
		frontier_srv.request.current_position.z = uav_position.z();
		frontier_srv.request.request_id = frontier_request_count;
		frontier_srv.request.new_request = new_map || first_global_request;
		bool call = find_frontiers_client.call(frontier_srv);
		if(call) 
        { 
        	has_more_goals = frontier_srv.response.success;
            if(frontier_srv.response.success)
            {
            	frontier_index = 0;
        		new_map = false;
				resetOPPair(uav_position);
				return true;
            }
            else
            {
            	// ROS_INFO_STREAM("[Goal SM] The frontier node identified no further frontiers.");
            	if(global)
            	{
            		resetOPPair(uav_position);
            		ROS_INFO_STREAM("[Goal SM] The search was conducted within the global geofence identified no further frontiers.");
            	}
            	// else
            	// 	ROS_INFO_STREAM("[Goal SM] The search was local, maybe there are frontiers still!");
            }
        } 
        else
        {
        	ROS_WARN("[Goal SM] Frontier node not accepting is frontier requests."); 
        } 
        return false;
	}

	void GoalStateMachine::saveSuccesfulFlyby()
	{
		if( getFlybyStart(success_flyby_start) )
		{
			getFlybyEnd(success_flyby_end);
		}
		else ROS_ERROR("[goal sm] There was no flyby available");
	}

	void GoalStateMachine::initLookupTable(double resolution, int tree_depth)
	{
    	LazyThetaStarOctree::fillLookupTable(resolution, tree_depth, sidelength_lookup_table); 
	}


    bool GoalStateMachine::IsOPStartReachable()
    {
		octomath::Vector3 cell_center_coordinates_start (getCurrentOPPairs().get_current_start().x(), getCurrentOPPairs().get_current_start().y(), getCurrentOPPairs().get_current_start().z());

    	double cell_size_start = -1;
		LazyThetaStarOctree::updateToCellCenterAndFindSize(cell_center_coordinates_start, *octree, cell_size_start, sidelength_lookup_table);

		LazyThetaStarOctree::unordered_set_pointers neighbors;
		LazyThetaStarOctree::generateNeighbors_filter_pointers(neighbors, cell_center_coordinates_start, cell_size_start, octree->getResolution(), *octree);
		int n_id = 1500;
		for(std::shared_ptr<octomath::Vector3> n_coordinates : neighbors)
		{
			octomap::OcTreeKey key = octree->coordToKey(*n_coordinates);
		    int depth = LazyThetaStarOctree::getNodeDepth_Octomap(key, *octree);
		    double cell_size = LazyThetaStarOctree::findSideLenght(octree->getTreeDepth(), depth, sidelength_lookup_table);

			geometry_msgs::Point neighbor_v;
			neighbor_v.x = n_coordinates->x();
			neighbor_v.y = n_coordinates->y();
			neighbor_v.z = n_coordinates->z();
			if( ! LazyThetaStarOctree::is_flight_corridor_free( LazyThetaStarOctree::InputData(*octree, cell_center_coordinates_start, *n_coordinates, path_safety_margin ), pi) )
			{
				// rviz_interface::publish_rejected_neighbor(neighbor_v, publish_input.marker_pub, marker_array_single_loop, n_id, cell_size);
				
			}
			else
			{
				// rviz_interface::publish_visible_neighbor(neighbor_v, publish_input.marker_pub, marker_array_single_loop, n_id, cell_size);
				return true;
			}
			n_id++;
		}
		csv_file << ",,,1,,,," << std::endl;
		return false;
    }

	bool GoalStateMachine::pointToNextGoal(Eigen::Vector3d& uav_position)
	{	
        Eigen::Vector3d unknown;
		while(has_more_goals)
		{
			if(hasNextFrontier())
			{
				get_current_frontier(unknown);
				bool existsNextOPPair = oppairs_side.Next();
				while(existsNextOPPair)
				{
					if( IsObservable(unknown) && IsVisible(unknown) && IsOPPairValid() && IsOPStartReachable() )
					{
						has_more_goals = true;
						saveSuccesfulFlyby();
						return true;
					}		
					existsNextOPPair = oppairs_side.Next();
				}
				is_oppairs_side = false;
				if(oppairs_under.get_frontier().z() >= geofence_min.z)
				{
					existsNextOPPair = oppairs_under.Next(); 
					while(existsNextOPPair)
					{
						if( IsObservable(unknown) && IsVisible(unknown) && IsOPPairValid() && IsOPStartReachable() )
						{
							has_more_goals = true;
							saveSuccesfulFlyby();
							return true;
						}
						existsNextOPPair = oppairs_under.Next();	 
					}
				}
			}
			if(hasNextFrontier())
			{
				resetOPPair(uav_position);
			}
			frontier_index++;
			if(!hasNextFrontier())
			{
				has_more_goals = findFrontiersAllMap(uav_position);
			}
		}
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
	    
		ROS_INFO_STREAM("[Goal SM] (" << unknown.x() << ", " << unknown.y() << ", " << unknown.z() << ") unobservable from (" << getCurrentOPPairs().get_current_start().x() << ", " << getCurrentOPPairs().get_current_start().y() << ", " << getCurrentOPPairs().get_current_start().z() << ")");
		log_file << "[Goal SM] (" << unknown.x() << ", " << unknown.y() << ", " << unknown.z() << ") unobservable from (" << getCurrentOPPairs().get_current_start().x() << ", " << getCurrentOPPairs().get_current_start().y() << ", " << getCurrentOPPairs().get_current_start().z() << ")" << std::endl;
		for (auto i = unobservable_set.begin(); i != unobservable_set.end(); ++i)
		{
			log_file << "(" << i->first.x() << ", " << i->first.y() << ", " << i->first.z() << ") - (" << i->second.x() << ", " << i->second.y() << ", " << i->second.z() << ")"	<< std::endl;
		}
		// #endif
	}

	bool GoalStateMachine::IsObservable(Eigen::Vector3d const& unknown)
	{
		return IsObservable(unknown, getCurrentOPPairs().get_current_start());
	}

	bool GoalStateMachine::IsObservable(Eigen::Vector3d const& unobservable, Eigen::Vector3d const& viewpoint)
	{
		bool is_observable =  (unobservable_set.find(std::make_pair(unobservable, viewpoint)) ==  unobservable_set.end() );
		if(!is_observable)
		{
			ROS_INFO_STREAM("[Goal] Unobservable point from this particular viewpoint");
			csv_file << "1,,,,,,," << std::endl;
		}
		return is_observable;
	}

	void GoalStateMachine::publishGoalToRviz(geometry_msgs::Point current_position)
    {
        geometry_msgs::Point frontier_geom = get_current_frontier();
        geometry_msgs::Point start_geom;
        start_geom.x = current_position.x;
        start_geom.y = current_position.y;
        start_geom.z = current_position.z;
        geometry_msgs::Point oppair_start_geom;
        getFlybyStart(oppair_start_geom);
        geometry_msgs::Point oppair_end_geom;
        getFlybyEnd(oppair_end_geom);
        visualization_msgs::MarkerArray marker_array;
        rviz_interface::build_stateManager(frontier_geom, oppair_start_geom, oppair_end_geom, start_geom, marker_array, path_safety_margin);
        pi.marker_pub.publish(marker_array);
    }

	bool GoalStateMachine::NextGoal(Eigen::Vector3d& uav_position)
	{
		ROS_WARN("[Goal] Next Goal");
		LazyThetaStarOctree::generateOffsets(octree->getResolution(), path_safety_margin, LazyThetaStarOctree::semiSphereIn, LazyThetaStarOctree::semiSphereOut );
		return pointToNextGoal(uav_position);
	}
}