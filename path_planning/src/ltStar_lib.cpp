#include <ltStar_temp.h>

namespace std
{
    template <>
    struct hash<octomath::Vector3>
    {
        size_t operator()( const octomath::Vector3& coordinates ) const
        {
            double fractpart, intpart;
            fractpart = modf (coordinates.x()*10000 , &intpart);
            return intpart;
        }
    };
}

namespace LazyThetaStarOctree{
	// TODO The old version was using vertex, cell centers or something else? --> nobody knows....
	/// TODO figure some weights!
	float weightedDistance(octomath::Vector3 const& start, octomath::Vector3 const& end)
	{
	    return sqrt(	pow(end.x()-start.x(),2) +
						pow(end.y()-start.y(),2) +
        				pow(end.z()-start.z(),2));
	}

	bool hasLineOfSight(octomap::OcTree const& octree, octomath::Vector3 const& start, octomath::Vector3 const& end)
	{
		// There seems to be a blind spot when the very first node is occupied, so this covers that case
		octomath::Vector3 mutable_end = end;
		auto res_node = octree.search(mutable_end);
		if(res_node == NULL)
		{
			return false;
		}
		else
		{
			if (octree.isNodeOccupied(res_node) )
			{
				return false;
			}
		}
		octomath::Vector3 dummy;
		octomath::Vector3 direction = octomath::Vector3(   end.x() - start.x(),   end.y() - start.y(),   end.z() - start.z()   );
		bool has_hit_obstacle = octree.castRay( start, direction, dummy, false, direction.norm());
		if(has_hit_obstacle)
		{
			return false;
		}
		return true;
	}

	bool normalizeToVisibleEndCenter(octomap::OcTree const& octree, std::shared_ptr<octomath::Vector3> const& start, std::shared_ptr<octomath::Vector3> & end, double& cell_size)
	{
		auto res_node = octree.search(end->x(), end->y(), end->z());
		if(!res_node)
		{
			// ROS_WARN_STREAM("[1] The coordinates " << *(end) << " do not correspond to a node in this octree  ==> this neighbor is unknown");
			// continue;
			return false;
		}
		// ROS_WARN_STREAM("From  " << *end);
		updatePointerToCellCenterAndFindSize(end, octree, cell_size);
		// ROS_WARN_STREAM("to " << *end << ". Size " << cell_size);
		return hasLineOfSight(octree, *start, *end);
	}

	
	/**
	 * @brief      Set vertex portion of pseudo code, ln 34.
	 *
	 * @param      octree     The octree
	 * @param      s          TThe node in analyzis
	 * @param      closed     The closed
	 * @param      open       The open
	 * @param[in]  neighbors  The neighbors
	 */
	void setVertex(
		octomap::OcTree 										const& 	octree, 
		std::shared_ptr<ThetaStarNode> 							& 		s, 
		std::unordered_map<octomath::Vector3, std::shared_ptr<ThetaStarNode>, Vector3Hash, VectorComparatorEqual> &  closed,
		Open 													& 		open, 
		std::unordered_set<std::shared_ptr<octomath::Vector3>> 	const& 	neighbors,
		std::ofstream & log_file)	// TODO optimization where the neighbors are pruned here, will do this when it is a proper class
	{
		// // TODO == VERY IMPORTANT == this neighbors are actually the ones calculated before in the main loop so we can just pass that along instead of generating
		// // There is a choice to be made here, 
		// //  -> at this point we only care about line of sight and not about if obstacle/unknown distinction
		// //  -> passing over the neighbors is one addicional parameter that would increse the difficulty of understanding the code and one more facto to debug
		// //  -> will leave this for if implementation is not fast enough
		// ln 35 if NOT lineofsight(parent(s), s) then
		// Path 1 by considering the path from s_start to each expanded visible neighbor s′′ of s′
		if ( !hasLineOfSight(octree, *(s->coordinates), *(s->parentNode->coordinates)))
		{
			// g(s)		= length of the shortest path from the start vertex to s found so far.
			// c(s,s') 	= straight line distance between vertices s and s'	
			// ln 36 /* Path 1*/
			// ln 37 parent(s) := argmin_(s' € (nghb_vis  intersection  closed)_) evaluating ( g(s') + c(s', s) );
			// ln 38 g(s) := min_s'€ngbr_vis(s) intersection closed ( g(s') + c(s', s) );
			double min_g = std::numeric_limits<double>::max();
			std::shared_ptr<ThetaStarNode> candidate_parent;
			bool new_parent_node = false;
			if(neighbors.empty())
			{
				// ROS_WARN_STREAM("No neighbor was found.");
				return;
			}
			double cell_size;
			// each expanded & visible & neighbor of s'
			for(std::shared_ptr<octomath::Vector3> n_coordinates : neighbors)																	// NEIGHBOR
			{
				try
				{
					bool has_line_of_sight_to_neighbor_center = normalizeToVisibleEndCenter(octree, s->coordinates, n_coordinates, cell_size);	// VISIBLE
					if(!has_line_of_sight_to_neighbor_center)
					{
						continue;
					}
					// closed.at(*n_coordinates) throws exception when there is no such element
					std::shared_ptr<ThetaStarNode> neighbor_in_closed = closed.at(*n_coordinates);  											// EXPANDED
					double candidate_g = neighbor_in_closed->distanceFromInitialPoint;
					double c_distance_beetween_s_and_candidate = weightedDistance(  *(neighbor_in_closed->coordinates), *(s->coordinates)  );
					// ln 38 (...) g(s') + c(s', s)
					if(   (candidate_g + c_distance_beetween_s_and_candidate) < min_g    )
					{
						// ROS_WARN_STREAM(std::setprecision(10) << "[SetVer] " << candidate_g << " + " << c_distance_beetween_s_and_candidate << " - " << min_g 
						// 	<< " ==> " <<  (candidate_g + c_distance_beetween_s_and_candidate) - min_g << " > " << scale);
						// ROS_WARN_STREAM("[SetVer] Previous best value " << min_g << " with parent " << s->parentNode << ": " << *(s->parentNode));
						min_g = candidate_g + c_distance_beetween_s_and_candidate;
						candidate_parent = neighbor_in_closed;
						new_parent_node = true;
						// ROS_WARN_STREAM("[SetVer] " << neighbor_in_closed << "  ==>  Min_G:" << min_g << " = " << candidate_g << " + " << c_distance_beetween_s_and_candidate);
						// ROS_WARN_STREAM("[SetVer] " << neighbor_in_closed << ": " << *neighbor_in_closed << "  ==>  ");
						// ROS_WARN_STREAM("[SetVer]  Min_G:" << candidate_g << " from start to " << *(neighbor_in_closed->coordinates) );
						// ROS_WARN_STREAM("[SetVer]  c    :" << c_distance_beetween_s_and_candidate << " from " << *(neighbor_in_closed->coordinates) << " to " << *(s->coordinates)   );
					}
				}
				catch(const std::out_of_range& oor)
				{
					// ROS_WARN_STREAM("[N] " << *n_coordinates << " is unknown space.");
				} // closed.at(*n_coordinates) throws exception when there is no such element
			}
			if(new_parent_node)
			{
				// ROS_WARN_STREAM("[SetVer] For " << s << ": " << *s << " parent will now be " << candidate_parent << ": " << *candidate_parent);
				// There is a parent for path 1
				s->parentNode = candidate_parent;
				open.changeDistanceFromInitialPoint(min_g, s);
			}
			else
			{
				ROS_ERROR_STREAM("No path 1 was found, adding to closed a bad node for " << *(s->coordinates));
				// log_file << "No path 1 was found, adding to closed a bad node for " << *(s->coordinates) << std::endl;

			}
		}
		// ln 39 end
	}

	/**
	 * @brief      Extracts a sequence of coordinates from the links between nodes starting at the goal node and expanding the connections to the prevuous point through parentNode.
	 *
	 * @param      path   The list object where to store the path
	 * @param      start  The starting node
	 * @param      end    The goal node
	 *
	 * @return     true is a path from goal node until start node was found (under 500 jumps). False otherwise.
	 */
	bool extractPath(std::list<octomath::Vector3> & path, ThetaStarNode const& start, ThetaStarNode & end, bool writeToFile)
	{
		std::shared_ptr<ThetaStarNode> current = std::make_shared<ThetaStarNode>(end);
		std::shared_ptr<ThetaStarNode> parentAdd;
		int safety_count = 0;
		int max_steps_count = 50;
		while(   !( *(current->coordinates) == *(start.coordinates) ) 
			&& safety_count < max_steps_count   )
		{
			if( current->parentNode == NULL )
			{
				ROS_ERROR_STREAM("The node with no parent is " << *current);
				return false;
			}


			path.push_front( *(current->coordinates) );
			if(writeToFile)
			{
				writeToFileWaypoint(*(current->coordinates), current->cell_size, "final_path");
			}
			parentAdd = current->parentNode;
			current = parentAdd;
			safety_count++;
			if(safety_count >= max_steps_count)
			{
				ROS_ERROR_STREAM("Possible recursion in generated path detected while extracted path. Full path trunkated at node "<< max_steps_count);
				return false;
			}
		}
		path.push_front( *(current->coordinates) );
		if(writeToFile)
		{
			writeToFileWaypoint(*(current->coordinates), current->cell_size, "final_path");
		}
		return true;
	}

	/**
	 * @brief      Calcute the cost if a link between these to nodes is created. This is not the ComputeCost method of the pseudo code.
	 *
	 * @param      s            node
	 * @param      s_neighbour  The neighbour
	 *
	 * @return     the new cost
	 */
	float CalculateCost(ThetaStarNode const& s, ThetaStarNode & s_neighbour)
	{ 
		if(s.parentNode == NULL)
		{
			ROS_ERROR_STREAM("Parent node of s (" << s << ") is null. @ CalculateCost");
		}
		float g_parent_s =  s.parentNode->distanceFromInitialPoint;
		float c_s_parent_AND_s_neighbour = weightedDistance(  *(s.parentNode->coordinates), *(s_neighbour.coordinates)  );
		// ROS_WARN_STREAM("Cost: " << g_parent_s << " + " << c_parent_s_AND_s_neighbour);
		// ln 32 g(s') := g(parent(s)) + c(parent(s), s');
		return g_parent_s + c_s_parent_AND_s_neighbour;
	}


	// s' == s_neighbour
	// ln 20 UpdateVertex(s, s')
	void UpdateVertex(ThetaStarNode const& s, std::shared_ptr<ThetaStarNode> s_neighbour,
		Open & open)
	{ 
		// ROS_WARN_STREAM("UpdateVertex");
		// ln 21 g_old := g(s')
		float g_old = s_neighbour->distanceFromInitialPoint;
		// ln 29 /* Path 2 */
		// ln 30 if  g(parent(s)) + c(parent(s), s') < g(s')  then
		float cost = CalculateCost(s, *s_neighbour);
		// ROS_WARN_STREAM("if(cost < g_old) <=> " << cost << " < " << g_old);
		if(cost < g_old)
		{
			// ln 24 if s' € open then
			if(   open.existsInMap( *(s_neighbour->coordinates) )   )
			{
				// ln 25 open.Remove(s')
				open.erase(*s_neighbour);
			}
			// Unrolling ComputeCost()
			// ln 31 parent(s') := parent(s);
			s_neighbour->parentNode = s.parentNode;
			// ln 32 g(s') := g(parent(s)) + c(parent(s), s');
			s_neighbour->distanceFromInitialPoint = cost;
			// ln 26 open.Insert(s', g(s') + h(s'));
			open.insert(s_neighbour);
		}

	}

	// TODO what about a database like SQLite? Since there is the need for two data structures for open 
	// TODO 	(one ordered by heuristics and another to access by coordintades) and closed manages the same objects
	// TODO		And also would solve the problem of ownership of objects
	// h(s)		= straight line distance between goal and s vertex
	// V 		= set of all grid vertices
	// s 		= current vertice
	// c(s,s') 	= straight line distance between vertices s and s'
	// g(s)		= length of the shortest path from the start vertex to s found so far.
	// nghbrvis(s) in V 	= set of neighbors of vertex s in V that have line-of-sight to s
	/**
	 * @brief      Lazy Theta Star. Decimal precision in 0.001 both in closed and in open (buildKey)
	 *
	 * @param      octree        The octree
	 * @param      disc_initial  The disc initial
	 * @param      disc_final    The disc final
	 *
	 * @return     A list of the ordered waypoints to get from initial to final
	 */
	std::list<octomath::Vector3> lazyThetaStar_(
		octomap::OcTree   const& octree, 
		octomath::Vector3 const& disc_initial, 
		octomath::Vector3 const& disc_final,
		ResultSet & resultSet,
		int const& max_search_iterations,
		bool print_resulting_path)
	{
		// GOAL
		// Init initial and final nodes to have the coordinates of respective cell centers
		double cell_size_goal = -1;
		octomath::Vector3 cell_center_coordinates_goal = disc_final;
		updateToCellCenterAndFindSize( cell_center_coordinates_goal, octree, cell_size_goal);
		std::shared_ptr<ThetaStarNode> disc_final_cell_center = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3>(cell_center_coordinates_goal), cell_size_goal);
		disc_final_cell_center->lineDistanceToFinalPoint = weightedDistance(cell_center_coordinates_goal, disc_final);
		// START
		// Get distance from start to cell center
		octomath::Vector3 cell_center_coordinates_start = disc_initial;
		double cell_size_start = -1;
		updateToCellCenterAndFindSize(cell_center_coordinates_start, octree, cell_size_start);


		// ln 1 Main()
		// ln 2 open := closed := 0
		Open open (cell_center_coordinates_goal);
		std::unordered_map<octomath::Vector3, std::shared_ptr<ThetaStarNode>, Vector3Hash, VectorComparatorEqual> closed;

		// M the starting point is the initial node
		// [Footnote] open.Insert(s,x) inserts vertex s with key x into open
		// map open's order = (shortest path from the start vertex to s found so far) + (straight line distance between goal and vertex)
		std::shared_ptr<ThetaStarNode> disc_initial_cell_center = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3>(cell_center_coordinates_start), cell_size_start, 
    		// ln 3 g(s_start) := 0;
			0, 
			weightedDistance(cell_center_coordinates_start, disc_final));
	    // ln 4 parent(s_start) := s_start;
		disc_initial_cell_center->parentNode = disc_initial_cell_center;	// TODO make sure nobody is realing on point pointing to themselves in general
		// ln 5 open.Insert(s_start, g(s_start) + h(s_start))
		open.insert(disc_initial_cell_center);
		// ROS_WARN_STREAM("__START__ " << disc_initial_cell_center << ": " << *disc_initial_cell_center);
		std::shared_ptr<ThetaStarNode> s_neighbour, s, solution_end_node;
		bool solution_found = false;
		double resolution = octree.getResolution();
		// TODO remove this, for debugging only
		int used_search_iterations = 0;
		std::ofstream log_file;
    	log_file.open("/home/mfaria/out.log", std::ios_base::app);
		// ROS_WARN_STREAM("Goal's voxel center " << *disc_final_cell_center);
		// ln 6 while open != empty do
		while(!open.empty() && !solution_found)
		{
			// open.printNodes("========= Starting state of open ");
			// ln 7 s := open.Pop();	
			// [Footnote] open.Pop() removes a vertex with the smallest key from open and returns it
			if(print_resulting_path)
			{
				open.printNodes(" ========== Before pop ========== ", log_file);
			}
			s = open.pop();
			resultSet.addOcurrance(s->cell_size);
			std::unordered_set<std::shared_ptr<octomath::Vector3>> neighbors;
			generateNeighbors_pointers(neighbors, *(s->coordinates), s->cell_size, resolution);
			// ln 8 SetVertex(s);
			// It is it's own parent, this happens on the first node when the initial position is the center of the voxel (by chance)
			if(s->hasSameCoordinates(s->parentNode, octree.getResolution()) == false)
			{
				setVertex(octree, s, closed, open, neighbors, log_file); 
			}
			// ln 9 if s = s_goal then 
			if( s->hasSameCoordinates(disc_final_cell_center, octree.getResolution()) )
			{
				// ln 10 return "path found"
				solution_found = true;
				solution_end_node = s;
				// ROS_WARN_STREAM( "Solution end node:" << s << ": " << *s << ": " << *(s->parentNode->coordinates) );
				continue;
			}
			// ln 11 closed := closed U {s}
			// ROS_WARN_STREAM("@"<< used_search_iterations << "  inserting s into closed " << s << " <--> " << *s);
			closed.insert( std::pair<octomath::Vector3, std::shared_ptr<ThetaStarNode>>( *(s->coordinates), s));
			if(print_resulting_path)
			{
				log_file << "@"<< used_search_iterations << "  inserting s into closed " << s << " <--> " << *s << std::endl;
				// writeToFileWaypoint(*(s->coordinates), s->cell_size, "closed");
			}

			// TODO check code repetition to go over the neighbors of s
			double cell_size = 0;
			// ln 12 foreach s' € nghbr_vis(s) do
			for(std::shared_ptr<octomath::Vector3> n_coordinates : neighbors)
			{
				// ROS_WARN_STREAM("@"<< used_search_iterations << "  Analyzing neighbor " << *n_coordinates);
				// ROS_WARN_STREAM("Existing Node objects " << ThetaStarNode::OustandingObjects());
				// Find minimum value for those with visibility and that it is in closed
				// TODO for neighbor pointd that belong to the same cell, the  calculations are duplicated it shouldn't be too hard to optimize this to skip all subsequent calculations (with a list of something)
				// TODO This assumes that line of sight between cell centers is enough
				// TODO If cell big, the edges of the UAV might need to be taken into account 
				// TODO resulting in two line of sight checks (edges instead of one (center)
				if(!normalizeToVisibleEndCenter(octree, s->coordinates, n_coordinates, cell_size))
				{
					// auto res_node = octree.search(*n_coordinates);
					// if(res_node == NULL)
					// {
     				// 	throw std::out_of_range("Skipping cases where unknown neighbors are found.");
					// }
					continue;
				}
				// ln 13 if s' !€ closed then
				bool is_neighbor_in_closed = closed.find(*n_coordinates) != closed.end();
				if (!is_neighbor_in_closed)
				{
					// ln 14 if s' !€ open then
					if( !open.existsInMap(*n_coordinates) )
					{
						// ln 15 g(s') := infinity;
						double g_distanceFromInitialPoint = std::numeric_limits<double>::max();
						// ln 16 parent(s') := NULL;
						s_neighbour = std::make_shared<ThetaStarNode>(
							n_coordinates, 
							cell_size, 
							g_distanceFromInitialPoint, 
							weightedDistance( *(n_coordinates), disc_final) // lineDistanceToFinalPoint
							);
						// TODO are we supposed to add to open? --> guess UpdateVertex takes care of that
					}
					else
					{
						s_neighbour =  open.getFromMap(*n_coordinates);
					}
					// ln 17 UpdateVertex(s, s');
					UpdateVertex(*s, s_neighbour, open);
				}
				// else
				// {
				// 	ROS_WARN_STREAM("[N] " << *n_coordinates << " is already in closed");
				// }
			}
			used_search_iterations++;	
			if(used_search_iterations > max_search_iterations)
			{
				ROS_ERROR_STREAM("Reached maximum iterations of A*. Breaking out");
				break;	
			}
			// if(used_search_iterations % 1000 == 0)
			// {
			// 	ROS_WARN_STREAM("Used "<<used_search_iterations<<" iterations.");
			// }
		}
		resultSet.iterations_used = used_search_iterations;
		// ROS_WARN_STREAM("Used "<< used_search_iterations << " iterations to find path");
		// ln 18 return "no path found";
		std::list<octomath::Vector3> path;
		if(!solution_found)
		{
			// ROS_ERROR_STREAM("No solution found. Giving empty path.");
		}
		else
		{ 
			// for( auto it = closed.begin(); it != closed.end(); ++it)
			// {
			// 	ROS_WARN_STREAM( "Closed node: " << it->second << ": " << *(it->second) << ": " << *(it->second->parentNode) );
			// }
			// return the found path 
			extractPath(path, *disc_initial_cell_center, *solution_end_node, print_resulting_path);
		}
		log_file.close();
		// Free all nodes in both open and closed
		open.clear();
		closed.clear();
		disc_initial_cell_center->parentNode = NULL;
		disc_initial_cell_center = NULL;
		// TODO Add path from starting point to center of the first cell
		return path;
	}
	// ln 19 end

	bool processLTStarRequest(octomap::OcTree const& octree, path_planning_msgs::LTStarRequest const& request, path_planning_msgs::LTStarReply & reply)
	{
		ResultSet statistical_data;
		octomath::Vector3 disc_initial(request.start.x, request.start.y, request.start.z);
		octomath::Vector3 disc_final(request.goal.x, request.goal.y, request.goal.z);
		ROS_INFO_STREAM("[LTStar] Starting to process path from " << disc_initial << " to " << disc_final);
		std::list<octomath::Vector3> resulting_path = lazyThetaStar_(octree, disc_initial, disc_final, statistical_data, request.max_search_iterations);
		ROS_INFO_STREAM("[LTStar] FINISHED! path from " << disc_initial << " to " << disc_final << ". Outcome with " << resulting_path.size() << " waypoints.");
		if(resulting_path.size()==0)
		{
			reply.success = false;
		}
		else
		{
			for (std::list<octomath::Vector3>::iterator i = resulting_path.begin(); i != resulting_path.end(); ++i)
			{
				ROS_INFO_STREAM(*i);
				geometry_msgs::Point waypoint;
	            waypoint.x = i->x();
	            waypoint.y = i->y();
	            waypoint.z = i->z();
	            reply.waypoints.push_back(waypoint);
			}
			reply.success = true;
		}
		reply.waypoint_amount = resulting_path.size();
		reply.request_id = request.header.seq;
		return true;
	}
}