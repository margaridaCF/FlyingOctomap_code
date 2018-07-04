#ifndef OPEN_H
#define OPEN_H

#include <ltStarOctree_common.h>
#include <exception>

namespace LazyThetaStarOctree{
/**
 * @brief      
 This class expects responsable use of element access. In getFromMap and pop
  when range logic is violated an std::out_of_range exception is thrown. Supports only precision to 2 decimal cases
 */
class Open
{
public:
	Open(octomath::Vector3 goal_voxel_center)
		:goal_voxel_center(goal_voxel_center)
	{

	}
	~Open()
	{
		clear();
	}
	/**
	 * @brief      Determines if a node with specified coordinates exists in map.
	 *
	 * @param      key   The coordinates
	 *
	 * @return     True if exists in map, False otherwise.
	 */
	bool existsInMap(octomath::Vector3 const& key) const
	{
		try {
			nodes.at(key);      // vector::at throws an out-of-range
			return true;
		}
		catch (const std::out_of_range& oor) {
			return false;
		}
	}

	/**
	 * @brief      Get a copy of a node on open
	 * 	An std::out_of_range exception is thrown if no element with 
	 * 	those coordinates exists.
	 *
	 * @param      key   The coordinates
	 *
	 * @return     A copy of the node in the map.
	 */
	std::shared_ptr<ThetaStarNode> getFromMap(octomath::Vector3 const& key) const
	{
		return nodes.at(key);      // vector::at throws an out-of-range
	}
	/**
	 * @brief      Insert a node into open. Replaces node if a node 
	 * with the same coordinates already exists.
	 *
	 * @param      node  The node to add
	 */
	void insert(std::shared_ptr<ThetaStarNode> node)
	{
		if(node->parentNode == NULL)
		{
			std::ostringstream oss;
			oss << "Trying to insert into open a node [" << node->coordinates << "] whose parent is NULL!";
			throw std::logic_error(oss.str());
		}
		// ROS_WARN_STREAM(*(node->coordinates));
		// printNodes("Starting insert ");
		erase(*node);
		// std::cout << "After erase " << std::endl;
		// printNodes("After erase");
		nodes[*(node->coordinates)] = node;

		double h = buildKey(*node);
		// heuristics[h] = *(node->coordinates);
		octomath::Vector3 coordinates = *(node->coordinates);
		if (heuristics.find(h) == heuristics.end())
		{
			// ROS_WARN_STREAM("No coordinates with heuristic " << h);
			std::list<octomath::Vector3> allAlone ;
			allAlone.push_back(coordinates);
			heuristics[h] = allAlone ;
			// printNodes("Righ after insert");
		}
		else
		{
			// ROS_WARN_STREAM("Already coordinates with heuristic " << h);
			heuristics[h].push_front( coordinates );
		}
		// std::cout << "After insertion " << std::endl;
		// printNodes();
		// std::cout << " --- end ---" << std::endl;
	}
	/**
	 * @brief      Test if there are nodes inside open.
	 *
	 * @return     True if there are nodes inside open, False otherwise.
	 */
	bool empty() const
	{
		return heuristics.empty();
	}
	/**
	 * @brief      Removes a node with the smallest heuristics from open and returns it.
	 *  The heuristics is the sum of the shorstest distance from the starting node found 
	 *  so far and the straight line distance to the final positions.
	 *
	 * @return     Returns a copy of the node with the smallest heuristics
	 * @exception  std::out_of_range { When open has no elements }
	 */
	std::shared_ptr<ThetaStarNode> pop()
	{
		if(heuristics.empty())
		{
			throw std::out_of_range("Open has no elements!");
		}
		// std::shared_ptr<ThetaStarNode> toReturn = nodes[heuristics.begin()->second];
		std::list <octomath::Vector3> & listOfCoordSameHeuristics = heuristics.begin()->second;
		octomath::Vector3 & coordinatesToReturn = listOfCoordSameHeuristics.back();
		std::shared_ptr<ThetaStarNode> toReturn = nodes[ coordinatesToReturn ];

		double key = heuristics.begin()->first;
		if(key != buildKey(*toReturn) )
		{
			std::ostringstream oss;
			oss << std::setprecision(10) << "Node has inconsistent heuristics. Original heuristic was " << heuristics.begin()->first << ", build it now it is " << buildKey(*toReturn) << " Please report this error to maintainer.";
			throw std::logic_error(oss.str());
		}
		// heuristics.erase(heuristics.begin());
		if (listOfCoordSameHeuristics.size() == 1)
		{
			heuristics.erase(heuristics.begin());
		}
		else
		{
			listOfCoordSameHeuristics.pop_back();
		}
		nodes.erase( coordinatesToReturn );
		return toReturn;
	}

	void printNodes(std::string title = "") const
	{
		std::cout << title << "\n - Nodes " << std::endl;
		// for(auto n : nodes)
		// {
		// 	std::cout << *(n.second->coordinates) << std::endl;
		// }

		for(auto list_pair : heuristics)
		{
			// std::cout << *(nodes.at(n.second));
			// std::cout << " ==> " << n.first << std::endl;
			std::cout << list_pair.first << " ==> ";
			for(auto coordinates : list_pair.second)
			{
				std::cout << coordinates << std::endl;
			}
		}
	}

	void printNodes(std::string title, std::ofstream & oss) const
	{
		oss << title << "\n - Nodes " << std::endl;
		for(auto list_pair : heuristics)
		{
			oss << list_pair.first << " ==> ";
			for(auto coordinates : list_pair.second)
			{
				oss << coordinates << std::endl;
			}
		}
	}
	/**
	 * @brief      Remove and destroy the node.
	 *
	 * @param      node  The node to be destroyed, identified by coordinates.
	 *
	 * @return     True if the node was found, False otherwise. 
	 * 
	 * @exception  std::out_of_range { When open has no element with those coordinates }
	 */
	bool erase(ThetaStarNode const& node)
	{
		double scale = 0.0001;
		bool neededToDelete = false;
		// printNodes("Starting erase");

		if(   existsInMap( *(node.coordinates) )   )
		{
			double old_node_key = buildKey(   *(nodes[ *(node.coordinates) ])   );
			// ROS_WARN_STREAM("Key: " << old_node_key);
			// heuristics.erase(old_node_key);
			if( heuristics.find(old_node_key) != heuristics.end() )
			{
				// printNodes("Key exists");
				// for (auto it = heuristics[old_node_key].begin();
				// 	it != heuristics[old_node_key].end();
				// 	++it)
				// {
				// 	ROS_WARN_STREAM("it " << *it);
				// }

				if( heuristics[old_node_key].size() ==1 ){
					heuristics.erase(old_node_key);
					neededToDelete = true;
					// printNodes("Deleting the whole list.");
				}
				else
				{
					std::list <octomath::Vector3> & toDeleteContainer = heuristics[old_node_key];
					for (std::list<octomath::Vector3>::iterator it = toDeleteContainer.begin();
						it != toDeleteContainer.end() && !neededToDelete;
						++it)
					{
						double distance = it->distance( *(node.coordinates) );
						if( distance <= scale)
						{
							toDeleteContainer.erase(it);
							// printNodes("Deleting only one element of the list");
							neededToDelete = true;
						}
					}
				}
			}
		}

		// std::cout << "After erase " << std::endl;
		// printNodes();
		// std::cout << " --- end ---" << std::endl;

		return neededToDelete;
	}
	int size()
	{
		// return nodes.size();
		int count = 0;
		for(auto list_pair : heuristics)
		{
			count += list_pair.second.size();
			// // std::cout << *(nodes.at(n.second));
			// // std::cout << " ==> " << n.first << std::endl;
			// std::cout << list_pair.first << " ==> ";
			// for(auto coordinates : list_pair.second)
			// {
			// 	std::cout << coordinates ;
			// }
		} 	
		return count;
	}
	void clear()
	{
		nodes.clear();
		heuristics.clear();
	}


	/**
	 * @brief      Update the distance from initial point while maintining internal ordering of nodes. 
	 * 			   This will update: the node and the node position inside open.
	 *
	 * @param[in]  new_distance  The new distance from initial position
	 *
	 * @return     true if all went well, false if the node could not be found
	 */
	bool changeDistanceFromInitialPoint(float new_distance, std::shared_ptr<ThetaStarNode> node)
	{
		if ( erase(*node) )
		{
			node->distanceFromInitialPoint = new_distance;
			insert(node);
			return true;
		}
		return false;

	}

private:
	double buildKey(ThetaStarNode const& node)
	{
		double scale = 0.0001;
		double distanceToGoalVoxelCenter = goal_voxel_center.distance( *(node.coordinates) );
		if(distanceToGoalVoxelCenter <= scale)
		{
			return 0;
		}
		else
		{
			return node.calculateH_();
		}
	}
	// this is not an unordered_map because Vector3 does not have a hash function
	std::map<octomath::Vector3, std::shared_ptr<ThetaStarNode>, VectorComparatorOrder> nodes;
	// key is a string to be able to distiguish among nodes with same heuristic value
	std::map< double, std::list<octomath::Vector3> > heuristics;
	octomath::Vector3 goal_voxel_center;
};

}


#endif // OPEN_H