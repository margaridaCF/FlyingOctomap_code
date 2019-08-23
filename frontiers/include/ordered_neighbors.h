#ifndef ORDERED_NEIGHBORS_H
#define ORDERED_NEIGHBORS_H

#include <frontiers_msgs/FrontierReply.h>
#include <cmath>
#include <set>

namespace Frontiers
{

struct compare {
    bool operator() (const frontiers_msgs::VoxelMsg& lhs, const frontiers_msgs::VoxelMsg& rhs) const {
        return (lhs.distance) < (rhs.distance);
    }
};

class OrderedNeighbors
{
public:
	OrderedNeighbors(frontiers_msgs::VoxelMsg const& current_position)
		:current_position(current_position)
	{
	}

	~OrderedNeighbors()
	{

	}

	void insert(frontiers_msgs::VoxelMsg & new_neighbor)
	{
		new_neighbor.distance = distance(new_neighbor, current_position);
		neighbors.insert(neighbors.begin(), new_neighbor);
	}

	void print() const
	{
		for (std::set <frontiers_msgs::VoxelMsg, compare>::iterator i = neighbors.begin(); 
			i != neighbors.end(); 
			++i)
		{
			ROS_INFO_STREAM(*i);
		}
	}

	int buildMessageList(int frontier_amount, frontiers_msgs::FindFrontiers::Response & reply) const
	{
		int frontier_counter = 0;
		for (std::set <frontiers_msgs::VoxelMsg, compare>::iterator i = neighbors.begin(); 
			i != neighbors.end() && frontier_counter < frontier_amount; 
			++i)
		{
			reply.frontiers.push_back(*i);
			frontier_counter++;
		}
		if(neighbors.size() < frontier_amount)
		{
			reply.frontiers_found = neighbors.size();
		}
		else
		{
			reply.frontiers_found = frontier_amount;
		}
		return frontier_counter;
	}	
	
private:
	frontiers_msgs::VoxelMsg current_position;
	std::set <frontiers_msgs::VoxelMsg, compare> neighbors;
	double distance(const frontiers_msgs::VoxelMsg & a, const frontiers_msgs::VoxelMsg & b) const
	{
	    const double x_diff = a.xyz_m.x - b.xyz_m.x;
	    const double y_diff = a.xyz_m.y - b.xyz_m.y;
	    const double z_diff = a.xyz_m.z - b.xyz_m.z;
	    return std::sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
	}

};

}


#endif // ORDERED_NEIGHBORS_H