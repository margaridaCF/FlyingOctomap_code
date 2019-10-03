#ifndef ORDERED_NEIGHBORS_H
#define ORDERED_NEIGHBORS_H

#include <frontiers_msgs/FindFrontiers.h>
#include <cmath>
#include <set>

namespace Frontiers
{
# define M_PI       3.14159265358979323846  /* pi */

 struct VoxelMsgHash
    {
        std::size_t operator()(const frontiers_msgs::VoxelMsg & v) const 
        {
            double scale = 0.0001;
            std::size_t hx = std::hash<float>{}( (int)std::round(v.xyz_m.x / scale) * scale );
            std::size_t hy = std::hash<float>{}( (int)std::round(v.xyz_m.y / scale) * scale );
            std::size_t hz = std::hash<float>{}( (int)std::round(v.xyz_m.z / scale) * scale );

            // ROS_WARN_STREAM(v << " => " << (int)(v.xyz_m.x / scale) * scale << "  " << (int)(v.xyz_m.y / scale) * scale << "  " << (int)(v.xyz_m.z / scale) * scale);
            // ROS_WARN_STREAM(v << " => " << hx << "  " << hy << "  " << hz);


            std::size_t return_value = ((hx 
               ^ (hy << 1)) >> 1)
               ^ (hz << 1);
            // std::cout << return_value << std::endl;
            return return_value;

        }
    };

    struct VoxelMsgComparatorEqual // for unordered_map
    { 
        bool operator () (const frontiers_msgs::VoxelMsg & lhs, const frontiers_msgs::VoxelMsg & rhs) const 
        { 
            double scale = 0.0001;
            double distance = std::sqrt(   std::pow( (lhs.xyz_m.x - rhs.xyz_m.x), 2 ) + std::pow( (lhs.xyz_m.y - rhs.xyz_m.y), 2 ) + std::pow( (lhs.xyz_m.z - rhs.xyz_m.z), 2 )   );
            // ROS_INFO_STREAM("lhs (" << lhs.xyz_m.x << "," << lhs.xyz_m.y << "," << lhs.xyz_m.z << ") rhs(" << rhs.xyz_m.x << "," << rhs.xyz_m.y << "," << rhs.xyz_m.z << ") => distance " << distance );
            return distance <= scale;
            // returns !0 if the two container object keys passed as arguments are to be considered equal.
        } 
    };

    typedef std::unordered_set<frontiers_msgs::VoxelMsg, VoxelMsgHash, VoxelMsgComparatorEqual> unordered_set_voxel_msgs;


class OrderedNeighbors
{
public:
	OrderedNeighbors(frontiers_msgs::VoxelMsg const& current_position, double max_distance = 0)
		:current_position(current_position), max_distance(max_distance)
	{
	}

	~OrderedNeighbors()
	{

	}

	void insert(frontiers_msgs::VoxelMsg & new_neighbor)
	{
		new_neighbor.distance = distance(new_neighbor, current_position);
		auto  out = neighbors.insert(new_neighbor);
		if (!out.second)
		{
			if (out.first->occupied_neighborhood < new_neighbor.occupied_neighborhood)
			{
				neighbors.erase(out.first);
				neighbors.insert(new_neighbor);
			}
		}
	}

	void print() const
	{
		for (unordered_set_voxel_msgs::const_iterator i = neighbors.begin(); 
			i != neighbors.end(); 
			++i)
		{
			ROS_INFO_STREAM(*i);
		}
	}

	int buildMessageList(int frontier_amount, frontiers_msgs::FindFrontiers::Response & reply) 
	{
		int frontier_counter = 0;
		if(neighbors.size() < frontier_amount)
		{
			reply.frontiers_found = neighbors.size();
		}
		else
		{
			reply.frontiers_found = frontier_amount;
		}
		while(neighbors.size() > 0)
		{
			reply.frontiers.push_back(popHighestHeuristic());
			frontier_counter++;
		}
		return frontier_counter;
	}

	int size()
	{
		return neighbors.size();
	}	
    
    double piecewiseFunc_2d(double distance)
    {
        double N = 1080; 
        double tm = 17;
        double voxel_height = 0.5;
        double safety_distance = 2.5;
        if      (distance <= safety_distance)                    return 0;
        else if (distance < calculateDM(voxel_height, N, tm))   return tm;
        else                                                    return calculate2dHeuristics(voxel_height, N, distance);
    }


private:
	frontiers_msgs::VoxelMsg current_position;
	unordered_set_voxel_msgs neighbors;
    double max_distance;
	double distance(const frontiers_msgs::VoxelMsg & a, const frontiers_msgs::VoxelMsg & b) const
	{
	    const double x_diff = a.xyz_m.x - b.xyz_m.x;
	    const double y_diff = a.xyz_m.y - b.xyz_m.y;
	    const double z_diff = a.xyz_m.z - b.xyz_m.z;
	    return std::sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
	}

    frontiers_msgs::VoxelMsg popHighestOccupancyNeighbourhood ()
    {
        frontiers_msgs::VoxelMsg top;
        double max = -1;
        bool found = false;
        for (unordered_set_voxel_msgs::iterator i = neighbors.begin(); i != neighbors.end(); ++i)
        {
            if(i->occupied_neighborhood > max)
            {
                max = i->occupied_neighborhood;
                top = *i;
                found = true;
            }
        }
        if(!found) ROS_INFO_STREAM("Didn't find anything");
        else if(neighbors.erase(top) == 0)
        {
        	ROS_ERROR("Did not delete! ");
            ros::Duration(10).sleep();
        }
        return top;
    }
	


    frontiers_msgs::VoxelMsg popLowestDistance ()
    {
        frontiers_msgs::VoxelMsg top;
        double min = neighbors.begin()->distance;
        top = *(neighbors.begin());
        for (unordered_set_voxel_msgs::iterator i = neighbors.begin(); i != neighbors.end(); ++i)
        {
            if(i->distance < min)
            {
                min = i->distance;
                top = *i;
            }
        }
        if(neighbors.erase(top) == 0)
        {
        	ROS_ERROR("Did not delete! ");
            ros::Duration(10).sleep();
        }
        return top;
    }



    double calculateDM(double h, double N, double tm)
    {
        double top = h*N;
        double bottom = 2*M_PI*tm;
        return top/bottom;
    }

    double calculate2dHeuristics(double h, double N, double r)
    {
        double top = h*N;
        double bottom = 2*M_PI*r;
        return top/bottom;
    }


    double nearestNeighbourAndOccupied(unordered_set_voxel_msgs::iterator & i)
    {
        return (max_distance - i->distance) + i->occupied_neighborhood;
    }

    frontiers_msgs::VoxelMsg popHighestHeuristic ()
    {
        frontiers_msgs::VoxelMsg top;
        double max = -1;
        bool found = false;
        for (unordered_set_voxel_msgs::iterator i = neighbors.begin(); i != neighbors.end(); ++i)
        {
            double temp = piecewiseFunc_2d(i->distance);
            if(temp > max)
            {
                max = temp;
                top = *i;
                found = true;
            }
        }
        if(!found) ROS_ERROR_STREAM("Didn't find anything");
        else if(neighbors.erase(top) == 0)
        {
            ROS_ERROR("Did not delete! ");
            ros::Duration(10).sleep();
        }
        return top;
    }
	

};

}


#endif // ORDERED_NEIGHBORS_H