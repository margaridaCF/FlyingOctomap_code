#ifndef FRONTIERS_H
#define FRONTIERS_H

#include <frontiers_common.h>

namespace Frontiers{

	class Circulator
	{
		bool is_finished;
		octomap::OcTree::leaf_bbx_iterator octree_it, octree_end, octree_begin;
		int count_from_beginning, starting_index;
	public:
		Circulator();
		Circulator(octomap::OcTree const& octree, octomath::Vector3  max, octomath::Vector3  min, int start);
		~Circulator(){}

		void increment();
		bool isFinished();
		octomath::Vector3 getCoordinate();
		double getSize();
		int getCounter();
	};
	
	Circulator::Circulator()
	:count_from_beginning(0), is_finished(true)
	{}

	Circulator::Circulator(octomap::OcTree const& octree, octomath::Vector3  max, octomath::Vector3  min, int start)
	:count_from_beginning(0), is_finished(false), starting_index(0)
	{
        octomap::OcTreeKey bbxMinKey, bbxMaxKey;
        if(!octree.coordToKeyChecked(min, bbxMinKey) || !octree.coordToKeyChecked(max, bbxMaxKey))
        {
            ROS_ERROR_STREAM("[Frontiers] Problems with the octree");
        }
        octree_it = octree.begin_leafs_bbx(bbxMinKey,bbxMaxKey);
        octree_begin = octree_it;
        octree_end = octree.end_leafs_bbx();
        for (int i = 0; i < count_from_beginning; ++i)
        {
        	octree_it++;
        	starting_index++;
	        if(octree_it == octree.end_leafs_bbx())
	    	{
	    		octree_it = octree_begin;
	    		starting_index = 0;
	    	}
        }
	}
	
	int Circulator::getCounter()
	{
		return count_from_beginning;
	}

	void Circulator::increment()
	{
		// ROS_INFO_STREAM("[Circulator] count_from_beginning: " << count_from_beginning);
		octree_it++;
		count_from_beginning++;
		if(octree_it == octree_end)
		{
			ROS_INFO("Reached end of octomap iterator");
			octree_it = octree_begin;
			count_from_beginning = 0;
		}
		if(count_from_beginning == starting_index) 
		{
			ROS_WARN_STREAM("[Circulator] Iterated the whole space. " );
			is_finished = true;
		}
	}
	bool Circulator::isFinished()
	{

		return is_finished;
	}

	octomath::Vector3 Circulator::getCoordinate()
	{
		return octree_it.getCoordinate();
	}

	double Circulator::getSize()
	{
		return octree_it.getSize();
	}

    Circulator  processFrontiersRequest(octomap::OcTree const& octree, frontiers_msgs::FindFrontiers::Request  &request,
        frontiers_msgs::FindFrontiers::Response &reply, ros::Publisher const& marker_pub, bool publish = true);
    bool isOccupied(octomath::Vector3 const& grid_coordinates_toTest, octomap::OcTree const& octree);
    bool isExplored(octomath::Vector3 const& grid_coordinates_toTest, octomap::OcTree const& octree);
    bool isFrontier(octomap::OcTree& octree, octomath::Vector3 const&  candidate); 
    void searchFrontier(octomap::OcTree const& octree, Circulator & it, frontiers_msgs::FindFrontiers::Request  &request,
        frontiers_msgs::FindFrontiers::Response &reply, ros::Publisher const& marker_pub, bool publish);
    void searchFrontier_optimized(octomap::OcTree const& octree, Circulator & it, frontiers_msgs::FindFrontiers::Request  &request,
        frontiers_msgs::FindFrontiers::Response &reply, ros::Publisher const& marker_pub, bool publish);

    typedef void (*search_function)(octomap::OcTree const& octree, Circulator & it, frontiers_msgs::FindFrontiers::Request  &request, frontiers_msgs::FindFrontiers::Response &reply, ros::Publisher const& marker_pub, bool publish);

}
#endif // FRONTIERS_H
