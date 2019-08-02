#include <collect_data.h>
#include <neighbors.h>


namespace collect_data
{
	bool isUnknown(octomap::OcTree& octree, octomath::Vector3 candidate)
	{
		try
		{
			octomap::OcTreeKey key = octree.coordToKey(candidate); 
	        LazyThetaStarOctree::getNodeDepth_Octomap(key, octree); 
	        return false;
		}
		catch(const std::out_of_range& oor)
		{
			return true;
		}
	}
	void checkFrontiers(octomap::OcTree& octree, frontiers_msgs::FindFrontiers::Request  &request,
        frontiers_msgs::FindFrontiers::Response &reply)
	{
		// Data
		ASSERT_LE(reply.frontiers_found, static_cast<int16_t>(request.frontier_amount) );
		for (int i = 0; i < 1; ++i)
		{
			// ROS_INFO_STREAM("[" << i << "]");
			octomath::Vector3 candidate (reply.frontiers[i].xyz_m.x, reply.frontiers[i].xyz_m.y, reply.frontiers[i].xyz_m.z) ;
			ASSERT_TRUE(isUnknown(octree, candidate));
	        bool is_frontier = false;
			ASSERT_LE(reply.frontiers[i].xyz_m.x, request.max.x+octree.getResolution());
			ASSERT_LE(reply.frontiers[i].xyz_m.y, request.max.y+octree.getResolution());
			ASSERT_LE(reply.frontiers[i].xyz_m.z, request.max.z+octree.getResolution());
			ASSERT_GE(reply.frontiers[i].xyz_m.x, request.min.x-octree.getResolution());
			ASSERT_GE(reply.frontiers[i].xyz_m.y, request.min.y-octree.getResolution());
			ASSERT_GE(reply.frontiers[i].xyz_m.z, request.min.z-octree.getResolution());
		}
	}
}