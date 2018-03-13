#include <gtest/gtest.h>
#include <frontiers.h>
#include <neighbors.h>

namespace Frontiers
{
	bool isFrontier(octomap::OcTree& octree, octomath::Vector3 const&  candidate)
	{
		bool is_frontier = isExplored(candidate, octree);
		is_frontier = !isOccupied(candidate, octree);
		// ToDo test neighbors
		std::unordered_set<std::shared_ptr<octomath::Vector3>> neighbors;
		double resolution = octree.getResolution();
		int tree_depth = octree.getTreeDepth();
		octomap::OcTreeKey key = octree.coordToKey(candidate);
		int depth = LazyThetaStarOctree::getNodeDepth_Octomap(key, octree);
		double voxel_size = ((tree_depth + 1) - depth) * resolution;
        LazyThetaStarOctree::generateNeighbors_pointers(neighbors, candidate, voxel_size, resolution);
        bool hasUnExploredNeighbors = false;
        for(std::shared_ptr<octomath::Vector3> n_coordinates : neighbors)
        {
            if(!isOccupied(*n_coordinates, octree))
            {
                hasUnExploredNeighbors = !isExplored(*n_coordinates, octree) || hasUnExploredNeighbors;
            }
        }
        if(hasUnExploredNeighbors)
        {
            return true;
        }
        else
        {
			return false;
        }
	}


	// experimentalDataset.bt
	// [ INFO] [1520351195.338686653]: OcTree min 
	// x: -8.9
	// y: -6.4
	// z: -0.5
	// [ INFO] [1520351195.338760768]: OcTree max 
	// x: 5
	// y: 8.2
	// z: 2

	void checkFrontiers(octomap::OcTree& octree, frontiers_msgs::FrontierRequest& request, frontiers_msgs::FrontierReply& reply)
	{
		// MetaData
		ASSERT_EQ(request.header.seq, reply.request_id);
		ASSERT_EQ(reply.header.seq, request.header.seq+1);
		ASSERT_EQ(reply.header.frame_id, request.header.frame_id);
		// Data
		ASSERT_LE(reply.frontiers_found, static_cast<int16_t>(request.frontier_amount) );
		for (int i = 0; i < reply.frontiers_found; ++i)
		{
			ASSERT_TRUE(isFrontier(octree, octomath::Vector3 (reply.frontiers[i].xyz_m.x, reply.frontiers[i].xyz_m.y, reply.frontiers[i].xyz_m.z) )   );
			ASSERT_LE(reply.frontiers[i].xyz_m.x, request.max.x);
			ASSERT_LE(reply.frontiers[i].xyz_m.y, request.max.y);
			ASSERT_LE(reply.frontiers[i].xyz_m.z, request.max.z);
			ASSERT_GE(reply.frontiers[i].xyz_m.x, request.min.x);
			ASSERT_GE(reply.frontiers[i].xyz_m.y, request.min.y);
			ASSERT_GE(reply.frontiers[i].xyz_m.z, request.min.z);
		}
	}


	TEST(FrontiersTest, Ask_one_frontier)
	{
		octomap::OcTree octree ("data/experimentalDataset.bt");
		frontiers_msgs::FrontierRequest request;
		request.header.seq = 1;
		request.header.frame_id = "request_frame";
		request.min.x = 0;
		request.min.y = 0;
		request.min.z = 0;
		request.max.x = 6;
		request.max.y = 2;
		request.max.z = 2;
		request.frontier_amount = 1;
		frontiers_msgs::FrontierReply reply;
		bool outcome = processFrontiersRequest(octree, request, reply);
		checkFrontiers(octree, request, reply);
	}


	TEST(FrontiersTest, Ask_ten_frontiers)
	{
		octomap::OcTree octree ("data/experimentalDataset.bt");
		frontiers_msgs::FrontierRequest request;
		request.header.seq = 1;
		request.header.frame_id = "request_frame";
		request.min.x = 0;
		request.min.y = 0;
		request.min.z = 0;
		request.max.x = 6;
		request.max.y = 2;
		request.max.z = 2;
		request.frontier_amount = 10;
		frontiers_msgs::FrontierReply reply;
		bool outcome = processFrontiersRequest(octree, request, reply);
		ASSERT_EQ(reply.frontiers_found, static_cast<int16_t>(request.frontier_amount) );
		checkFrontiers(octree, request, reply); 
	}

	TEST(FrontiersTest, Ask_many_frontiers_push_bounderies)
	{
		octomap::OcTree octree ("data/experimentalDataset.bt");
		frontiers_msgs::FrontierRequest request;
		request.header.seq = 1;
		request.header.frame_id = "request_frame";
		request.min.x = 0;
		request.min.y = 0;
		request.min.z = 0;
		request.max.x = 6;
		request.max.y = 2;
		request.max.z = 2;
		request.frontier_amount = 127;
		frontiers_msgs::FrontierReply reply;
		bool outcome = processFrontiersRequest(octree, request, reply);
		ASSERT_EQ(reply.frontiers_found, static_cast<int16_t>(request.frontier_amount) );
		checkFrontiers(octree, request, reply); 
	}

	TEST(FrontiersTest, No_frontiers)
	{
		octomap::OcTree octree ("data/experimentalDataset.bt");
		frontiers_msgs::FrontierRequest request;
		request.header.seq = 1;
		request.header.frame_id = "request_frame";
		request.min.x = 0;
		request.min.y = 0;
		request.min.z = 0;
		request.max.x = 1;
		request.max.y = 1;
		request.max.z = 1;
		request.frontier_amount = 127;
		frontiers_msgs::FrontierReply reply;
		bool outcome = processFrontiersRequest(octree, request, reply);
		ASSERT_EQ(reply.frontiers_found, 0 );
		checkFrontiers(octree, request, reply); 
	}
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
