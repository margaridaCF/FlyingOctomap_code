#include <gtest/gtest.h>
#include <frontiers.h>

namespace Frontiers
{
	bool isFrontier(octomap::OcTree& octree, octomath::Vector3 const&  cadidate)
	{
		bool is_frontier = isExplored(cadidate, octree);
		is_frontier = !isOccupied(cadidate, octree);
		// ToDo test neighbors
		return is_frontier;
	}


	TEST(FrontiersTest, Ask_one_frontier)
	{
		octomap::OcTree octree ("data/experimentalDataset.bt");

		frontiers_msgs::FrontierRequest request;
		request.header.seq = 1;
		request.header.frame_id = "request_frame";
		frontiers_msgs::FrontierReply reply;

		bool outcome = processFrontiersRequest(octree, request, reply);

		// MetaData
		ASSERT_EQ(request.header.seq, reply.request_id);
		ASSERT_EQ(reply.header.seq, request.header.seq+1);
		ASSERT_EQ(reply.header.frame_id, request.header.frame_id);
		// Data
		ASSERT_GE(reply.frontiers_found, 1 );
		ASSERT_TRUE(isFrontier(octree, 
			octomath::Vector3 (reply.frontiers[0].xyz_m.x, reply.frontiers[0].xyz_m.y, reply.frontiers[0].xyz_m.z) )   );

	}
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
