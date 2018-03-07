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
		ASSERT_LE(reply.frontiers_found, 10 );

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

		checkFrontiers(octree, request, reply); 
	}
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
