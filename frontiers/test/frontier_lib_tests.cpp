#include <gtest/gtest.h>
#include <frontiers.h>
#include <neighbors.h>

namespace Frontiers
{
	// This might be affected because of blind spot calculation, put in 90º angle which is the closest to no blind spot
	void checkFrontiers(octomap::OcTree& octree, frontiers_msgs::FindFrontiers::Request  &request,
        frontiers_msgs::FindFrontiers::Response &reply)
	{
		// Data
		ASSERT_LE(reply.frontiers_found, static_cast<int16_t>(request.frontier_amount) );
		for (int i = 0; i < reply.frontiers_found; ++i)
		{
			// ASSERT_TRUE(isFrontier(octree, octomath::Vector3 (reply.frontiers[i].xyz_m.x, reply.frontiers[i].xyz_m.y, reply.frontiers[i].xyz_m.z), 1.5708 )   );
			ASSERT_LE(reply.frontiers[i].xyz_m.x, request.max.x+octree.getResolution());
			ASSERT_LE(reply.frontiers[i].xyz_m.y, request.max.y+octree.getResolution());
			ASSERT_LE(reply.frontiers[i].xyz_m.z, request.max.z+octree.getResolution());
			ASSERT_GE(reply.frontiers[i].xyz_m.x, request.min.x-octree.getResolution());
			ASSERT_GE(reply.frontiers[i].xyz_m.y, request.min.y-octree.getResolution());
			ASSERT_GE(reply.frontiers[i].xyz_m.z, request.min.z-octree.getResolution());
		}
	}


	TEST(FrontiersTest, Ask_one_frontier)
	{
		ros::Publisher marker_pub;
		octomap::OcTree octree ("data/experimentalDataset.bt");
		frontiers_msgs::FindFrontiers find_frontiers_msg;
		find_frontiers_msg.request.min.x = 0;
		find_frontiers_msg.request.min.y = 0;
		find_frontiers_msg.request.min.z = 0;
		find_frontiers_msg.request.max.x = 6;
		find_frontiers_msg.request.max.y = 2;
		find_frontiers_msg.request.max.z = 2;
		find_frontiers_msg.request.frontier_amount = 1;
		processFrontiersRequest(octree, find_frontiers_msg.request, find_frontiers_msg.response, marker_pub, false);
		checkFrontiers(octree, find_frontiers_msg.request, find_frontiers_msg.response);
	}


	TEST(FrontiersTest, Ask_ten_frontiers_calculated )
	{
		ros::Publisher marker_pub;
		octomap::OcTree octree ("data/experimentalDataset.bt");
		frontiers_msgs::FindFrontiers find_frontiers_msg;
		find_frontiers_msg.request.min.x = 2.7;
		find_frontiers_msg.request.min.y = 0.1;
		find_frontiers_msg.request.min.z = 0.3;
		find_frontiers_msg.request.max.x = 3.2;
		find_frontiers_msg.request.max.y = 0.4;
		find_frontiers_msg.request.max.z = 0.7;
		find_frontiers_msg.request.frontier_amount = 8;
		processFrontiersRequest(octree, find_frontiers_msg.request, find_frontiers_msg.response, marker_pub, false);
		checkFrontiers(octree, find_frontiers_msg.request, find_frontiers_msg.response);
	}

	TEST(FrontiersTest, Ask_ten_frontiers)
	{
		ros::Publisher marker_pub;
		octomap::OcTree octree ("data/experimentalDataset.bt");
		frontiers_msgs::FindFrontiers find_frontiers_msg;
		find_frontiers_msg.request.min.x = 0;
		find_frontiers_msg.request.min.y = 0;
		find_frontiers_msg.request.min.z = 0;
		find_frontiers_msg.request.max.x = 6;
		find_frontiers_msg.request.max.y = 2;
		find_frontiers_msg.request.max.z = 2;
		find_frontiers_msg.request.frontier_amount = 10;
		processFrontiersRequest(octree, find_frontiers_msg.request, find_frontiers_msg.response, marker_pub, false);
		ASSERT_EQ(find_frontiers_msg.response.frontiers_found, static_cast<int16_t>(find_frontiers_msg.request.frontier_amount) );
		checkFrontiers(octree, find_frontiers_msg.request, find_frontiers_msg.response); 
	}

	TEST(FrontiersTest, Ask_many_frontiers_push_bounderies)
	{
		ros::Publisher marker_pub;
		octomap::OcTree octree ("data/experimentalDataset.bt");
		frontiers_msgs::FindFrontiers find_frontiers_msg;
		find_frontiers_msg.request.min.x = 0;
		find_frontiers_msg.request.min.y = 0;
		find_frontiers_msg.request.min.z = 0;
		find_frontiers_msg.request.max.x = 6;
		find_frontiers_msg.request.max.y = 2;
		find_frontiers_msg.request.max.z = 2;
		find_frontiers_msg.request.frontier_amount = 127;
		processFrontiersRequest(octree, find_frontiers_msg.request, find_frontiers_msg.response, marker_pub, false);
		ASSERT_EQ(find_frontiers_msg.response.frontiers_found, static_cast<int16_t>(find_frontiers_msg.request.frontier_amount) );
		checkFrontiers(octree, find_frontiers_msg.request, find_frontiers_msg.response); 
	}

	TEST(FrontiersTest, No_frontiers)
	{
		ros::Publisher marker_pub;
		octomap::OcTree octree ("data/experimentalDataset.bt");
		frontiers_msgs::FindFrontiers find_frontiers_msg;
		find_frontiers_msg.request.min.x = 0;
		find_frontiers_msg.request.min.y = 0;
		find_frontiers_msg.request.min.z = 0;
		find_frontiers_msg.request.max.x = 1;
		find_frontiers_msg.request.max.y = 1;
		find_frontiers_msg.request.max.z = 1;
		find_frontiers_msg.request.frontier_amount = 127;
		processFrontiersRequest(octree, find_frontiers_msg.request, find_frontiers_msg.response, marker_pub, false);
		ASSERT_EQ(find_frontiers_msg.response.frontiers_found, 0 );
		checkFrontiers(octree, find_frontiers_msg.request, find_frontiers_msg.response); 
	}


	TEST(FrontiersTest, Test_is_frontiers_on_unknown)	// This might be affected due to blind spot calculation, put in 90º angle which is the closest to no blind spot
	{
		octomap::OcTree octree ("data/experimentalDataset.bt");

		ASSERT_TRUE(   isFrontier( octree, octomath::Vector3 (0, 1, 1.85) )   );
		ASSERT_FALSE(   isFrontier( octree, octomath::Vector3 (1.50, 0.5, 0))   );
	}
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
