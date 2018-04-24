#include <gtest/gtest.h>
#include <frontiers.h>
#include <neighbors.h>

namespace Frontiers
{
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
		ros::Publisher marker_pub;
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
		bool outcome = processFrontiersRequest(octree, request, reply, marker_pub, false);
		checkFrontiers(octree, request, reply);
	}


	TEST(FrontiersTest, Ask_ten_frontiers)
	{
		ros::Publisher marker_pub;
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
		bool outcome = processFrontiersRequest(octree, request, reply, marker_pub, false);
		ASSERT_EQ(reply.frontiers_found, static_cast<int16_t>(request.frontier_amount) );
		checkFrontiers(octree, request, reply); 
	}

	TEST(FrontiersTest, Ask_many_frontiers_push_bounderies)
	{
		ros::Publisher marker_pub;
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
		bool outcome = processFrontiersRequest(octree, request, reply, marker_pub, false);
		ASSERT_EQ(reply.frontiers_found, static_cast<int16_t>(request.frontier_amount) );
		checkFrontiers(octree, request, reply); 
	}

	TEST(FrontiersTest, No_frontiers)
	{
		ros::Publisher marker_pub;
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
		bool outcome = processFrontiersRequest(octree, request, reply, marker_pub, false);
		ASSERT_EQ(reply.frontiers_found, 0 );
		checkFrontiers(octree, request, reply); 
	}


	TEST(FrontiersTest, Test_is_frontiers_on_unknown)
	{
		octomap::OcTree octree ("data/experimentalDataset.bt");

		ASSERT_TRUE(   isFrontier( octree, octomath::Vector3 (0, 1, 1.85) )   );
		ASSERT_FALSE(   isFrontier( octree, octomath::Vector3 (1.50, 0.5, 0) )   );
	}

	TEST(FrontiersTest, Test_no_frontiers_velodyne) // failing - did not solve yet
	{
		octomap::OcTree octree ("data/octree_noFrontiers_velodyne.bt");
		ros::Publisher marker_pub;

		frontiers_msgs::FrontierRequest request;
		request.header.seq = 5;
		geometry_msgs::Point min;
		min.x = -15;
		min.y = -20;
		min.z = 1;
		request.min = min;
		geometry_msgs::Point max;
		max.x = 12;
		max.y = 20;
		max.z = 4;
		request.max = max;
		geometry_msgs::Point pos;
		pos.x = -8.66228;
		pos.y = 17.2551;
		pos.z = 2.86449;
		request.current_position = pos;
		request.frontier_amount = 1;
		request.min_distance = 0.5;
		request.safety_margin = 3;
		frontiers_msgs::FrontierReply reply;
		bool outcome = processFrontiersRequest(octree, request, reply, marker_pub, false);
		ASSERT_EQ(1, reply.frontiers_found);
	}


	TEST(FrontiersTest, Test_no_frontiers_hokuyo)
	{
		octomap::OcTree octree ("data/octree_noFrontiers_hokuyo.bt");
		ros::Publisher marker_pub;

		frontiers_msgs::FrontierRequest request;
		request.header.seq = 13;
		geometry_msgs::Point min;
		min.x = -15;
		min.y = -20;
		min.z = 1;
		request.min = min;
		geometry_msgs::Point max;
		max.x = 12;
		max.y = 20;
		max.z = 4;
		request.max = max;
		geometry_msgs::Point pos;
		pos.x = -4.27578;
		pos.y = 16.5513;
		pos.z = 3.49062;
		request.current_position = pos;
		request.frontier_amount = 1;
		request.min_distance = 0.5;
		request.safety_margin = 3;
		frontiers_msgs::FrontierReply reply;
		bool outcome = processFrontiersRequest(octree, request, reply, marker_pub, false);
		ASSERT_EQ(1, reply.frontiers_found);
	}
	
	TEST(WIPFrontiersTest, Test_isCenterGoodGoal_false)
	{
		double voxel_side = 30;
		double sensing_distance = 10;
		ASSERT_FALSE(isCenterGoodGoal(voxel_side, sensing_distance));
	}


	TEST(WIPFrontiersTest, Test_isCenterGoodGoal_true)
	{
		double voxel_side = 4;
		double sensing_distance = 10;
		ASSERT_TRUE(isCenterGoodGoal(voxel_side, sensing_distance));
	}


	TEST(WIPFrontiersTest, Test_isCenterGoodGoal_border)
	{
		double voxel_side = 10;
		double sensing_distance = 10;
		ASSERT_TRUE(isCenterGoodGoal(voxel_side, sensing_distance));
	}
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
