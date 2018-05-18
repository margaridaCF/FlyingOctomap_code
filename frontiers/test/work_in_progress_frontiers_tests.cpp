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
			ASSERT_LE(reply.frontiers[i].xyz_m.x, request.max.x+octree.getResolution());
			ASSERT_LE(reply.frontiers[i].xyz_m.y, request.max.y+octree.getResolution());
			ASSERT_LE(reply.frontiers[i].xyz_m.z, request.max.z+octree.getResolution());
			ASSERT_GE(reply.frontiers[i].xyz_m.x, request.min.x-octree.getResolution());
			ASSERT_GE(reply.frontiers[i].xyz_m.y, request.min.y-octree.getResolution());
			ASSERT_GE(reply.frontiers[i].xyz_m.z, request.min.z-octree.getResolution());
		}
	}
    
	TEST(WIPFrontiersTest, Test_frontierAmount_NeighborToFarToSense)
	{
		// -10_-18_6__8__frontierTooBig.bt
		octomath::Vector3 frontier (-10,-18,6);
		ros::Publisher marker_pub;
		octomap::OcTree octree ("data/-10_-18_6__8__frontierTooBig.bt");
		frontiers_msgs::FrontierRequest request;
		request.header.seq = 1;
		request.header.frame_id = "request_frame";
		request.min.x = -15;
		request.min.y = -20;
		request.min.z = 1;
		request.max.x = 12;
		request.max.y = 20;
		request.max.z = 14;
		request.frontier_amount = 1;
		request.sensing_distance = 2;
		frontiers_msgs::FrontierReply reply;
		bool outcome = processFrontiersRequest(octree, request, reply, marker_pub, false);
		checkFrontiers(octree, request, reply);
		// ROS_INFO_STREAM(reply);
	}

	TEST(WIPFrontiersTest, Test_frontierAmount_1)
	{
		// -10_-18_6__8__frontierTooBig.bt
		octomath::Vector3 frontier (-10,-18,6);
		ros::Publisher marker_pub;
		octomap::OcTree octree ("data/-10_-18_6__8__frontierTooBig.bt");
		frontiers_msgs::FrontierRequest request;
		request.header.seq = 1;
		request.header.frame_id = "request_frame";
		request.min.x = -15;
		request.min.y = -20;
		request.min.z = 1;
		request.max.x = 12;
		request.max.y = 20;
		request.max.z = 14;
		request.frontier_amount = 1;
		request.sensing_distance = 4;
		frontiers_msgs::FrontierReply reply;
		bool outcome = processFrontiersRequest(octree, request, reply, marker_pub, false);
		ASSERT_EQ(reply.frontiers_found, 1);
		// ROS_INFO_STREAM(reply);
		checkFrontiers(octree, request, reply);
		double diff = std::abs(reply.frontiers[0].xyz_m.x - (-14));
		ASSERT_LE(diff, 0.1) << reply.frontiers[0].xyz_m.x << " and " << -14;

		diff = std::abs(reply.frontiers[0].xyz_m.y - (-18));
		ASSERT_LE(diff, 0.1) << reply.frontiers[0].xyz_m.y << " and " << -18 << " diff = " << diff;

		diff = std::abs(reply.frontiers[0].xyz_m.z - (6));
		ASSERT_LE(diff, 0.1) << reply.frontiers[0].xyz_m.z << " and " << 6 << " diff = " << diff;
	}

	// TEST(WIPFrontiersTest, Test_normalizeToObservableRange_x)
	// {
	// 	double range = 2;
	// 	octomath::Vector3 goal_voxel_center(0, 0, 0);
	// 	octomath::Vector3 unknown_neighbor (5, 0, 0);
	// 	makeGoalInObservableRange(goal_voxel_center, range, unknown_neighbor);
	// 	ASSERT_EQ(goal_voxel_center.x(), 3);
	// 	ASSERT_EQ(goal_voxel_center.y(), 0);
	// 	ASSERT_EQ(goal_voxel_center.z(), 0);
	// }

	// TEST(WIPFrontiersTest, Test_normalizeToObservableRange_y)
	// {
	// 	double range = 2;
	// 	octomath::Vector3 goal_voxel_center(0, 0, 0);
	// 	octomath::Vector3 unknown_neighbor (0, 5, 0);
	// 	makeGoalInObservableRange(goal_voxel_center, range, unknown_neighbor);
	// 	ASSERT_EQ(goal_voxel_center.x(), 0);
	// 	ASSERT_EQ(goal_voxel_center.y(), 3);
	// 	ASSERT_EQ(goal_voxel_center.z(), 0);
	// }

	// TEST(WIPFrontiersTest, Test_normalizeToObservableRange_z)
	// {
	// 	double range = 2;
	// 	octomath::Vector3 goal_voxel_center(0, 0, 0);
	// 	octomath::Vector3 unknown_neighbor (0, 0, 5);
	// 	makeGoalInObservableRange(goal_voxel_center, range, unknown_neighbor);
	// 	ASSERT_EQ(goal_voxel_center.x(), 0);
	// 	ASSERT_EQ(goal_voxel_center.y(), 0);
	// 	ASSERT_EQ(goal_voxel_center.z(), 3);
	// }



	// TEST(WIPFrontiersTest, Test_normalizeToObservableRange_negativeX)
	// {
	// 	double range = 2;
	// 	octomath::Vector3 goal_voxel_center(0, 0, 0);
	// 	octomath::Vector3 unknown_neighbor (-5, 0, 0);
	// 	makeGoalInObservableRange(goal_voxel_center, range, unknown_neighbor);
	// 	ASSERT_EQ(goal_voxel_center.x(), -3);
	// 	ASSERT_EQ(goal_voxel_center.y(), 0);
	// 	ASSERT_EQ(goal_voxel_center.z(), 0);
	// }

	
	// TEST(WIPFrontiersTest, Test_isCenterGoodGoal_true)
	// {
	// 	ASSERT_TRUE(isCenterGoodGoal(2, 3));
	// }
	
	// TEST(WIPFrontiersTest, Test_isCenterGoodGoal_false)
	// {
	// 	ASSERT_FALSE(isCenterGoodGoal(8, 3));
	// }

	// TEST(WIPFrontiersTest, Test_isCenterGoodGoal_trueSame)
	// {
	// 	ASSERT_TRUE(isCenterGoodGoal(3, 3));
	// }

}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}