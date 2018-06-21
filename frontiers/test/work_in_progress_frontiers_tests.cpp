#include <gtest/gtest.h>
#include <frontiers.h>
#include <neighbors.h>
#include <ordered_neighbors.h>

namespace Frontiers
{
	
    
	TEST(OctreeNeighborTest, InsertTest)
	{

		frontiers_msgs::VoxelMsg current_position;
        current_position.xyz_m.x = 0;
        current_position.xyz_m.y = 0;
        current_position.xyz_m.z = 0;
		OrderedNeighbors neighbors (current_position);
		frontiers_msgs::VoxelMsg voxel_msg;
        voxel_msg.xyz_m.x = 1;
        voxel_msg.xyz_m.y = 0;
        voxel_msg.xyz_m.z = 0;
		neighbors.insert(voxel_msg);
		frontiers_msgs::FrontierReply reply;
		int frontier_count = neighbors.buildMessageList(1, reply);

		ASSERT_EQ(reply.frontiers_found, 1);
		ASSERT_EQ(frontier_count, 1);
		frontier_count = neighbors.buildMessageList(5, reply);
		ASSERT_EQ(reply.frontiers_found, 1);
		ASSERT_EQ(frontier_count, 1);
		frontier_count = neighbors.buildMessageList(0, reply);
		ASSERT_EQ(reply.frontiers_found, 0);
		ASSERT_EQ(frontier_count, 0);
	}
	
	TEST(OctreeNeighborTest, OrderTest)
	{

		frontiers_msgs::VoxelMsg current_position;
        current_position.xyz_m.x = 0;
        current_position.xyz_m.y = 0;
        current_position.xyz_m.z = 0;
		OrderedNeighbors neighbors (current_position);
		frontiers_msgs::VoxelMsg voxel_msg;
        voxel_msg.xyz_m.x = 1;
        voxel_msg.xyz_m.y = 0;
        voxel_msg.xyz_m.z = 0;
		neighbors.insert(voxel_msg);
        voxel_msg.xyz_m.x = 1;
        voxel_msg.xyz_m.y = 1;
        voxel_msg.xyz_m.z = 0;
		neighbors.insert(voxel_msg);
        voxel_msg.xyz_m.x = 1;
        voxel_msg.xyz_m.y = 1;
        voxel_msg.xyz_m.z = 1;
		neighbors.insert(voxel_msg);
		frontiers_msgs::FrontierReply reply;
		int frontier_count = neighbors.buildMessageList(1, reply);

		ASSERT_EQ(reply.frontiers_found, 1);
		ASSERT_EQ(frontier_count, 1);
		ASSERT_EQ(reply.frontiers[0].xyz_m.x, 1);
		ASSERT_EQ(reply.frontiers[0].xyz_m.y, 0);
		ASSERT_EQ(reply.frontiers[0].xyz_m.z, 0);
	}
	TEST(OctreeNeighborTest, OrderTest_2)
	{

		frontiers_msgs::VoxelMsg current_position;
        current_position.xyz_m.x = 0;
        current_position.xyz_m.y = 0;
        current_position.xyz_m.z = 0;
		OrderedNeighbors neighbors (current_position);
		frontiers_msgs::VoxelMsg voxel_msg;
        voxel_msg.xyz_m.x = 1;
        voxel_msg.xyz_m.y = -1;
        voxel_msg.xyz_m.z = 21;
		neighbors.insert(voxel_msg);
        voxel_msg.xyz_m.x = -1;
        voxel_msg.xyz_m.y = 0;
        voxel_msg.xyz_m.z = 0;
		neighbors.insert(voxel_msg);
        voxel_msg.xyz_m.x = 1;
        voxel_msg.xyz_m.y = 10;
        voxel_msg.xyz_m.z = 0;
		neighbors.insert(voxel_msg);
		frontiers_msgs::FrontierReply reply;
		int frontier_count = neighbors.buildMessageList(1, reply);
		ASSERT_EQ(reply.frontiers_found, 1);
		ASSERT_EQ(frontier_count, 1);
		ASSERT_EQ(reply.frontiers[0].xyz_m.x, -1);
		ASSERT_EQ(reply.frontiers[0].xyz_m.y, 0);
		ASSERT_EQ(reply.frontiers[0].xyz_m.z, 0);
	}
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}