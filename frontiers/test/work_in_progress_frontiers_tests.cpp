#include <gtest/gtest.h>
#include <frontiers.h>
#include <neighbors.h>

namespace Frontiers
{
    
	TEST(WIPFrontiersTest, Test_calculateCloserPosition_x)
	{
		octomath::Vector3 sensing_position;
		octomath::Vector3 n_coordinates (3, 0, 0);
		double sensing_distance = 2;
		octomath::Vector3 voxel_center(0, 0, 0);
		calculate_closer_position(sensing_position, n_coordinates, sensing_distance, voxel_center);
		ASSERT_EQ(sensing_position.x(), 1);
		ASSERT_EQ(sensing_position.y(), 0);
		ASSERT_EQ(sensing_position.z(), 0);
	}

}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}