#include <grid_benchmark.h>
#include <gtest/gtest.h>

namespace mapper{
	TEST(VoxelTest, Voxel_is_InZlevel)
	{
		Voxel target (0, 0, 0, 0.2f);
		EXPECT_TRUE(target.isInZlevel(0));
	}
	TEST(VoxelTest, Voxel_is_InZlevel_size)
	{
		Voxel target (0, 0, 0.5f, 2);
		EXPECT_TRUE(target.isInZlevel(0));
	}

	TEST(VoxelTest, Voxel_not_InZlevel)
	{
		Voxel target (0, 0, 0, 0.2f);
		EXPECT_FALSE(target.isInZlevel(1));
	}
	TEST(VoxelTest, Voxel_not_InZlevel_size)
	{
		Voxel target (0, 0, 1, 1);
		EXPECT_FALSE(target.isInZlevel(0));
	}
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}