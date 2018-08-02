#include <gtest/gtest.h>
#include <frontiers.h>
#include <neighbors.h>
#include <ordered_neighbors.h>

namespace Frontiers
{
	TEST(OctreeNeighborTest, fillLookupTable)
	{
		double resolution = 0.2;
		int treeDepth = 16;
		double sidelength_lookup_table [treeDepth];
		LazyThetaStarOctree::fillLookupTable(resolution, treeDepth, sidelength_lookup_table);
		ASSERT_EQ(LazyThetaStarOctree::findSideLenght(treeDepth, treeDepth, sidelength_lookup_table), resolution);
		ASSERT_EQ(LazyThetaStarOctree::findSideLenght(treeDepth, treeDepth-1, sidelength_lookup_table), resolution*2);
		ASSERT_EQ(LazyThetaStarOctree::findSideLenght(treeDepth, treeDepth-2, sidelength_lookup_table), resolution*2 + resolution*2);
	}
    
	TEST(OctreeNeighborTest, UpdateToCellCenterAndFindSize)
	{
		// ARRANGE
		// Cell
		int depth = 14;
		octomath::Vector3 cell_center (-0.4f, 1.2f, 1.2f);
		double cell_size = 0.8;
		octomap::OcTree octree ("data/circle_1m.bt");
		double sidelength_lookup_table [octree.getTreeDepth()];
		LazyThetaStarOctree::fillLookupTable(octree.getResolution(), octree.getTreeDepth(), sidelength_lookup_table);
		double cell_size_result = -1;
		// INSIDE
		std::list <octomath::Vector3> inside_points_to_test {
			octomath::Vector3  (-0.4f, 1.2f, 1.4f),
			octomath::Vector3  (-0.4f, 1.2f, 1.f),
			octomath::Vector3  (-0.4f, 1.4f, 1.2f),
			octomath::Vector3  (-0.4f, 1.f, 1.2f),
			octomath::Vector3  (-0.2f, 1.2f, 1.2f),
			octomath::Vector3  (-0.6f, 1.2f, 1.2f),
		};
		for(octomath::Vector3 point : inside_points_to_test)
		{
			// ACT
			LazyThetaStarOctree::updateToCellCenterAndFindSize(point, octree, cell_size_result, sidelength_lookup_table);
			// ASSERT
			EXPECT_EQ(cell_center, point);
			EXPECT_EQ(cell_size, cell_size_result);
		}
		// OUTSIDE
		std::list <octomath::Vector3> outside_points_to_test {
			octomath::Vector3  (0.f, 1.2f, 1.2f),
			octomath::Vector3  (-0.4f, 1.6f, 1.2f),
			octomath::Vector3  (-0.4f, 1.2f, 1.6f),
			octomath::Vector3  (0.1f, 1.2f, 1.2f),
			octomath::Vector3  (-0.4f, 1.7f, 1.2f),
			octomath::Vector3  (-0.4f, 1.2f, 1.7f),
			octomath::Vector3  (-1.f, 3.f, 1.f),
		};
		for(octomath::Vector3 point : outside_points_to_test)
		{
			// ACT
			LazyThetaStarOctree::updateToCellCenterAndFindSize(point, octree, cell_size_result, sidelength_lookup_table);
			// ASSERT
			EXPECT_FALSE(cell_center==point);
		}
	}
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}