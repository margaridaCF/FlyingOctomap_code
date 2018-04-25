#include <ltStar_temp.h>
#include <gtest/gtest.h>
#include <queue>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <visualization_msgs/Marker.h>

namespace LazyThetaStarOctree{
	
	void testStraightLinesForwardWithObstacles(octomap::OcTree octree, octomath::Vector3 disc_initial, octomath::Vector3 disc_final,
		int const& max_search_iterations = 10000)
	{
		double safety_margin = 2;
		// Initial node is not occupied
		octomap::OcTreeNode* originNode = octree.search(disc_initial);
		ASSERT_TRUE(originNode);
		ASSERT_FALSE(octree.isNodeOccupied(originNode));
		// Final node is not occupied
		octomap::OcTreeNode* finalNode = octree.search(disc_final);
		ASSERT_TRUE(finalNode);
		ASSERT_FALSE(octree.isNodeOccupied(finalNode));
		// The path is clear from start to finish
		octomath::Vector3 direction (1, 0, 0);
		octomath::Vector3 end;
		bool isOccupied = octree.castRay(disc_initial, direction, end, false, weightedDistance(disc_initial, disc_final));
		ASSERT_FALSE(isOccupied); // false if the maximum range or octree bounds are reached, or if an unknown node was hit.

		ResultSet statistical_data;
		std::list<octomath::Vector3> resulting_path = lazyThetaStar_(octree, disc_initial, disc_final, statistical_data, safety_margin, max_search_iterations, true);
		// NO PATH
		ASSERT_NE(resulting_path.size(), 0);
		// 2 waypoints: The center of start voxel & The center of the goal voxel
		double cell_size_goal = -1;
		octomath::Vector3 cell_center_coordinates_goal = disc_final;
		updateToCellCenterAndFindSize( cell_center_coordinates_goal, octree, cell_size_goal);
		ASSERT_LT(      resulting_path.back().distance( cell_center_coordinates_goal ),  cell_size_goal   );
		double cell_size_start = -1;
		octomath::Vector3 cell_center_coordinates_start = disc_initial;
		updateToCellCenterAndFindSize( cell_center_coordinates_start, octree, cell_size_start);
		ASSERT_LT(      resulting_path.begin()->distance( cell_center_coordinates_start ),  cell_size_start   );
		
		ASSERT_EQ(0, ThetaStarNode::OustandingObjects());
	}

	TEST(LazyThetaStarTests, LazyThetaStar_AddingBadNode)
	{
		// (0.420435 0.313896 1.92169) to (-2.5 -10.5 3.5)
		octomap::OcTree octree ("data/(-10.3054; -18.2637; 2.34813)_(-8.5; 6.5; 3.5)_badNodeAdded.bt");
		LTStarRequest request;
		request.header.seq = 2;
		request.request_id = 3;
		request.start.x = -10.3054;
		request.start.y = -18.2637;
		request.start.z = 2.34813;
		request.goal.x = -8.5;
		request.goal.y = 6.5;
		request.goal.z = 3.5;
		request.max_search_iterations = 500;
		request.safety_margin = 0;
		octomath::Vector3 disc_initial(-10.3054, 0.313896, 1.92169);
		octomath::Vector3 disc_final  (-2.5, -10.5, 3.5);
		testStraightLinesForwardWithObstacles(octree, disc_initial, disc_final);

	}
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}