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

	TEST(LazyThetaStarTests, LazyThetaStar_Long_Test)
	{
		// (0.420435 0.313896 1.92169) to (-2.5 -10.5 3.5)
		octomap::OcTree octree ("data/octree_noPath1s.bt");
		octomath::Vector3 disc_initial(0.420435, 0.313896, 1.92169);
		octomath::Vector3 disc_final  (-2.5, -10.5, 3.5);
		testStraightLinesForwardWithObstacles(octree, disc_initial, disc_final);

	}


	/*TEST(WorkInProgressTest, ReverseNormalizedLineOfSight)
	{
		double cell_size = 0;
	    octomath::Vector3 p1(0.7, 3.1, 1.3); 
	    octomath::Vector3 p2(0.7, 2.9, 1.3); 
	    std::shared_ptr<octomath::Vector3> p1_ptr = std::make_shared<octomath::Vector3>(p1);
	    std::shared_ptr<octomath::Vector3> p2_ptr = std::make_shared<octomath::Vector3>(p2);
		octomap::OcTree octree ("/home/mfaria/ws_mavlink_grvcHal/src/path_planning/test/data/run_2.bt");
		bool line_of_sight_A = normalizeToVisibleEndCenter(octree, p1_ptr, p2_ptr, cell_size);
		bool line_of_sight_B = normalizeToVisibleEndCenter(octree, p2_ptr, p1_ptr, cell_size);
		ASSERT_EQ(line_of_sight_B, line_of_sight_A);
	}

	
	TEST(WorkInProgressTest, ReverseLineOfSight)
	{
	    octomath::Vector3 p1(0.7, 3.1, 1.3); 
	    octomath::Vector3 p2(0.7, 2.9, 1.3); 
		octomap::OcTree octree ("/home/mfaria/ws_mavlink_grvcHal/src/path_planning/test/data/run_2.bt");

		auto res_node = octree.search(p1);
		if(res_node == NULL)
		{
			ROS_WARN_STREAM("[1] The coordinates " << p1 << " do not correspond to a node in this octree  ==> this neighbor is unknown");
		}
		else
		{
			ASSERT_FALSE(octree.isNodeOccupied(res_node));
		}
		res_node = octree.search(p2);
		if(res_node == NULL)
		{
			ROS_WARN_STREAM("[1] The coordinates " << p2 << " do not correspond to a node in this octree  ==> this neighbor is unknown");
		}
		else
		{
			ASSERT_TRUE(octree.isNodeOccupied(res_node));
		}


		bool line_of_sight_A = hasLineOfSight(octree, p1, p2);
		bool line_of_sight_B = hasLineOfSight(octree, p2, p1);
		ASSERT_FALSE(line_of_sight_B);
		ASSERT_FALSE(line_of_sight_A);
		ASSERT_EQ(line_of_sight_B, line_of_sight_A);
	}*/
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}