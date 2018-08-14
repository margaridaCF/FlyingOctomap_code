#include <ltStar_temp.h>
#include <gtest/gtest.h>
#include <queue>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <visualization_msgs/Marker.h>


#include <chrono>

namespace LazyThetaStarOctree
{

	void testStraightLinesForwardNoObstacles(octomap::OcTree octree, octomath::Vector3 disc_initial, octomath::Vector3 disc_final,
		int const& max_search_iterations = 55)
	{
		double sidelength_lookup_table  [octree.getTreeDepth()];
	   	LazyThetaStarOctree::fillLookupTable(octree.getResolution(), octree.getTreeDepth(), sidelength_lookup_table); 
		ros::Publisher marker_pub;
		double safety_margin = 0.1;
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
		std::list<octomath::Vector3> resulting_path = lazyThetaStar_(octree, disc_initial, disc_final, statistical_data, safety_margin, sidelength_lookup_table, marker_pub, max_search_iterations, true, true);
		// NO PATH
		ASSERT_NE(resulting_path.size(), 0);
		// CANONICAL: straight line, no issues
		// 2 waypoints: The center of start voxel & The center of the goal voxel
		double cell_size_goal = -1;
		octomath::Vector3 cell_center_coordinates_goal = disc_final;
		updateToCellCenterAndFindSize( cell_center_coordinates_goal, octree, cell_size_goal, sidelength_lookup_table);
		ASSERT_LT(      resulting_path.back().distance( cell_center_coordinates_goal ),  cell_size_goal   );
		double cell_size_start = -1;
		octomath::Vector3 cell_center_coordinates_start = disc_initial;
		updateToCellCenterAndFindSize( cell_center_coordinates_start, octree, cell_size_start, sidelength_lookup_table);
		ASSERT_LT(      resulting_path.begin()->distance( cell_center_coordinates_start ),  cell_size_start   );
		// LONG PATHS: 
		if(resulting_path.size() > 2)
		{
			std::list<octomath::Vector3>::iterator it = resulting_path.begin();
			octomath::Vector3 previous = *it;
			++it;
			// Check that there are no redundant parts in the path
			while(it != resulting_path.end())
			{
				ROS_INFO_STREAM("Step: " << *it);
				bool dimensions_y_or_z_change = (previous.y() != it->y()) || (previous.z() != it->z());
				EXPECT_TRUE(dimensions_y_or_z_change) << "(" << previous.y() << " != " << it->y() << ") || (" << previous.z() << " != " << it->z() << ")";
				EXPECT_TRUE(dimensions_y_or_z_change) << " this should be a straight line. Find out why parent node is being replaced.";
				previous = *it;
				++it;
			}
		}
		ASSERT_EQ(0, ThetaStarNode::OustandingObjects());
	}

	// TEST(LazyThetaStarTests, LazyThetaStar_CoreDumped_Test)
	// {
	// 	// (-8.3 -8.3 0.5) to (-7.3 -8.3 0.5)
	// 	octomap::OcTree octree ("data/offShoreOil_1m.bt");
	// 	octomath::Vector3 disc_initial(-8.3, -8.3, 0.5);
	// 	octomath::Vector3 disc_final  (-7.3, -8.3, 0.5);
	// 	testStraightLinesForwardNoObstacles(octree, disc_initial, disc_final);
	// }

	// TEST(LazyThetaStarTests, LazyThetaStar_filteredNeighbors_depthException)
	// {
	// 	// (-8.3 -8.3 0.5) to (-7.3 -8.3 0.5)
	// 	octomap::OcTree octree ("data/offShoreOil_1m.bt");
	// 	octomath::Vector3 center(-8.3, -8.3, 0.5);
	// 	double cell_size = 0.2;
	// 	std::unordered_set<std::shared_ptr<octomath::Vector3>> neighbors;
	// 	generateNeighbors_filter_pointers(neighbors, center, cell_size, octree.getResolution(), octree);
	// 	for (std::unordered_set<std::shared_ptr<octomath::Vector3>>::iterator i = neighbors.begin(); i != neighbors.end(); ++i)
	// 	{
	// 		ROS_INFO_STREAM (**i);
	// 	}
	// 	ROS_INFO_STREAM("Original") ;
	// 	neighbors.clear();
	// 	center = octomath::Vector3 (-8.3, -8.3, 0.5);
	// 	generateNeighbors_pointers(neighbors, center, cell_size, octree.getResolution());
	// 	for (std::unordered_set<std::shared_ptr<octomath::Vector3>>::iterator i = neighbors.begin(); i != neighbors.end(); ++i)
	// 	{
	// 		ROS_INFO_STREAM (**i);
	// 	}
	// }


	  TEST(LazyThetaStarTests, Key_ObstacleAvoidance)
	  {
	    ros::Publisher marker_pub;
	    
		octomath::Vector3 start(  9.85, -5.52, 6.72); 
		octomath::Vector3 end  ( 25.9,   3.53, 6.64); 
		octomap::OcTree octree ("data/20180804_karting_tooSlowObstacleAvoidance.bt");
		bool ignoreUnknown = 0;
		bool publish = 1;
		double safety_margin = 5;
		int max_search_iterations = 150;
	    ResultSet statistical_data;
	    double sidelength_lookup_table  [octree.getTreeDepth()];
	   	LazyThetaStarOctree::fillLookupTable(octree.getResolution(), octree.getTreeDepth(), sidelength_lookup_table); 
		std::list<octomath::Vector3> resulting_path = lazyThetaStar_(octree, start, end, statistical_data, safety_margin, sidelength_lookup_table, marker_pub, max_search_iterations, true, false);
	  }	

	// TEST(OctreeNeighborTest, NeighborTest_generateFromRealData_Depth13)
	// {
	// 	// ARRANGE
	// 	octomap::OcTree octree ("data/circle_1m.bt");
	// 	octomath::Vector3 point_coordinates (10.4f, -0.8f, 0.8f);
	// 	std::unordered_set<std::shared_ptr<octomath::Vector3>> neighbors;
	// 	octomap::OcTreeKey point_key = octree.coordToKey(point_coordinates);
	// 	int depth = getNodeDepth_Octomap(point_key, octree);
	// 	// EXPECT_EQ(13, depth);
	// 	double node_size = octree.getNodeSize(depth); // in meters
	// 	// EXPECT_EQ(1.6, node_size);
	// 	float resolution = octree.getResolution();
	// 	// EXPECT_FLOAT_EQ(0.2, resolution);
	// 	// // ACT
	// 	// octomath::Vector3 cell_center_coordinates = getCellCenter(point_coordinates, octree);
	// 	generateNeighbors_filter_pointers(neighbors, point_coordinates, node_size, resolution, octree);
	// 	// printForTesting(neighbors_us);
	// 	// printForMatlab(neighbors_us);
	// 	// ASSERT
	// 	// EXPECT_EQ(cell_center_coordinates, octomath::Vector3 (10.4f, -0.8f, 0.8f));	
	// 	// EXPECT_EQ(384, neighbors_us.size());
	// 	// EXPECT_EQ(384, right_answers.size());
	// 	// ASSERT_TRUE (allNeighborsAreCorrect(neighbors_us, right_answers));
	// }

	TEST(OctreeNeighborTest, NeighborTest_addTwoEqualToNeighbors)
	{
		octomap::OcTree octree ("data/circle_1m.bt");
		octomath::Vector3 point_coordinates (10.4f, -0.8f, 0.8f);
		unordered_set_pointers neighbors;
		addSparseNeighbor(neighbors, 9.5, -1.5, 0.1, octree);
		ASSERT_EQ(neighbors.size(), 1);
		addSparseNeighbor(neighbors, 9.5, -1.5, 0.3, octree);
		ASSERT_EQ(neighbors.size(), 1);
	}


	TEST(OctreeNeighborTest, NeighborTest_addOneToNeighbors)
	{
		octomap::OcTree octree ("data/circle_1m.bt");
		octomath::Vector3 point_coordinates (10.4f, -0.8f, 0.8f);
		unordered_set_pointers neighbors;
		addSparseNeighbor(neighbors, 9.5, -1.5, 0.1, octree);
		ASSERT_EQ(neighbors.size(), 1);
		
	}

	TEST(OctreeNeighborTest, NeighborTest_SetOfPointers)
	{
		octomath::Vector3 point_coordinates (10.4f, -0.8f, 0.8f);
		unordered_set_pointers neighbors;
		std::shared_ptr<octomath::Vector3> toInsert_ptr = std::make_shared<octomath::Vector3> (point_coordinates);
		bool result = neighbors.insert(toInsert_ptr).second;
		ASSERT_EQ(neighbors.size(), 1);
        ASSERT_TRUE( result);
		toInsert_ptr = std::make_shared<octomath::Vector3> (point_coordinates);
       	result = neighbors.insert(toInsert_ptr).second;
		ASSERT_EQ(neighbors.size(), 1);
        ASSERT_FALSE( result);   
	}
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}