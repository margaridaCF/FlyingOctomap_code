#include <ltStar_temp.h>
#include <gtest/gtest.h>
#include <queue>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <visualization_msgs/Marker.h>


#include <chrono>

namespace LazyThetaStarOctree
{

	// void testStraightLinesForwardWithObstacles(octomap::OcTree octree, octomath::Vector3 disc_initial, octomath::Vector3 disc_final,
	// 	int const& max_search_iterations = 55, double safety_margin = 2)
	// {

	// 	double sidelength_lookup_table  [octree.getTreeDepth()];
	//    	LazyThetaStarOctree::fillLookupTable(octree.getResolution(), octree.getTreeDepth(), sidelength_lookup_table); 
	// 	ros::Publisher marker_pub;
	// 	// Initial node is not occupied
	// 	octomap::OcTreeNode* originNode = octree.search(disc_initial);
	// 	ASSERT_TRUE(originNode);
	// 	ASSERT_FALSE(octree.isNodeOccupied(originNode));
	// 	// Final node is not occupied
	// 	octomap::OcTreeNode* finalNode = octree.search(disc_final);
	// 	ASSERT_TRUE(finalNode);
	// 	ASSERT_FALSE(octree.isNodeOccupied(finalNode));
	// 	// The path is clear from start to finish
	// 	octomath::Vector3 direction (1, 0, 0);
	// 	octomath::Vector3 end;
	// 	bool isOccupied = octree.castRay(disc_initial, direction, end, false, weightedDistance(disc_initial, disc_final));
	// 	ASSERT_FALSE(isOccupied); // false if the maximum range or octree bounds are reached, or if an unknown node was hit.

	// 	ResultSet statistical_data;
	// 	auto start = std::chrono::high_resolution_clock::now();
	// 	std::list<octomath::Vector3> resulting_path = lazyThetaStar_(octree, disc_initial, disc_final, statistical_data, safety_margin, sidelength_lookup_table, marker_pub, max_search_iterations);
	// 	auto finish = std::chrono::high_resolution_clock::now();
	// 	auto time_span = finish - start;
	// 	ROS_WARN_STREAM("Old took " << std::chrono::duration_cast<std::chrono::milliseconds>(time_span).count());


	// 	start = std::chrono::high_resolution_clock::now();
	// 	resulting_path = lazyThetaStar_sparse(octree, disc_initial, disc_final, statistical_data, safety_margin, sidelength_lookup_table, marker_pub, max_search_iterations);
	// 	finish = std::chrono::high_resolution_clock::now();
	// 	time_span = finish - start;
	// 	ROS_WARN_STREAM("Sparse took " << std::chrono::duration_cast<std::chrono::milliseconds>(time_span).count());


		
	// 	// NO PATH
	// 	ASSERT_NE(resulting_path.size(), 0) << safety_margin;
	// 	// 2 waypoints: The center of start voxel & The center of the goal voxel
	// 	double cell_size_goal = -1;
	// 	octomath::Vector3 cell_center_coordinates_goal = disc_final;
	// 	updateToCellCenterAndFindSize( cell_center_coordinates_goal, octree, cell_size_goal, sidelength_lookup_table);
	// 	ASSERT_LT(      resulting_path.back().distance( cell_center_coordinates_goal ),  cell_size_goal   );
	// 	double cell_size_start = -1;
	// 	octomath::Vector3 cell_center_coordinates_start = disc_initial;
	// 	updateToCellCenterAndFindSize( cell_center_coordinates_start, octree, cell_size_start, sidelength_lookup_table);
	// 	ASSERT_LT(      resulting_path.begin()->distance( cell_center_coordinates_start ),  cell_size_start   );
		
	// 	ASSERT_EQ(0, ThetaStarNode::OustandingObjects());
	// }

	// TEST(LazyThetaStarTests, ObstaclePath_10m_Test_16)
	// {
		
	//     octomath::Vector3 disc_initial(0, 5, 1.5); 
	//     octomath::Vector3 disc_final  (2, -5, 1.5); 
	//     octomap::OcTree octree ("data/run_2.bt");
	//     std::string dataset_name = "run 2";
	//     testStraightLinesForwardWithObstacles(octree, disc_initial, disc_final, 1000, 1.6);
	// }

	TEST(LazyThetaStarTests, ObstaclePath_10m_Test_16_vanilla)
	{
		
	    octomath::Vector3 disc_initial(0, 5, 1.5); 
	    octomath::Vector3 disc_final  (2, -5, 1.5); 
	    octomap::OcTree octree ("data/run_2.bt");
	    std::string dataset_name = "run 2";
		double sidelength_lookup_table  [octree.getTreeDepth()];
	   	LazyThetaStarOctree::fillLookupTable(octree.getResolution(), octree.getTreeDepth(), sidelength_lookup_table); 
		ResultSet statistical_data;
		ros::Publisher marker_pub;
		double safety_margin = 1.6;
		int max_search_iterations = 1000;

	    auto start = std::chrono::high_resolution_clock::now();
		std::list<octomath::Vector3> resulting_path = lazyThetaStar_(octree, disc_initial, disc_final, statistical_data, safety_margin, sidelength_lookup_table, marker_pub, max_search_iterations);
		auto finish = std::chrono::high_resolution_clock::now();
		auto time_span = finish - start;
		ROS_WARN_STREAM("Margin took " << std::chrono::duration_cast<std::chrono::milliseconds>(time_span).count());
	}

	TEST(LazyThetaStarTests, ObstaclePath_10m_Test_16_margin)
	{
		
	    octomath::Vector3 disc_initial(0, 5, 1.5); 
	    octomath::Vector3 disc_final  (2, -5, 1.5); 
	    octomap::OcTree octree ("data/run_2.bt");
	    std::string dataset_name = "run 2";
		double sidelength_lookup_table  [octree.getTreeDepth()];
	   	LazyThetaStarOctree::fillLookupTable(octree.getResolution(), octree.getTreeDepth(), sidelength_lookup_table); 
		ResultSet statistical_data;
		ros::Publisher marker_pub;
		double safety_margin = 1.6;
		int max_search_iterations = 1000;

	    auto start = std::chrono::high_resolution_clock::now();
		std::list<octomath::Vector3> resulting_path = lazyThetaStar_margin_n(octree, disc_initial, disc_final, statistical_data, safety_margin, sidelength_lookup_table, marker_pub, 3, max_search_iterations);
		auto finish = std::chrono::high_resolution_clock::now();
		auto time_span = finish - start;
		ROS_WARN_STREAM("Margin took " << std::chrono::duration_cast<std::chrono::milliseconds>(time_span).count());
	}

	TEST(LazyThetaStarTests, ObstaclePath_10m_Test_16_sparse)
	{
		
	    octomath::Vector3 disc_initial(0, 5, 1.5); 
	    octomath::Vector3 disc_final  (2, -5, 1.5); 
	    octomap::OcTree octree ("data/run_2.bt");
	    std::string dataset_name = "run 2";
		double sidelength_lookup_table  [octree.getTreeDepth()];
	   	LazyThetaStarOctree::fillLookupTable(octree.getResolution(), octree.getTreeDepth(), sidelength_lookup_table); 
		ResultSet statistical_data;
		ros::Publisher marker_pub;
		double safety_margin = 1.6;
		int max_search_iterations = 1000;

	    auto start = std::chrono::high_resolution_clock::now();
		std::list<octomath::Vector3> resulting_path = lazyThetaStar_sparse(octree, disc_initial, disc_final, statistical_data, safety_margin, sidelength_lookup_table, marker_pub, max_search_iterations);
		auto finish = std::chrono::high_resolution_clock::now();
		auto time_span = finish - start;
		ROS_WARN_STREAM("Margin took " << std::chrono::duration_cast<std::chrono::milliseconds>(time_span).count());
	}
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}