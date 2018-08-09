#include <gtest/gtest.h>
#include <frontiers.h>
#include <neighbors.h>
#include <ordered_neighbors.h>
#include <chrono>

namespace LazyThetaStarOctree
{
	void printForMatlab(std::unordered_set<std::shared_ptr<octomath::Vector3>> neighbors)
	{
		std::cout << " x_values = [ ];\n";
		std::cout << " y_values = [ ];\n";
		std::cout << " z_values = [ ];\n";
		for (std::shared_ptr<octomath::Vector3> n : neighbors)
		{
			std::cout << " x_values = [x_values, " << n->x() << "];\n ";
			std::cout << " y_values = [y_values, " << n->y() << "];\n ";
			std::cout << " z_values = [z_values, " << n->z() << "];\n ";
		}
	}

	// TEST(OctreeNeighborTest, NeighborTest_generateFromRealData_Depth13)
	// {
	// 	// ARRANGE
	// 	std::unordered_set<std::shared_ptr<octomath::Vector3>> neighbors_us;
	// 	octomap::OcTree octree ("data/circle_1m.bt");
	// 	double sidelength_lookup_table  [octree.getTreeDepth()];
	//    	LazyThetaStarOctree::fillLookupTable(octree.getResolution(), octree.getTreeDepth(), sidelength_lookup_table); 
	// 	octomath::Vector3 point_coordinates (10.4f, -0.8f, 0.8f);
	// 	octomap::OcTreeKey point_key = octree.coordToKey(point_coordinates);
	// 	int depth = LazyThetaStarOctree::getNodeDepth_Octomap(point_key, octree);
	// 	EXPECT_EQ(13, depth);
	// 	double node_size = octree.getNodeSize(depth); // in meters
	// 	EXPECT_EQ(1.6, node_size);
	// 	float resolution = octree.getResolution();
	// 	EXPECT_FLOAT_EQ(0.2, resolution);
	// 	// ACT
	// 	octomath::Vector3 cell_center_coordinates = LazyThetaStarOctree::getCellCenter(point_coordinates, octree);
	// 	// auto start = std::chrono::high_resolution_clock::now();
	// 	// LazyThetaStarOctree::generateNeighbors_pointers(neighbors_us, cell_center_coordinates, node_size, resolution);
	// 	// auto finish = std::chrono::high_resolution_clock::now();
	// 	// auto time_span = finish - start;
	// 	// ROS_WARN_STREAM("Old took " << std::chrono::duration_cast<std::chrono::nanoseconds>(time_span).count());

	// 	// start = std::chrono::high_resolution_clock::now();
	// 	// LazyThetaStarOctree::generateNeighbors_pointers_sparse(octree, sidelength_lookup_table, neighbors_us, cell_center_coordinates, node_size, resolution);
	// 	// finish = std::chrono::high_resolution_clock::now();
	// 	// time_span = finish - start;
	// 	// ROS_WARN_STREAM("Sparse took " << std::chrono::duration_cast<std::chrono::nanoseconds>(time_span).count());

	// 	// Calculate fraction
	// 	double margin = 1;
	// 	int check_only_x_fraction = 3;
	// 	double resolution_for_neighbors_m = calculate_fraction(resolution, margin, check_only_x_fraction);
	// 	auto start = std::chrono::high_resolution_clock::now();
	// 	LazyThetaStarOctree::generateNeighbors_pointers_margin(neighbors_us, cell_center_coordinates, node_size, resolution, resolution_for_neighbors_m);
	// 	auto finish = std::chrono::high_resolution_clock::now();
	// 	auto time_span = finish - start;
	// 	ROS_WARN_STREAM("Security margin took " << std::chrono::duration_cast<std::chrono::nanoseconds>(time_span).count());
	// 	printForMatlab(neighbors_us);
	// 	// printForMatlab(neighbors_us);
	// }

	TEST(OctreeNeighborTest, NeighborTest_generateMargin_Depth11)
	{
		// ARRANGE
		std::unordered_set<std::shared_ptr<octomath::Vector3>> neighbors_us;
		octomath::Vector3 point_coordinates (10.4f, -0.8f, 0.8f);
		float resolution = 0.2;
		float node_size = 3.2;
		// ACT
		// Calculate fraction
		double margin = 1;
		int check_only_x_fraction = 2;
		double resolution_for_neighbors_m = calculate_fraction(resolution, margin, check_only_x_fraction);
		auto start = std::chrono::high_resolution_clock::now();
		LazyThetaStarOctree::generateNeighbors_pointers_margin(neighbors_us, point_coordinates, node_size, resolution, resolution_for_neighbors_m);
		auto finish = std::chrono::high_resolution_clock::now();
		auto time_span = finish - start;
		ROS_WARN_STREAM("Security margin took " << std::chrono::duration_cast<std::chrono::nanoseconds>(time_span).count());
		printForMatlab(neighbors_us);
	}


	// TEST(OctreeNeighborTest, NeighborTest_generateFromRealData_MaxRes_2D)
	// {
	// 	// ARRANGE
	// 	std::list <octomath::Vector3> right_answers (
	// 		{	octomath::Vector3(0.18f, 0.22f, 0), 
	// 			octomath::Vector3(0.18f, -0.02f, 0), 
	// 			octomath::Vector3(-0.22f, 0.18f, 0),
	// 			octomath::Vector3(0.22f, 0.18f, 0)} );
	// 	octomap::OcTree octree ("data/circle_1m.bt");
	// 	double sidelength_lookup_table  [octree.getTreeDepth()];
	//    	LazyThetaStarOctree::fillLookupTable(octree.getResolution(), octree.getTreeDepth(), sidelength_lookup_table); 
	// 	octomath::Vector3 point_coordinates (0,0,0);
	// 	octomap::OcTreeKey point_key = octree.coordToKey(point_coordinates);
	// 	int depth = LazyThetaStarOctree::getNodeDepth_Octomap(point_key, octree);
	// 	EXPECT_EQ(16, depth);
	// 	double node_size = octree.getNodeSize(depth); // in meters
	// 	EXPECT_EQ(0.2, node_size);
	// 	double resolution = octree.getResolution();
	// 	EXPECT_EQ(0.2, resolution);
	// 	// ACT
	// 	octomath::Vector3 cell_center_coordinates = LazyThetaStarOctree::getCellCenter(point_coordinates, octree);
	// 	std::unordered_set<std::shared_ptr<octomath::Vector3>> neighbors_us;
	// 	auto start = std::chrono::high_resolution_clock::now();
	// 	LazyThetaStarOctree::generateNeighbors_pointers(neighbors_us, cell_center_coordinates, node_size, resolution);
	// 	auto finish = std::chrono::high_resolution_clock::now();
	// 	auto time_span = finish - start;
	// 	ROS_WARN_STREAM("Old took " << std::chrono::duration_cast<std::chrono::nanoseconds>(time_span).count());
	// 	start = std::chrono::high_resolution_clock::now();
	// 	LazyThetaStarOctree::generateNeighbors_pointers_sparse(octree, sidelength_lookup_table, neighbors_us, cell_center_coordinates, node_size, resolution);
	// 	finish = std::chrono::high_resolution_clock::now();
	// 	time_span = finish - start;
	// 	ROS_WARN_STREAM("Sparse took " << std::chrono::duration_cast<std::chrono::nanoseconds>(time_span).count());
		
	// }
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}