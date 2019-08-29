#include <neighbors.h>
#include <gtest/gtest.h>
#include <ordered_neighbors.h>

namespace LazyThetaStarOctree{


	bool equal (const octomath::Vector3 & a, const octomath::Vector3 & b, 
		const double theta = 0.00000000000000000001) 
	{

		bool is_x_equal = abs(a.x() - b.x()) < theta;
		bool is_y_equal = abs(a.y() - b.y()) < theta;
		bool is_z_equal = abs(a.z() - b.z()) < theta;

		return is_x_equal && is_y_equal && is_z_equal;
	}

	bool allNeighborsAreCorrect(unordered_set_pointers const neighbors , 
		std::list <octomath::Vector3> const right_answers)
	{

		bool all_neighbors_are_correct = true;
		unordered_set_pointers wrong_answers;
		octomath::Vector3 temp;

		if(neighbors.size() != right_answers.size())
		{
			all_neighbors_are_correct = false;
			ROS_WARN_STREAM("There are " << neighbors.size() << " neighbors but there should be " << right_answers.size());
		}
		for (std::shared_ptr<octomath::Vector3> n : neighbors)
		{
			bool found  = false;
			for(octomath::Vector3 answer : right_answers)
			{
				if( equal(answer, (*n)) )
 				{
 					found  = true;
 					break;
				}
			}
				
			if(!found)
			{
				wrong_answers.insert(n);
				all_neighbors_are_correct = false;
			}
		}


		if(!wrong_answers.empty())
		{
			ROS_WARN_STREAM("There are " << wrong_answers.size() << " wrong answers.");
			std::cout << std::setprecision(30);

			octomath::Vector3 wrong;
			for (auto w : wrong_answers)
			{
				std::cout << "Wrong answer: " << *w << "\n";
				wrong = *w;
			}
			std::cout << "Right answers: " <<  "\n";
			for(octomath::Vector3 nv : right_answers)
			{
				std::cout << nv << "\n";
			}

		}

		return all_neighbors_are_correct;
	}

	void printForMatlab(unordered_set_pointers neighbors)
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

	void printForTesting(unordered_set_pointers neighbors)
	{
		std::cout << "unordered_set_pointers right_answers =\n{";
		for(std::shared_ptr<octomath::Vector3> nv : neighbors)
		{
			std::cout << " octomath::Vector3(" << nv->x() << ", " << nv->y() << ", " << nv->z() << "),\n";
		}
		std::cout << "} ;\n";
	}


	TEST(OctreeNeighborTest, DepthTest)
	{
		// ARRANGE
		octomap::OcTree octree ("data/circle_1m.bt");
	    // 16
	    octomath::Vector3 point_coordinates = octomath::Vector3 (0.1f, 0.1f, 0.f);
	    octomap::OcTreeKey point_key = octree.coordToKey(point_coordinates);
	    int depth = getNodeDepth_Octomap(point_key, octree);
		ASSERT_EQ(16, depth);
	    // 15
	    point_coordinates = octomath::Vector3 (-8.6f, -6.6f, -0.2f);
	    point_key = octree.coordToKey(point_coordinates);
	    depth = getNodeDepth_Octomap(point_key, octree);
		ASSERT_EQ(15, depth);
	    // 14
	    point_coordinates = octomath::Vector3 (-13.2f, -6.8f, 3.6f);
	    point_key = octree.coordToKey(point_coordinates);
	    depth = getNodeDepth_Octomap(point_key, octree);
		ASSERT_EQ(14, depth);
	    // 13
	    point_coordinates = octomath::Vector3 (-10.4f, -4.f, 0.8f);
	    point_key = octree.coordToKey(point_coordinates);
	    depth = getNodeDepth_Octomap(point_key, octree);
		ASSERT_EQ(13, depth);
	}
	/* Earlier assessement of how well the depth was calculated
		TEST(OctreeNeighborTest, DepthTest_Rainville_16lvl)
		{
			// ARRANGE
			octomap::OcTree octree ("data/circle_1m.bt");
			octomath::Vector3 point_coordinates (0.1f, 0.1f, 0);
			octomap::OcTreeKey point_key = octree.coordToKey(point_coordinates);
			// ACT
			int depth = getDepth_sourceCodeInspiration(point_key, octree);
			// ASSERT
			ASSERT_EQ(16, depth);
		}

		TEST(OctreeNeighborTest, DepthTest_Rainville_13lvl)
		{
			// ARRANGE
			octomap::OcTree octree ("data/circle_1m.bt");
			octomath::Vector3 point_coordinates (10.4f, -0.8f, 0);
			octomap::OcTreeKey point_key = octree.coordToKey(point_coordinates);
			// ACT
			int depth = getNodeDepth(&octree, point_key);
			// ASSERT
			ASSERT_EQ(13, depth);
		}

		TEST(OctreeNeighborTest, DepthTest_octomap_13lvl)
		{
			// ARRANGE
			octomap::OcTree octree ("data/circle_1m.bt");
			octomath::Vector3 point_coordinates (10.4f, -0.8f, 0);
			octomap::OcTreeKey point_key = octree.coordToKey(point_coordinates);
			// ACT
			int depth = getDepth_sourceCodeInspiration(point_key, octree);
			// ASSERT
			ASSERT_EQ(13, depth);
		}

		TEST(OctreeNeighborTest, DepthTest_Rainville_15lvl)
		{
			// ARRANGE
			octomap::OcTree octree ("data/circle_1m.bt");
			octomath::Vector3 point_coordinates (58.6f, 42.6f, 0);
			octomap::OcTreeKey point_key = octree.coordToKey(point_coordinates);
			// ACT
			int depth = getNodeDepth(&octree, point_key);
			// ASSERT
			ASSERT_EQ(15, depth);
		}

		TEST(OctreeNeighborTest, DepthTest_octomap_15lvl)
		{
			// ARRANGE
			octomap::OcTree octree ("data/circle_1m.bt");
			octomath::Vector3 point_coordinates (58.6f, 42.6f, 0);
			octomap::OcTreeKey point_key = octree.coordToKey(point_coordinates);
			// ACT
			int depth = getDepth_sourceCodeInspiration(point_key, octree);
			// ASSERT
			ASSERT_EQ(15, depth);
		}

		TEST(OctreeNeighborTest, DepthTest_Rainville_14lvl)
		{
			// ARRANGE
			octomap::OcTree octree ("data/circle_1m.bt");
			octomath::Vector3 point_coordinates (17.2f, 1.2f, 0);
			octomap::OcTreeKey point_key = octree.coordToKey(point_coordinates);
			// ACT
			int depth = getNodeDepth(&octree, point_key);
			// ASSERT
			ASSERT_EQ(14, depth);
		}

		TEST(OctreeNeighborTest, DepthTest_octomap_14lvl)
		{
			// ARRANGE
			octomap::OcTree octree ("data/circle_1m.bt");
			octomath::Vector3 point_coordinates (17.2f, 1.2f, 0);
			octomap::OcTreeKey point_key = octree.coordToKey(point_coordinates);
			// ACT
			int depth = getDepth_sourceCodeInspiration(point_key, octree);
			// ASSERT
			ASSERT_EQ(14, depth);
		}
	*/

	TEST(OctreeNeighborTest, NeighborTest_generateFromRealData_Depth14)
	{
		// ARRANGE
		std::list <octomath::Vector3> right_answers =
		{ 
			octomath::Vector3(-0.1, 1.5, 0.7),
			octomath::Vector3(-0.1, 1.5, 1.7),
			octomath::Vector3(-0.1, 0.7, 1.5),
			octomath::Vector3(-0.1, 1.7, 1.5),
			octomath::Vector3(0.1, 1.5, 1.5),
			octomath::Vector3(-0.9, 1.5, 1.5),
			octomath::Vector3(-0.1, 1.3, 0.7),
			octomath::Vector3(-0.1, 1.3, 1.7),
			octomath::Vector3(-0.1, 0.7, 1.3),
			octomath::Vector3(-0.1, 1.7, 1.3),
			octomath::Vector3(0.1, 1.5, 1.3),
			octomath::Vector3(-0.9, 1.5, 1.3),
			octomath::Vector3(-0.1, 1.1, 0.7),
			octomath::Vector3(-0.1, 1.1, 1.7),
			octomath::Vector3(-0.1, 0.7, 1.1),
			octomath::Vector3(-0.1, 1.7, 1.1),
			octomath::Vector3(0.1, 1.5, 1.1),
			octomath::Vector3(-0.9, 1.5, 1.1),
			octomath::Vector3(-0.1, 0.9, 0.7),
			octomath::Vector3(-0.1, 0.9, 1.7),
			octomath::Vector3(-0.1, 0.7, 0.9),
			octomath::Vector3(-0.1, 1.7, 0.9),
			octomath::Vector3(0.1, 1.5, 0.9),
			octomath::Vector3(-0.9, 1.5, 0.9),
			octomath::Vector3(-0.3, 1.5, 0.7),
			octomath::Vector3(-0.3, 1.5, 1.7),
			octomath::Vector3(-0.3, 0.7, 1.5),
			octomath::Vector3(-0.3, 1.7, 1.5),
			octomath::Vector3(0.1, 1.3, 1.5),
			octomath::Vector3(-0.9, 1.3, 1.5),
			octomath::Vector3(-0.3, 1.3, 0.7),
			octomath::Vector3(-0.3, 0.9, 1.7),
			octomath::Vector3(-0.3, 0.7, 0.9),
			octomath::Vector3(-0.3, 1.7, 0.9),
			octomath::Vector3(0.1, 1.3, 0.9),
			octomath::Vector3(-0.9, 1.3, 0.9),
			octomath::Vector3(-0.5, 1.5, 0.7),
			octomath::Vector3(-0.3, 1.3, 1.7),
			octomath::Vector3(-0.5, 1.5, 1.7),
			octomath::Vector3(0.1, 0.9, 1.5),
			octomath::Vector3(-0.9, 0.9, 1.5),
			octomath::Vector3(-0.7, 1.3, 0.7),
			octomath::Vector3(-0.7, 1.3, 1.7),
			octomath::Vector3(-0.7, 0.7, 1.3),
			octomath::Vector3(-0.7, 1.7, 1.3),
			octomath::Vector3(0.1, 0.9, 1.3),
			octomath::Vector3(-0.9, 0.9, 1.3),
			octomath::Vector3(-0.7, 1.1, 0.7),
			octomath::Vector3(-0.7, 1.1, 1.7),
			octomath::Vector3(-0.7, 1.7, 1.5),
			octomath::Vector3(-0.9, 0.9, 0.9),
			octomath::Vector3(-0.3, 1.1, 0.7),
			octomath::Vector3(-0.5, 1.3, 0.7),
			octomath::Vector3(-0.7, 0.7, 1.5),
			octomath::Vector3(0.1, 0.9, 0.9),
			octomath::Vector3(-0.9, 1.3, 1.3),
			octomath::Vector3(-0.9, 1.1, 1.5),
			octomath::Vector3(-0.7, 1.7, 0.9),
			octomath::Vector3(0.1, 1.3, 1.3),
			octomath::Vector3(0.1, 1.1, 1.5),
			octomath::Vector3(-0.7, 0.7, 0.9),
			octomath::Vector3(-0.3, 1.7, 1.3),
			octomath::Vector3(-0.5, 1.7, 1.5),
			octomath::Vector3(-0.7, 0.9, 1.7),
			octomath::Vector3(-0.3, 0.7, 1.3),
			octomath::Vector3(-0.5, 0.7, 1.5),
			octomath::Vector3(-0.7, 0.9, 0.7),
			octomath::Vector3(-0.9, 0.9, 1.1),
			octomath::Vector3(0.1, 0.9, 1.1),
			octomath::Vector3(-0.7, 1.7, 1.1),
			octomath::Vector3(-0.7, 0.7, 1.1),
			octomath::Vector3(-0.7, 1.5, 1.7),
			octomath::Vector3(-0.7, 1.5, 0.7),
			octomath::Vector3(-0.9, 1.1, 0.9),
			octomath::Vector3(0.1, 1.1, 0.9),
			octomath::Vector3(-0.5, 1.7, 0.9),
			octomath::Vector3(-0.5, 0.7, 0.9),
			octomath::Vector3(-0.5, 0.9, 1.7),
			octomath::Vector3(-0.5, 0.9, 0.7),
			octomath::Vector3(-0.9, 1.1, 1.1),
			octomath::Vector3(0.1, 1.1, 1.1),
			octomath::Vector3(-0.5, 1.7, 1.1),
			octomath::Vector3(-0.5, 0.7, 1.1),
			octomath::Vector3(-0.5, 1.1, 1.7),
			octomath::Vector3(-0.3, 0.9, 0.7),
			octomath::Vector3(-0.5, 1.1, 0.7),
			octomath::Vector3(-0.9, 1.3, 1.1),
			octomath::Vector3(-0.9, 1.1, 1.3),
			octomath::Vector3(0.1, 1.3, 1.1),
			octomath::Vector3(0.1, 1.1, 1.3),
			octomath::Vector3(-0.3, 1.7, 1.1),
			octomath::Vector3(-0.5, 1.7, 1.3),
			octomath::Vector3(-0.3, 0.7, 1.1),
			octomath::Vector3(-0.5, 0.7, 1.3),
			octomath::Vector3(-0.3, 1.1, 1.7),
			octomath::Vector3(-0.5, 1.3, 1.7)
		} ;
		unordered_set_pointers neighbors_us;
		octomap::OcTree octree ("data/circle_1m.bt");
		octomath::Vector3 point_coordinates (-0.4f, 1.2f, 1.2f);
		octomap::OcTreeKey point_key = octree.coordToKey(point_coordinates);
		int depth = getNodeDepth_Octomap(point_key, octree);
		EXPECT_EQ(14, depth);
		double node_size = octree.getNodeSize(depth); // in meters
		EXPECT_EQ(0.8, node_size);
		float resolution = octree.getResolution();
		EXPECT_FLOAT_EQ(0.2, resolution);
		// ACT
		octomath::Vector3 cell_center_coordinates = getCellCenter(point_coordinates, octree);
		generateNeighbors_pointers(neighbors_us, cell_center_coordinates, node_size, resolution);
		// ASSERT
		EXPECT_EQ(cell_center_coordinates, octomath::Vector3 (-0.4f, 1.2f, 1.2f));	// Ground data
		EXPECT_EQ(96, neighbors_us.size());
		EXPECT_EQ(96, right_answers.size());
		ASSERT_TRUE (allNeighborsAreCorrect(neighbors_us, right_answers));
	}

	TEST(OctreeNeighborTest, NeighborTest_generateFromRealData_Depth13)
	{
		// ARRANGE
		std::list<octomath::Vector3> right_answers =
		{ 
			octomath::Vector3(11.1, -0.1, -0.1),
			octomath::Vector3(11.1, -0.1, 1.7),
			octomath::Vector3(11.1, -1.7, 1.5),
			octomath::Vector3(11.1, 0.1, 1.5),
			octomath::Vector3(11.3, -0.1, 1.5),
			octomath::Vector3(9.5, -0.1, 1.5),
			octomath::Vector3(11.1, -0.3, -0.1),
			octomath::Vector3(11.1, -0.3, 1.7),
			octomath::Vector3(11.1, -1.7, 1.3),
			octomath::Vector3(11.1, 0.1, 1.3),
			octomath::Vector3(11.3, -0.1, 1.3),
			octomath::Vector3(9.5, -0.1, 1.3),
			octomath::Vector3(11.1, -0.5, -0.1),
			octomath::Vector3(11.1, -0.5, 1.7),
			octomath::Vector3(11.1, -1.7, 1.1),
			octomath::Vector3(11.1, 0.1, 1.1),
			octomath::Vector3(11.3, -0.1, 1.1),
			octomath::Vector3(9.5, -0.1, 1.1),
			octomath::Vector3(11.1, -0.7, -0.1),
			octomath::Vector3(11.1, -0.7, 1.7),
			octomath::Vector3(11.1, -1.7, 0.9),
			octomath::Vector3(11.1, 0.1, 0.9),
			octomath::Vector3(11.3, -0.1, 0.9),
			octomath::Vector3(9.5, -0.1, 0.5),
			octomath::Vector3(11.1, -1.3, -0.1),
			octomath::Vector3(11.1, -1.3, 1.7),
			octomath::Vector3(11.1, -1.7, 0.3),
			octomath::Vector3(11.1, 0.1, 0.3),
			octomath::Vector3(11.3, -0.1, 0.3),
			octomath::Vector3(9.5, -0.1, 0.3),
			octomath::Vector3(11.1, -1.5, -0.1),
			octomath::Vector3(11.1, -1.5, 1.7),
			octomath::Vector3(11.1, -1.7, 0.1),
			octomath::Vector3(11.1, 0.1, 0.1),
			octomath::Vector3(11.3, -0.1, 0.1),
			octomath::Vector3(9.5, -0.1, 0.1),
			octomath::Vector3(10.9, -0.1, -0.1),
			octomath::Vector3(10.9, -0.1, 1.7),
			octomath::Vector3(10.9, -1.7, 1.5),
			octomath::Vector3(10.9, 0.1, 1.5),
			octomath::Vector3(11.3, -0.3, 1.5),
			octomath::Vector3(10.9, -1.7, 1.3),
			octomath::Vector3(10.9, 0.1, 1.3),
			octomath::Vector3(11.3, -0.3, 1.3),
			octomath::Vector3(9.5, -0.3, 1.3),
			octomath::Vector3(10.9, -0.5, -0.1),
			octomath::Vector3(10.9, -0.5, 1.7),
			octomath::Vector3(10.9, -1.7, 1.1),
			octomath::Vector3(10.9, 0.1, 1.1),
			octomath::Vector3(11.3, -0.3, 1.1),
			octomath::Vector3(9.5, -0.3, 1.1),
			octomath::Vector3(10.9, -0.7, -0.1),
			octomath::Vector3(10.9, -0.7, 1.7),
			octomath::Vector3(10.9, -1.7, 0.9),
			octomath::Vector3(10.9, 0.1, 0.9),
			octomath::Vector3(11.3, -0.3, 0.9),
			octomath::Vector3(9.5, -0.3, 0.9),
			octomath::Vector3(10.9, -0.9, -0.1),
			octomath::Vector3(10.9, -0.9, 1.7),
			octomath::Vector3(10.9, -1.7, 0.7),
			octomath::Vector3(10.9, 0.1, 0.7),
			octomath::Vector3(11.3, -0.3, 0.7),
			octomath::Vector3(9.5, -0.3, 0.7),
			octomath::Vector3(10.9, -1.1, -0.1),
			octomath::Vector3(10.9, -1.1, 1.7),
			octomath::Vector3(10.9, -1.7, 0.5),
			octomath::Vector3(10.9, 0.1, 0.5),
			octomath::Vector3(11.3, -0.3, 0.5),
			octomath::Vector3(9.5, -0.3, 0.5),
			octomath::Vector3(10.9, -1.3, -0.1),
			octomath::Vector3(10.9, -1.3, 1.7),
			octomath::Vector3(10.9, -1.7, 0.3),
			octomath::Vector3(10.9, 0.1, 0.3),
			octomath::Vector3(11.3, -0.3, 0.3),
			octomath::Vector3(9.5, -0.3, 0.3),
			octomath::Vector3(10.9, -1.5, -0.1),
			octomath::Vector3(10.9, -1.5, 1.7),
			octomath::Vector3(9.5, -0.3, 0.1),
			octomath::Vector3(10.7, -0.1, -0.1),
			octomath::Vector3(10.7, -0.1, 1.7),
			octomath::Vector3(10.7, -1.7, 1.5),
			octomath::Vector3(10.7, 0.1, 1.5),
			octomath::Vector3(11.3, -0.5, 1.5),
			octomath::Vector3(9.5, -0.5, 1.5),
			octomath::Vector3(10.7, -0.3, -0.1),
			octomath::Vector3(10.7, -0.3, 1.7),
			octomath::Vector3(10.7, -1.7, 1.3),
			octomath::Vector3(10.7, 0.1, 1.3),
			octomath::Vector3(11.3, -0.5, 1.3),
			octomath::Vector3(9.5, -0.5, 1.3),
			octomath::Vector3(10.7, -0.5, -0.1),
			octomath::Vector3(10.7, -0.5, 1.7),
			octomath::Vector3(9.5, -0.7, 0.3),
			octomath::Vector3(11.1, -1.1, -0.1),
			octomath::Vector3(9.5, -1.1, 0.5),
			octomath::Vector3(10.5, -0.3, 1.7),
			octomath::Vector3(9.5, -0.9, 0.5),
			octomath::Vector3(11.1, -1.1, 1.7),
			octomath::Vector3(10.1, -1.3, -0.1),
			octomath::Vector3(10.5, -1.7, 1.3),
			octomath::Vector3(10.3, -1.3, -0.1),
			octomath::Vector3(11.1, -1.7, 0.5),
			octomath::Vector3(10.1, -1.3, 1.7),
			octomath::Vector3(10.5, 0.1, 1.3),
			octomath::Vector3(10.3, -1.3, 1.7),
			octomath::Vector3(11.1, 0.1, 0.5),
			octomath::Vector3(10.1, -1.7, 0.3),
			octomath::Vector3(11.3, -0.7, 1.3),
			octomath::Vector3(10.3, -1.7, 0.3),
			octomath::Vector3(11.3, -0.1, 0.5),
			octomath::Vector3(10.1, 0.1, 0.3),
			octomath::Vector3(9.5, -0.7, 1.3),
			octomath::Vector3(10.3, 0.1, 0.3),
			octomath::Vector3(10.5, -0.5, -0.1),
			octomath::Vector3(11.3, -0.9, 0.3),
			octomath::Vector3(9.9, -0.3, -0.1),
			octomath::Vector3(10.5, -0.5, 1.7),
			octomath::Vector3(9.5, -0.9, 0.3),
			octomath::Vector3(9.9, -0.3, 1.7),
			octomath::Vector3(10.5, -1.7, 1.1),
			octomath::Vector3(10.3, -1.5, -0.1),
			octomath::Vector3(9.9, -1.7, 1.3),
			octomath::Vector3(10.5, 0.1, 1.1),
			octomath::Vector3(10.3, -1.5, 1.7),
			octomath::Vector3(9.9, 0.1, 1.3),
			octomath::Vector3(10.3, -1.7, 0.1),
			octomath::Vector3(11.3, -1.3, 1.3),
			octomath::Vector3(10.3, 0.1, 0.1),
			octomath::Vector3(9.5, -1.3, 1.3),
			octomath::Vector3(11.3, -0.9, 0.1),
			octomath::Vector3(9.9, -0.5, -0.1),
			octomath::Vector3(9.5, -0.9, 0.1),
			octomath::Vector3(9.9, -0.5, 1.7),
			octomath::Vector3(10.1, -0.1, -0.1),
			octomath::Vector3(9.9, -1.7, 1.1),
			octomath::Vector3(10.1, -0.1, 1.7),
			octomath::Vector3(9.9, 0.1, 1.1),
			octomath::Vector3(10.1, -1.7, 1.5),
			octomath::Vector3(11.3, -1.3, 1.1),
			octomath::Vector3(10.1, 0.1, 1.5),
			octomath::Vector3(9.5, -1.3, 1.1),
			octomath::Vector3(11.3, -1.1, 1.5),
			octomath::Vector3(9.9, -0.7, -0.1),
			octomath::Vector3(9.5, -1.1, 1.5),
			octomath::Vector3(9.9, -0.7, 1.7),
			octomath::Vector3(10.1, -0.3, -0.1),
			octomath::Vector3(9.9, -1.7, 0.9),
			octomath::Vector3(10.1, -0.3, 1.7),
			octomath::Vector3(9.9, 0.1, 0.9),
			octomath::Vector3(10.1, -1.7, 1.3),
			octomath::Vector3(11.3, -1.3, 0.9),
			octomath::Vector3(10.1, 0.1, 1.3),
			octomath::Vector3(9.5, -1.3, 0.9),
			octomath::Vector3(11.3, -1.1, 1.3),
			octomath::Vector3(9.9, -0.9, -0.1),
			octomath::Vector3(9.5, -1.1, 1.3),
			octomath::Vector3(9.9, -0.9, 1.7),
			octomath::Vector3(10.1, -0.5, -0.1),
			octomath::Vector3(9.9, -1.7, 0.7),
			octomath::Vector3(10.1, -0.5, 1.7),
			octomath::Vector3(9.9, 0.1, 0.7),
			octomath::Vector3(10.1, -1.7, 1.1),
			octomath::Vector3(11.3, -1.3, 0.7),
			octomath::Vector3(10.1, 0.1, 1.1),
			octomath::Vector3(9.5, -1.3, 0.7),
			octomath::Vector3(11.3, -1.1, 1.1),
			octomath::Vector3(9.9, -1.1, -0.1),
			octomath::Vector3(9.5, -1.1, 1.1),
			octomath::Vector3(11.3, -0.7, 1.1),
			octomath::Vector3(9.9, -1.5, 1.7),
			octomath::Vector3(9.5, -0.7, 1.1),
			octomath::Vector3(9.9, -1.7, 0.1),
			octomath::Vector3(10.5, -0.7, -0.1),
			octomath::Vector3(9.9, 0.1, 0.1),
			octomath::Vector3(10.5, -0.7, 1.7),
			octomath::Vector3(11.3, -1.3, 0.1),
			octomath::Vector3(10.5, -1.7, 0.9),
			octomath::Vector3(9.5, -1.3, 0.1),
			octomath::Vector3(10.5, 0.1, 0.9),
			octomath::Vector3(9.7, -0.1, -0.1),
			octomath::Vector3(9.9, -1.1, 1.7),
			octomath::Vector3(10.1, -0.7, -0.1),
			octomath::Vector3(9.7, -0.1, 1.7),
			octomath::Vector3(10.5, -0.9, 1.7),
			octomath::Vector3(11.3, -1.5, 0.7),
			octomath::Vector3(10.5, -1.7, 0.7),
			octomath::Vector3(9.5, -1.5, 0.7),
			octomath::Vector3(10.5, 0.1, 0.7),
			octomath::Vector3(9.7, -1.1, -0.1),
			octomath::Vector3(11.3, -0.7, 0.7),
			octomath::Vector3(9.7, -1.1, 1.7),
			octomath::Vector3(9.5, -0.7, 0.7),
			octomath::Vector3(9.7, -1.7, 0.5),
			octomath::Vector3(10.5, -1.1, -0.1),
			octomath::Vector3(9.7, 0.1, 0.5),
			octomath::Vector3(10.5, -1.1, 1.7),
			octomath::Vector3(11.3, -1.5, 0.5),
			octomath::Vector3(10.5, -1.7, 0.5),
			octomath::Vector3(9.5, -1.5, 0.5),
			octomath::Vector3(10.5, 0.1, 0.5),
			octomath::Vector3(9.7, -1.3, -0.1),
			octomath::Vector3(10.5, -1.3, 1.7),
			octomath::Vector3(9.7, -1.3, 1.7),
			octomath::Vector3(10.5, -0.9, -0.1),
			octomath::Vector3(9.7, 0.1, 0.7),
			octomath::Vector3(9.5, -1.3, 1.5),
			octomath::Vector3(9.5, -1.5, 0.1),
			octomath::Vector3(9.9, -1.3, -0.1),
			octomath::Vector3(9.5, -1.1, 0.9),
			octomath::Vector3(9.7, -0.3, -0.1),
			octomath::Vector3(9.5, -0.7, 0.9),
			octomath::Vector3(9.7, -1.7, 0.7),
			octomath::Vector3(11.3, -1.3, 1.5),
			octomath::Vector3(11.3, -1.5, 0.1),
			octomath::Vector3(9.5, -1.3, 0.5),
			octomath::Vector3(11.3, -1.1, 0.9),
			octomath::Vector3(9.5, -1.5, 1.5),
			octomath::Vector3(9.9, 0.1, 1.5),
			octomath::Vector3(9.7, 0.1, 0.1),
			octomath::Vector3(11.3, -1.3, 0.5),
			octomath::Vector3(10.1, 0.1, 0.9),
			octomath::Vector3(11.3, -1.5, 1.5),
			octomath::Vector3(9.9, -1.7, 1.5),
			octomath::Vector3(9.7, -1.7, 0.1),
			octomath::Vector3(9.9, 0.1, 0.5),
			octomath::Vector3(10.1, -1.7, 0.9),
			octomath::Vector3(9.7, 0.1, 1.5),
			octomath::Vector3(9.9, -0.1, 1.7),
			octomath::Vector3(9.7, -1.5, 1.7),
			octomath::Vector3(9.9, -1.7, 0.5),
			octomath::Vector3(10.1, -0.7, 1.7),
			octomath::Vector3(9.7, -1.7, 1.5),
			octomath::Vector3(9.9, -0.1, -0.1),
			octomath::Vector3(9.7, -1.5, -0.1),
			octomath::Vector3(11.3, -0.7, 0.3),
			octomath::Vector3(11.3, -1.5, 0.3),
			octomath::Vector3(10.5, 0.1, 0.3),
			octomath::Vector3(9.7, 0.1, 0.3),
			octomath::Vector3(10.5, -1.7, 0.3),
			octomath::Vector3(9.7, -1.7, 0.3),
			octomath::Vector3(11.3, -0.7, 0.9),
			octomath::Vector3(9.7, -0.9, 1.7),
			octomath::Vector3(10.5, -1.3, -0.1),
			octomath::Vector3(9.7, -0.9, -0.1),
			octomath::Vector3(9.5, -0.7, 0.5),
			octomath::Vector3(9.5, -1.5, 0.9),
			octomath::Vector3(11.3, -0.7, 0.5),
			octomath::Vector3(11.3, -1.5, 0.9),
			octomath::Vector3(9.7, 0.1, 0.9),
			octomath::Vector3(9.7, -1.7, 0.9),
			octomath::Vector3(9.7, -0.7, 1.7),
			octomath::Vector3(9.7, -0.7, -0.1),
			octomath::Vector3(9.5, -1.5, 1.1),
			octomath::Vector3(11.3, -1.5, 1.1),
			octomath::Vector3(9.7, 0.1, 1.1),
			octomath::Vector3(9.7, -1.7, 1.1),
			octomath::Vector3(9.7, -0.5, 1.7),
			octomath::Vector3(9.9, -1.5, -0.1),
			octomath::Vector3(9.7, -0.5, -0.1),
			octomath::Vector3(9.5, -1.3, 0.3),
			octomath::Vector3(9.5, -1.5, 1.3),
			octomath::Vector3(11.3, -1.3, 0.3),
			octomath::Vector3(10.1, 0.1, 0.7),
			octomath::Vector3(11.3, -1.5, 1.3),
			octomath::Vector3(9.9, 0.1, 0.3),
			octomath::Vector3(10.1, -1.7, 0.7),
			octomath::Vector3(9.7, 0.1, 1.3),
			octomath::Vector3(9.9, -1.7, 0.3),
			octomath::Vector3(10.1, -0.9, 1.7),
			octomath::Vector3(9.9, -1.3, 1.7),
			octomath::Vector3(10.1, -0.9, -0.1),
			octomath::Vector3(9.7, -0.3, 1.7),
			octomath::Vector3(9.5, -1.5, 0.3),
			octomath::Vector3(9.5, -1.1, 0.1),
			octomath::Vector3(9.5, -0.1, 0.7),
			octomath::Vector3(11.3, -1.1, 0.5),
			octomath::Vector3(10.5, -0.3, -0.1),
			octomath::Vector3(11.3, -0.9, 0.5),
			octomath::Vector3(9.7, -1.7, 1.3),
			octomath::Vector3(11.3, -1.1, 0.1),
			octomath::Vector3(10.7, -1.7, 0.7),
			octomath::Vector3(10.3, -0.3, -0.1),
			octomath::Vector3(11.3, -0.3, 0.1),
			octomath::Vector3(10.1, 0.1, 0.1),
			octomath::Vector3(10.9, 0.1, 0.1),
			octomath::Vector3(10.1, -1.7, 0.1),
			octomath::Vector3(10.9, -1.7, 0.1),
			octomath::Vector3(10.1, -1.5, 1.7),
			octomath::Vector3(10.9, -0.3, 1.7),
			octomath::Vector3(10.1, -1.5, -0.1),
			octomath::Vector3(10.9, -0.3, -0.1),
			octomath::Vector3(9.5, -1.1, 0.3),
			octomath::Vector3(9.5, -0.3, 1.5),
			octomath::Vector3(11.3f, -1.1f, 0.3f),
			octomath::Vector3(11.3, -0.1, 0.7),
			octomath::Vector3(10.1, 0.1, 0.5),
			octomath::Vector3(9.5, -0.7, 1.5),
			octomath::Vector3(10.3, 0.1, 0.5),
			octomath::Vector3(11.1, 0.1, 0.7),
			octomath::Vector3(10.1, -1.7, 0.5),
			octomath::Vector3(11.3, -0.7, 1.5),
			octomath::Vector3(10.3, -1.7, 0.5),
			octomath::Vector3(11.1, -1.7, 0.7),
			octomath::Vector3(10.1, -1.1, 1.7),
			octomath::Vector3(10.5, 0.1, 1.5),
			octomath::Vector3(10.3, -1.1, 1.7),
			octomath::Vector3(11.1, -0.9, 1.7),
			octomath::Vector3(10.1, -1.1, -0.1),
			octomath::Vector3(10.5, -1.7, 1.5),
			octomath::Vector3(10.3, -1.1, -0.1),
			octomath::Vector3(11.1, -0.9, -0.1),
			octomath::Vector3(9.5, -1.1, 0.7),
			octomath::Vector3(10.5, -0.1, 1.7),
			octomath::Vector3(9.5, -0.9, 0.7),
			octomath::Vector3(9.5, -0.1, 0.9),
			octomath::Vector3(11.3, -1.1, 0.7),
			octomath::Vector3(10.5, -0.1, -0.1),
			octomath::Vector3(11.3, -0.9, 0.7),
			octomath::Vector3(9.5, -0.5, 0.1),
			octomath::Vector3(10.3, 0.1, 0.7),
			octomath::Vector3(11.3, -0.5, 0.1),
			octomath::Vector3(10.3, -1.7, 0.7),
			octomath::Vector3(10.7, 0.1, 0.1),
			octomath::Vector3(10.3, -0.9, 1.7),
			octomath::Vector3(10.7, -1.7, 0.1),
			octomath::Vector3(10.3, -0.9, -0.1),
			octomath::Vector3(10.7, -1.5, 1.7),
			octomath::Vector3(9.5, -0.9, 0.9),
			octomath::Vector3(10.7, -1.5, -0.1),
			octomath::Vector3(11.3, -0.9, 0.9),
			octomath::Vector3(9.5, -0.5, 0.3),
			octomath::Vector3(10.3, 0.1, 0.9),
			octomath::Vector3(11.3, -0.5, 0.3),
			octomath::Vector3(10.3, -1.7, 0.9),
			octomath::Vector3(10.7, 0.1, 0.3),
			octomath::Vector3(10.3, -0.7, 1.7),
			octomath::Vector3(10.7, -1.7, 0.3),
			octomath::Vector3(10.3, -0.7, -0.1),
			octomath::Vector3(10.7, -1.3, 1.7),
			octomath::Vector3(9.5, -0.9, 1.1),
			octomath::Vector3(10.7, -1.3, -0.1),
			octomath::Vector3(11.3, -0.9, 1.1),
			octomath::Vector3(9.5, -0.5, 0.5),
			octomath::Vector3(10.3, 0.1, 1.1),
			octomath::Vector3(11.3, -0.5, 0.5),
			octomath::Vector3(10.3, -1.7, 1.1),
			octomath::Vector3(10.7, 0.1, 0.5),
			octomath::Vector3(10.3, -0.5, 1.7),
			octomath::Vector3(10.7, -1.7, 0.5),
			octomath::Vector3(10.3, -0.5, -0.1),
			octomath::Vector3(10.7, -1.1, 1.7),
			octomath::Vector3(9.5, -0.9, 1.3),
			octomath::Vector3(10.7, -1.1, -0.1),
			octomath::Vector3(11.3, -0.9, 1.3),
			octomath::Vector3(9.5, -0.5, 0.7),
			octomath::Vector3(10.3, 0.1, 1.3),
			octomath::Vector3(11.3, -0.5, 0.7),
			octomath::Vector3(10.3, -1.7, 1.3),
			octomath::Vector3(10.7, 0.1, 0.7),
			octomath::Vector3(10.3, -0.3, 1.7),
			octomath::Vector3(10.7, -0.9, 1.7),
			octomath::Vector3(9.5, -0.9, 1.5),
			octomath::Vector3(10.7, -0.9, -0.1),
			octomath::Vector3(11.3, -0.9, 1.5),
			octomath::Vector3(9.5, -0.5, 0.9),
			octomath::Vector3(10.3, 0.1, 1.5),
			octomath::Vector3(11.3, -0.5, 0.9),
			octomath::Vector3(10.3, -1.7, 1.5),
			octomath::Vector3(10.7, 0.1, 0.9),
			octomath::Vector3(10.3, -0.1, 1.7),
			octomath::Vector3(10.7, -1.7, 0.9),
			octomath::Vector3(10.3, -0.1, -0.1),
			octomath::Vector3(10.7, -0.7, 1.7),
			octomath::Vector3(9.5, -0.7, 0.1),
			octomath::Vector3(10.7, -0.7, -0.1),
			octomath::Vector3(11.3, -0.7, 0.1),
			octomath::Vector3(9.5, -0.5, 1.1),
			octomath::Vector3(10.5, 0.1, 0.1),
			octomath::Vector3(11.3, -0.5, 1.1),
			octomath::Vector3(10.5, -1.7, 0.1),
			octomath::Vector3(10.7, 0.1, 1.1),
			octomath::Vector3(10.5, -1.5, 1.7),
			octomath::Vector3(10.7, -1.7, 1.1),
			octomath::Vector3(10.5, -1.5, -0.1)
		} ;
		unordered_set_pointers neighbors_us;
		octomap::OcTree octree ("data/circle_1m.bt");
		octomath::Vector3 point_coordinates (10.4f, -0.8f, 0.8f);
		octomap::OcTreeKey point_key = octree.coordToKey(point_coordinates);
		int depth = getNodeDepth_Octomap(point_key, octree);
		EXPECT_EQ(13, depth);
		double node_size = octree.getNodeSize(depth); // in meters
		EXPECT_EQ(1.6, node_size);
		float resolution = octree.getResolution();
		EXPECT_FLOAT_EQ(0.2, resolution);
		// ACT
		octomath::Vector3 cell_center_coordinates = getCellCenter(point_coordinates, octree);
		generateNeighbors_pointers(neighbors_us, cell_center_coordinates, node_size, resolution);
		// printForTesting(neighbors_us);
		// printForMatlab(neighbors_us);
		// ASSERT
		EXPECT_EQ(cell_center_coordinates, octomath::Vector3 (10.4f, -0.8f, 0.8f));	
		EXPECT_EQ(384, neighbors_us.size());
		EXPECT_EQ(384, right_answers.size());
		ASSERT_TRUE (allNeighborsAreCorrect(neighbors_us, right_answers));
	}

	TEST(OctreeNeighborTest, NeighborTest_GroundData_CellCenter)
	{
		// ARRANGE
		octomap::OcTree octree ("data/circle_1m.bt");
		octomath::Vector3 point_coordinates (0,0,0);
		// ACT
		octomath::Vector3 cell_center_coordinates = getCellCenter(point_coordinates, octree);
		// ASSERT
		ASSERT_EQ(cell_center_coordinates, octomath::Vector3 (0.1, 0.1, 0.1));
	}


	/*TEST(OctreeNeighborTest, GenerateData_FindCells)
	{
		// ARRANGE
		ros::Time::init();
		octomap::OcTree octree ("data/circle_1m.bt");
        findDifferentSizeCells_ptr_3D(octree);

        // findDifferentSizeCells_ptr_3D(octree, false);


		octomap::OcTree octree_2 ("data/double_circle_1m.bt");
        findDifferentSizeCells_ptr_3D(octree_2);


		octomap::OcTree octree_3 ("data/s_1m.bt");
        findDifferentSizeCells_ptr_3D(octree_3);

		octomap::OcTree octree_4 ("data/offShoreOil_1m.bt");
        findDifferentSizeCells_ptr_3D(octree_4);


		octomap::OcTree octree_5 ("data/offShoreOil_4obstacles_1m.bt");
        findDifferentSizeCells_ptr_3D(octree_5);
	}*/

	// TAKES VERY VERY LONG!!!
	// TEST(OctreeNeighborTest, GenerateData_CalculateVolume)
	// {
	// 	// ARRANGE
	// 	ros::Time::init();
	// 	octomap::OcTree octree ("data/circle_1m.bt");
 //        findDifferentSizeCells_ptr_3D(octree);

 //        // findDifferentSizeCells_ptr_3D(octree, false);


	// 	octomap::OcTree octree_2 ("data/double_circle_1m.bt");
 //        findDifferentSizeCells_ptr_3D(octree_2);


	// 	octomap::OcTree octree_3 ("data/s_1m.bt");
 //        findDifferentSizeCells_ptr_3D(octree_3);

	// 	octomap::OcTree octree_4 ("data/offShoreOil_1m.bt");
 //        findDifferentSizeCells_ptr_3D(octree_4);


	// 	octomap::OcTree octree_5 ("data/offShoreOil_4obstacles_1m.bt");
 //        findDifferentSizeCells_ptr_3D(octree_5);
	// }


	TEST(OctreeNeighborTest, VerifyNeighborCountTest_all)
	{
		unordered_set_pointers neighbors_us;
		octomath::Vector3 center_coords( 0, -2, 65 );
		float resolution = 0.2f;

		float node_size [4] = {0.2f, 0.4f, 0.8f, 1.6f};
		int neighbor_count_3d [4] = {6, 24, 96, 384};

		for (int i = 0; i < 4; i++)
		{
			generateNeighbors_pointers(neighbors_us, center_coords, node_size[i], resolution);
			ASSERT_EQ(neighbors_us.size(), neighbor_count_3d[i]);
			neighbors_us.clear();
		}
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
			updateToCellCenterAndFindSize(point, octree, cell_size_result, sidelength_lookup_table);
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
			updateToCellCenterAndFindSize(point, octree, cell_size_result, sidelength_lookup_table);
			// ASSERT
			EXPECT_FALSE(cell_center==point);
		}
	}

	// Check the that after using the function the pointer is to an object 
	// with the right coordinates and that the cell size is correct
	TEST(OctreeNeighborTest, updateToCellCenter_FindSize)
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
		std::list< std::shared_ptr<octomath::Vector3> > inside_points_to_test {
			std::make_shared<octomath::Vector3>  (-0.4f, 1.2f, 1.4f),
			std::make_shared<octomath::Vector3>  (-0.4f, 1.2f, 1.f),
			std::make_shared<octomath::Vector3>  (-0.4f, 1.4f, 1.2f),
			std::make_shared<octomath::Vector3>  (-0.4f, 1.f, 1.2f),
			std::make_shared<octomath::Vector3>  (-0.2f, 1.2f, 1.2f),
			std::make_shared<octomath::Vector3>  (-0.6f, 1.2f, 1.2f),
		};
		for(std::shared_ptr<octomath::Vector3> point : inside_points_to_test)
		{
			// ACT
			updatePointerToCellCenterAndFindSize(point, octree, cell_size_result, sidelength_lookup_table);
			// ASSERT
			EXPECT_TRUE(cell_center == *(point) ) << cell_center << " != " << *(point);
			EXPECT_EQ(cell_size, cell_size_result);
		}
		// OUTSIDE
		std::list < std::shared_ptr<octomath::Vector3> > outside_points_to_test {
			std::make_shared<octomath::Vector3>  (0.f, 1.2f, 1.2f),
			std::make_shared<octomath::Vector3>  (-0.4f, 1.6f, 1.2f),
			std::make_shared<octomath::Vector3>  (-0.4f, 1.2f, 1.6f),
			std::make_shared<octomath::Vector3>  (0.1f, 1.2f, 1.2f),
			std::make_shared<octomath::Vector3>  (-0.4f, 1.7f, 1.2f),
			std::make_shared<octomath::Vector3>  (-0.4f, 1.2f, 1.7f),
			std::make_shared<octomath::Vector3>  (-1.f, 3.f, 1.f),
		};
		for(std::shared_ptr<octomath::Vector3> point : outside_points_to_test)
		{
			// ACT
			updatePointerToCellCenterAndFindSize(point, octree, cell_size_result, sidelength_lookup_table);
			// ASSERT
			EXPECT_FALSE(cell_center== *(point) );
		}
	}


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

	
	TEST(SetTest, compareInsertion_insertHighestValue)
	{
		frontiers_msgs::VoxelMsg voxel_msg;
        voxel_msg.size = 1;
        voxel_msg.xyz_m.x = 0;
        voxel_msg.xyz_m.y = 0;
        voxel_msg.xyz_m.z = 0;
        Frontiers::OrderedNeighbors list (voxel_msg);
        voxel_msg.occupied_neighborhood=0;
        voxel_msg.size = 1;
        voxel_msg.xyz_m.x = 0;
        voxel_msg.xyz_m.y = 0;
        voxel_msg.xyz_m.z = 0;
		list.insert(voxel_msg);
        ASSERT_EQ(list.size(), 1);
        voxel_msg.size = 8;
        voxel_msg.occupied_neighborhood=1;
        voxel_msg.distance=34;
        voxel_msg.xyz_m.x = 0;
        voxel_msg.xyz_m.y = 0;
        voxel_msg.xyz_m.z = 0;
		list.insert( voxel_msg);
        ASSERT_EQ(list.size(), 1);
	}

	TEST(SetTest, compareInsertion_insertHighestValue_2)
	{
		frontiers_msgs::VoxelMsg voxel_msg;
        voxel_msg.size = 1;
        voxel_msg.xyz_m.x = 0;
        voxel_msg.xyz_m.y = 0;
        voxel_msg.xyz_m.z = 0;
        Frontiers::OrderedNeighbors list (voxel_msg);
        voxel_msg.occupied_neighborhood=10;
        voxel_msg.size = 1;
        voxel_msg.xyz_m.x = 0;
        voxel_msg.xyz_m.y = 0;
        voxel_msg.xyz_m.z = 0;
		list.insert( voxel_msg);
        // ASSERT_EQ(list.begin()->occupied_neighborhood, 10);
        ASSERT_EQ(list.size(), 1);
        voxel_msg.size = 8;
        voxel_msg.occupied_neighborhood=1;
        voxel_msg.distance=34;
        voxel_msg.xyz_m.x = 0;
        voxel_msg.xyz_m.y = 0;
        voxel_msg.xyz_m.z = 0;
		list.insert( voxel_msg);
        ASSERT_EQ(list.size(), 1);
        // ASSERT_EQ(list.begin()->occupied_neighborhood, 10);
	}

	TEST(SetTest, compareInsertion_insertEqual)
	{
		frontiers_msgs::VoxelMsg voxel_msg;
        voxel_msg.occupied_neighborhood=0;
        voxel_msg.size = 1;
        voxel_msg.xyz_m.x = 0;
        voxel_msg.xyz_m.y = 0;
        voxel_msg.xyz_m.z = 0;
        Frontiers::OrderedNeighbors list (voxel_msg);
        voxel_msg.occupied_neighborhood=13;
        voxel_msg.distance=34;
        voxel_msg.xyz_m.x = 0;
        voxel_msg.xyz_m.y = 20;
        voxel_msg.xyz_m.z = 0;
		list.insert( voxel_msg);
        ASSERT_EQ(list.size(), 1);
        voxel_msg.occupied_neighborhood=13;
        voxel_msg.distance=34;
        voxel_msg.xyz_m.x = 0;
        voxel_msg.xyz_m.y = 20;
        voxel_msg.xyz_m.z = 0;
		list.insert( voxel_msg);
        ASSERT_EQ(list.size(), 1);
	}
	TEST(SetTest, compareInsertion_insert)
	{
		frontiers_msgs::VoxelMsg voxel_msg;
        voxel_msg.occupied_neighborhood=0;
        voxel_msg.size = 1;
        voxel_msg.xyz_m.x = 0;
        voxel_msg.xyz_m.y = 0;
        voxel_msg.xyz_m.z = 0;
        Frontiers::OrderedNeighbors list (voxel_msg);
        voxel_msg.occupied_neighborhood=10;
        voxel_msg.size = 1;
        voxel_msg.xyz_m.x = 1;
        voxel_msg.xyz_m.y = 0;
        voxel_msg.xyz_m.z = 0;
		list.insert( voxel_msg);
        // ASSERT_EQ(list.begin()->occupied_neighborhood, 10);
        ASSERT_EQ(list.size(), 1);
        voxel_msg.size = 8;
        voxel_msg.occupied_neighborhood=1;
        voxel_msg.distance=34;
        voxel_msg.xyz_m.x = 0;
        voxel_msg.xyz_m.y = 0;
        voxel_msg.xyz_m.z = 0;
		list.insert( voxel_msg);
        ASSERT_EQ(list.size(), 2);
        voxel_msg.size = 8;
        voxel_msg.occupied_neighborhood=133;
        voxel_msg.distance=34;
        voxel_msg.xyz_m.x = 10;
        voxel_msg.xyz_m.y = 0;
        voxel_msg.xyz_m.z = 0;
		list.insert( voxel_msg);
        ASSERT_EQ(list.size(), 3);
        voxel_msg.size = 8;
        voxel_msg.occupied_neighborhood=13;
        voxel_msg.distance=34;
        voxel_msg.xyz_m.x = 0;
        voxel_msg.xyz_m.y = 20;
        voxel_msg.xyz_m.z = 0;
		list.insert( voxel_msg);
        ASSERT_EQ(list.size(), 4);
        voxel_msg.size = 8;
        voxel_msg.occupied_neighborhood=13;
        voxel_msg.distance=34;
        voxel_msg.xyz_m.x = 0;
        voxel_msg.xyz_m.y = 20;
        voxel_msg.xyz_m.z = 0;
		list.insert( voxel_msg);
        ASSERT_EQ(list.size(), 4);
	}
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}