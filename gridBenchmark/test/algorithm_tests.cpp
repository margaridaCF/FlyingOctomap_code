#include <grid_benchmark.h>
#include <gtest/gtest.h>

namespace mapper{

	class AlgorithmTest : public ::testing::Test {
	protected:
	  	AlgorithmTest() 
	  		: octree("data/circle_1m.bt"),
	  		real_min_negative(-5, -1, 0.5),
	  		real_min_positive(0, 1, 0.5)
	  	{
			std::tuple<double, double, double> min, max;
	        octree.getMetricMin(std::get<0>(min), std::get<1>(min), std::get<2>(min));
	        octree.getMetricMax(std::get<0>(max), std::get<1>(max), std::get<2>(max));
			real_min = real_min_negative;
			real_max = octomath::Vector3(0, 1, 1.5);
	  	}
		virtual ~AlgorithmTest() {}
		Voxel jumpFirstDimension(int numberOfTimes, RegularGrid& grid);
		void  testIteration(RegularGrid& grid);

		Voxel firstCell;
		octomap::OcTree octree;
		octomath::Vector3 real_min, real_max;
		octomath::Vector3 const real_min_negative;
		octomath::Vector3 const real_min_positive;
	};

	// ===== ITERATION INITIALIZATION =====

	TEST_F(AlgorithmTest, regularGrid_2d_Iterator_init_positive)
	{
		RegularGrid grid("test", Algorithm::regularGrid_2d, octree, 0.2f);
		real_min = real_min_positive;
		ASSERT_TRUE( grid.iteratorInit(real_min, real_max));
		grid.getCurrentCell(firstCell);
		ASSERT_EQ(1, firstCell.z);
		ASSERT_EQ(real_min.x(), firstCell.x);
		ASSERT_EQ(real_min.y(), firstCell.y);
	}
	TEST_F(AlgorithmTest, regularGrid_2d_Iterator_init_negative)
	{
		RegularGrid grid("test", Algorithm::regularGrid_2d, octree, 0.2f);
		real_min = real_min_negative;
		grid.iteratorInit(real_min, real_max);
		grid.getCurrentCell(firstCell);
		ASSERT_EQ(1, firstCell.z);
		ASSERT_EQ(real_min.x(), firstCell.x);
		ASSERT_EQ(real_min.y(), firstCell.y);
	}

	TEST_F(AlgorithmTest, regularGrid_3d_Iterator_init_positive)
	{
		RegularGrid grid("test", regularGrid_3d, octree, 0.2f);
		real_min = real_min_positive;
		grid.iteratorInit(real_min, real_max);
		grid.getCurrentCell(firstCell);

		ASSERT_EQ(firstCell.z, real_min.z());
		ASSERT_EQ(firstCell.x, real_min.x());
		ASSERT_EQ(firstCell.y, real_min.y());
	}

	TEST_F(AlgorithmTest, regularGrid_3d_Iterator_init_negative)
	{
		RegularGrid grid("test", regularGrid_3d, octree, 0.2f);
		real_min = real_min_negative;
		grid.iteratorInit(real_min, real_max);
		grid.getCurrentCell(firstCell);
		ASSERT_EQ(firstCell.z, real_min.z());
		ASSERT_EQ(firstCell.x, real_min.x());
		ASSERT_EQ(firstCell.y, real_min.y());
	}

	TEST_F(AlgorithmTest, sparseGrid_2d_Iterator_init_negative)
	{
		SparseGrid grid("test", sparseGrid_2d, octree, 0.2f);
		real_min = real_min_negative;
		grid.iteratorInit(real_min, real_max);
		grid.getCurrentCell(firstCell);
		ASSERT_EQ(1.f, grid.getBoundingBoxMin().z());
		ASSERT_EQ(1.1f, grid.getBoundingBoxMax().z());
		// Can't really test x and y
	}
	TEST_F(AlgorithmTest, sparseGrid_2d_Iterator_init_positive)
	{
		SparseGrid grid("test", sparseGrid_2d, octree, 0.2f);
		real_min = real_min_positive;
		grid.iteratorInit(real_min, real_max);
		ASSERT_FALSE(grid.iteratorEndReached());
		grid.getCurrentCell(firstCell);

		ASSERT_EQ(1.f, grid.getBoundingBoxMin().z());
		ASSERT_EQ(1.1f, grid.getBoundingBoxMax().z());
		// Can't really test x and y
	}

	// // Can't really test for sparseGrid_3d as it will start wherever according to data

	// // ===== ITERATION =====
	Voxel AlgorithmTest::jumpFirstDimension(int numberOfTimes, RegularGrid& grid)
	{
		int rowCount = std::ceil(std::abs(real_max.y() - real_min.y()) / grid.grid_res);

		// float remainder = fmod(real_max.y() - real_min.y(),grid.grid_res);
		// if( remainder <= grid.grid_res)
		// {
		// 	ROS_WARN_STREAM("Got in ");
		// 	++rowCount;
		// }
		// ROS_WARN_STREAM("Row count "<<rowCount<< " remainder "<< fmod(real_max.y() - real_min.y(),grid.grid_res));

		//ROS_WARN_STREAM("Row count "<<rowCount);
		Voxel nextRow;
		for (int j = 0; j < numberOfTimes; ++j)
		{
			for (int i = 0 ; i <= rowCount; ++i) 
			{
				grid.iteratorNext();
				grid.getCurrentCell(nextRow);
				//ROS_WARN_STREAM(nextRow);
			}
			//ROS_WARN_STREAM(nextRow);
		}
		//ROS_WARN_STREAM("After jumping a row ("<< rowCount <<" iterations) "<< numberOfTimes << " times:  " <<  nextRow);
		//ROS_WARN_STREAM(" ");
		return nextRow;
	}

	void AlgorithmTest::testIteration(RegularGrid& grid)
	{
		//ROS_WARN_STREAM("Boundaries  [" <<  real_min << "]  -  [" << real_max << "]");
		Voxel cell;
		grid.iteratorInit(real_min, real_max);
		grid.getCurrentCell(cell);
		real_min = grid.getBoundingBoxMin();
		real_max = grid.getBoundingBoxMax();
		//ROS_WARN_STREAM("First cell " <<  cell);

		// iteration in y
		Voxel secondCell;
		grid.iteratorNext();
		grid.getCurrentCell(secondCell);
		//ROS_WARN_STREAM("Second cell " <<  secondCell);
		ASSERT_EQ(secondCell.z, grid.getBoundingBoxMin().z());
		ASSERT_EQ(secondCell.x, real_min.x());
		float diff = std::abs( secondCell.y - (real_min.y()+secondCell.size) );
		ASSERT_LT(diff, 0.001 );
		
		// Iteration in x
		Voxel nextRow_2ndCell = jumpFirstDimension(1, grid);
		//ROS_WARN_STREAM("Second cell " <<  secondCell << ", nextRow_2ndCell "<< nextRow_2ndCell);
		ASSERT_EQ(secondCell.y, nextRow_2ndCell.y);
		ASSERT_EQ(secondCell.z, grid.getBoundingBoxMin().z());
		diff = std::abs( nextRow_2ndCell.x - (secondCell.x+secondCell.size) );
		ASSERT_LT(diff, 0.001 );

		// Iteration in z
		//cell = nextRow;
		int columnCount = std::abs(real_max.x() - real_min.x()) / grid.grid_res;
		//ROS_WARN_STREAM("Before z test " <<  cell << ". Column count "<<columnCount);
		Voxel y2_x2_z2 = jumpFirstDimension(columnCount, grid);
		ASSERT_EQ(y2_x2_z2.x, real_min.x());
		ASSERT_EQ(y2_x2_z2.y, nextRow_2ndCell.y);
		diff = std::abs( y2_x2_z2.z - (nextRow_2ndCell.z+grid.grid_res) );
		ASSERT_LT(diff, 0.001 );
	}

	// // Exact match of the max does not work
	// /*TEST_F(AlgorithmTest, regularGrid_2d_Iterator_iteration)
	// {
	// 	RegularGrid grid("test", Algorithm::regularGrid_2d, octree, 1);
	// 	testIteration(grid);
	// }
	// TEST_F(AlgorithmTest, regularGrid_3d_Iterator_iteration)
	// {
	// 	RegularGrid grid("test", Algorithm::regularGrid_3d, octree, 1);
	// 	testIteration(grid);
	// }*/

	TEST_F(AlgorithmTest, regularGrid_2d_Iterator_iteration_decimalResolution)
	{
		RegularGrid grid("test", Algorithm::regularGrid_2d, octree, 0.2f);
		testIteration(grid);
	}
	TEST_F(AlgorithmTest, regularGrid_3d_Iterator_iteration_decimalResolution)
	{
		RegularGrid grid("test", Algorithm::regularGrid_3d, octree, 0.2f);
		testIteration(grid);
	}

	TEST_F(AlgorithmTest, sparseGrid_2d_Iterator_iterationFirst)
	{
		SparseGrid grid("test", sparseGrid_2d, octree, 0.2f);
		real_min = real_min_negative;
		Voxel firstCell;
		grid.iteratorInit(real_min, real_max);
		grid.getCurrentCell(firstCell);
		Voxel secondCell;
		grid.iteratorNext();
		grid.getCurrentCell(secondCell);
		ASSERT_LT(firstCell.x, secondCell.x);
		ASSERT_EQ(firstCell.y, secondCell.y);
		ASSERT_EQ(firstCell.z, secondCell.z);
	}
	TEST_F(AlgorithmTest, sparseGrid_3d_Iterator_iterationFirst)
	{
		SparseGrid grid("test", sparseGrid_3d, octree, 0.2f);
		real_min = real_min_negative;
		Voxel firstCell;
		grid.iteratorInit(real_min, real_max);
		grid.getCurrentCell(firstCell);
		Voxel secondCell;
		grid.iteratorNext();
		grid.getCurrentCell(secondCell);
		ASSERT_LT(firstCell.x, secondCell.x);
		ASSERT_EQ(firstCell.y, secondCell.y);
		ASSERT_EQ(firstCell.z, secondCell.z);
	}

	TEST_F(AlgorithmTest, sparseGrid_2d_Iterator_iterationNextLevel)
	{
		SparseGrid grid("test", sparseGrid_2d, octree, 0.2f);
		real_min = real_min_negative;
		Voxel firstCell;
		grid.iteratorInit(real_min, real_max);
		grid.getCurrentCell(firstCell);

		Voxel cell;
		for (int i = 0; i < 5; ++i)
		{
			grid.iteratorNext();
			grid.getCurrentCell(cell);
			//ROS_WARN_STREAM(cell);
		}
		ASSERT_LT(firstCell.y, cell.y);
		ASSERT_LT(firstCell.z, cell.z);
	}
	TEST_F(AlgorithmTest, sparseGrid_3d_Iterator_iterationNextLevel)
	{
		SparseGrid grid("test", sparseGrid_2d, octree, 0.2f);
		real_min = real_min_negative;
		Voxel firstCell;
		grid.iteratorInit(real_min, real_max);
		grid.getCurrentCell(firstCell);

		Voxel cell;
		for (int i = 0; i < 5; ++i)
		{
			grid.iteratorNext();
			grid.getCurrentCell(cell);
			//ROS_WARN_STREAM(cell);
		}
		ASSERT_LT(firstCell.y, cell.y);
		ASSERT_LT(firstCell.z, cell.z);
	}

	// ===== END CONDITION =====
	TEST_F(AlgorithmTest, regularGrid_2d_Iterator_end)
	{
		RegularGrid grid("test", Algorithm::regularGrid_2d, octree, 0.2f);
		Voxel cell;
		grid.iteratorInit(real_min, real_max);
		grid.getCurrentCell(cell);


		real_min = grid.getBoundingBoxMin();
		real_max = grid.getBoundingBoxMax();


		int columnCount = std::abs(real_max.x() - real_min.x()) / grid.grid_res;
		//ROS_WARN_STREAM("Boundaries  [" <<  real_min << "]  -  [" << real_max << "]. Column count "<<columnCount);
		Voxel finalCell = jumpFirstDimension(columnCount+1, grid);
		//ROS_WARN_STREAM("Final cell "<<finalCell);
		ASSERT_EQ(finalCell.x, real_min.x());
		ASSERT_EQ(finalCell.y, real_min.y());
		float diff = std::abs( finalCell.z - (real_min.z()+grid.grid_res) );
		ASSERT_LT(diff, 0.001 );

		ASSERT_TRUE(grid.iteratorEndReached());
	}

	TEST_F(AlgorithmTest, sparseGrid_2d_Iterator_startNotEnd)
	{
		SparseGrid grid("test", sparseGrid_2d, octree, 0.2f);
		octomath::Vector3 min (-5, -1, 0.5);
		octomath::Vector3 max (0, 1, 1.5);
		Voxel firstCell;
		grid.iteratorInit(min, max);
		grid.getCurrentCell(firstCell);
		ASSERT_FALSE (grid.iteratorEndReached());
	}

	TEST_F(AlgorithmTest, regularGrid_2d_Iterator_endFalse)
	{
		RegularGrid grid("test", Algorithm::regularGrid_2d, octree, 0.2f);
		Voxel cell;
		grid.iteratorInit(real_min, real_max);
		grid.getCurrentCell(cell);

		ASSERT_FALSE(grid.iteratorEndReached());
	}
	TEST_F(AlgorithmTest, regularGrid_3d_Iterator_end)
	{
		RegularGrid grid("test", Algorithm::regularGrid_3d, octree, 0.2f);
		Voxel cell;
		grid.iteratorInit(real_min, real_max);
		grid.getCurrentCell(cell);
		real_min = grid.getBoundingBoxMin();
		real_max = grid.getBoundingBoxMax();
		int i;
		for ( i = 0; i<=1430; ++i)
		{
			grid.iteratorNext();
			grid.getCurrentCell(cell);
		}
		//ROS_WARN_STREAM("Stopped on cell "<<cell<<", after "<<i<<" iterations.");
		ASSERT_TRUE(grid.iteratorEndReached());
		ASSERT_TRUE(cell.z >= real_max.z());
	}


	// // ===== CALCULATE NEIGHBOR =====
	TEST_F(AlgorithmTest, regularGrid_neighbors_x)
	{
		float size = 0.2f;
		RegularGrid grid("test", Algorithm::regularGrid_2d, octree, size);

		octomath::Vector3 grid_coordinates_curr = octomath::Vector3(1, 0, 0);
        octomath::Vector3 direction = octomath::Vector3(1, 0, 0);
        octomath::Vector3 neighbor = grid.calculateNeighbor(grid_coordinates_curr, direction, size);
        
        ASSERT_EQ (grid_coordinates_curr.x()+(direction.x()*size), neighbor.x());
        ASSERT_EQ (grid_coordinates_curr.y(), neighbor.y());
        ASSERT_EQ (grid_coordinates_curr.z(), neighbor.z());
	}
	TEST_F(AlgorithmTest, regularGrid_neighbors_y)
	{
		float size = 0.2f;
		RegularGrid grid("test", Algorithm::regularGrid_2d, octree, size);

		octomath::Vector3 grid_coordinates_curr = octomath::Vector3(0, 1, 0);
        octomath::Vector3 direction = octomath::Vector3(0, 1, 0);
        octomath::Vector3 neighbor = grid.calculateNeighbor(grid_coordinates_curr, direction, size);
        //ROS_WARN_STREAM("NEIGHBOR "<<neighbor);
        //ROS_WARN_STREAM(grid_coordinates_curr.y() << "+(" << direction.y() << "*" << size << ")");
        ASSERT_EQ (grid_coordinates_curr.x(), neighbor.x());
        ASSERT_EQ (grid_coordinates_curr.y()+(direction.y()*size), neighbor.y());
        ASSERT_EQ (grid_coordinates_curr.z(), neighbor.z());
	}
	TEST_F(AlgorithmTest, regularGrid_neighbors_z)
	{
		float size = 0.2f;
		RegularGrid grid("test", Algorithm::regularGrid_2d, octree, size);

		octomath::Vector3 grid_coordinates_curr = octomath::Vector3(0, 0, 1);
        octomath::Vector3 direction = octomath::Vector3(0, 0, 1);
        octomath::Vector3 neighbor = grid.calculateNeighbor(grid_coordinates_curr, direction, size);
        
        ASSERT_EQ (grid_coordinates_curr.x(), neighbor.x());
        ASSERT_EQ (grid_coordinates_curr.y(), neighbor.y());
        ASSERT_EQ (grid_coordinates_curr.z()+(direction.z()*size), neighbor.z());
	}
	TEST_F(AlgorithmTest, sparseGrid_neighbors_x)
	{
		float size = 1.2f;
		SparseGrid grid("test", Algorithm::regularGrid_2d, octree, 0.2f);

		octomath::Vector3 grid_coordinates_curr = octomath::Vector3(1, 0, 0);
        octomath::Vector3 direction = octomath::Vector3(1, 0, 0);
        octomath::Vector3 neighbor = grid.calculateNeighbor(grid_coordinates_curr, direction, size);
        
        float offset = (size / 2) +0.001;

        ASSERT_EQ (grid_coordinates_curr.x()+offset, neighbor.x());
        ASSERT_EQ (grid_coordinates_curr.y(), neighbor.y());
        ASSERT_EQ (grid_coordinates_curr.z(), neighbor.z());
	}
	TEST_F(AlgorithmTest, sparseGrid_neighbors_y)
	{
		float size = 1.2f;
		SparseGrid grid("test", Algorithm::regularGrid_2d, octree, 0.2f);

		octomath::Vector3 grid_coordinates_curr = octomath::Vector3(0, 1 , 0);
        octomath::Vector3 direction = octomath::Vector3(0, 1, 0);
        octomath::Vector3 neighbor = grid.calculateNeighbor(grid_coordinates_curr, direction, size);
        
        float offset = (size / 2) +0.001;

        ASSERT_EQ (grid_coordinates_curr.x(), neighbor.x());
        ASSERT_EQ (grid_coordinates_curr.y()+offset, neighbor.y());
        ASSERT_EQ (grid_coordinates_curr.z(), neighbor.z());
	}
	TEST_F(AlgorithmTest, sparseGrid_neighbors_z)
	{
		float size = 1.2f;
		SparseGrid grid("test", Algorithm::regularGrid_2d, octree, 0.2f);

		octomath::Vector3 grid_coordinates_curr = octomath::Vector3(0, 0, 1 );
        octomath::Vector3 direction = octomath::Vector3(0, 0, 1);
        octomath::Vector3 neighbor = grid.calculateNeighbor(grid_coordinates_curr, direction, size);
        
        float offset = (size / 2) +0.001;

        ASSERT_EQ (grid_coordinates_curr.x(), neighbor.x());
        ASSERT_EQ (grid_coordinates_curr.y(), neighbor.y());
        ASSERT_EQ (grid_coordinates_curr.z()+offset, neighbor.z());
	}


}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}