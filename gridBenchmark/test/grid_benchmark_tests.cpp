#include <grid_benchmark.h>
#include <gtest/gtest.h>

namespace mapper{

	class GridBenchmarkTest : public ::testing::Test {
	protected:
	  	GridBenchmarkTest() 
	  		: octree("data/circle_1m.bt"),
	  		correctResult_regular_2D(71205, 2848.2, 245,   9.8f, 0.0639509f, circle, regularGrid_2d),
	  		correctResult_sparse_2D ( 1208, 215.12, 226, 10.48f, 0.0108043f, circle, sparseGrid_2d),
	  		correctResult_regular_3D (356025, 2848.2, 12568, 100.544, 0.418576f, circle, regularGrid_3d),
	  		correctResult_sparse_3D_thrun  ( 24579, 590.48, 18631, 167.64f, 0.228566f, circle, sparseGrid_3d)
	  	{
	  	}
		virtual ~GridBenchmarkTest() {}
		bool isResultMatch(AlgorithmObject & grid, 
			Directions dimensions, ResultSet const& correctResult);

		void runAllCombionationsCrossCheck(std::string fileName, Scenario scenario);
		octomap::OcTree octree;
		octomath::Vector3 real_max, real_min, real_max_sparse3D, real_min_sparse3D;
		ResultSecretary storage;
		// cellIterations, scenarioArea, frontierCellsFound, frontierArea,  executionTime
		ResultSet correctResult_regular_2D, correctResult_regular_3D, correctResult_sparse_2D, correctResult_sparse_3D, correctResult_sparse_3D_thrun ;
	};

	bool GridBenchmarkTest::isResultMatch(AlgorithmObject & grid, 
		Directions dimensions, ResultSet const& correctResult)
	{
		GridBenchmark generic(circle, "circle", octree);
		ResultSet result = generic.findFrontierCells(real_max, real_min, dimensions, grid);		
		return result.compareResults(correctResult, true);
	}

	void GridBenchmarkTest::runAllCombionationsCrossCheck(std::string fileName, Scenario scenario)
	{
		std::tuple<double, double, double> min, max;
        octree.getMetricMin(std::get<0>(min), std::get<1>(min), std::get<2>(min));
        octree.getMetricMax(std::get<0>(max), std::get<1>(max), std::get<2>(max));
		real_max  =octomath::Vector3 (std::get<0>(max)-55, std::get<1>(max)-50, 1.5);
		real_min = octomath::Vector3(std::get<0>(min)+60, std::get<1>(min)+40, 0.5);


		// This is for verifing the results - data analysis the max z should be 1.5
	  	real_max_sparse3D = octomath::Vector3 (std::get<0>(max)-55, std::get<1>(max)-50, 1.6);
		real_min_sparse3D = octomath::Vector3 (std::get<0>(min)+60, std::get<1>(min)+40, 0.5);


		octomap::OcTree octree ("data/"+fileName+".bt");
		octomath::Vector3 maxForDraw (real_max.x(), real_max.x(), 1);
		octomath::Vector3 minForDraw (real_min.x(), real_min.x(), 1);
		GridBenchmark generic(scenario, fileName, octree);


	  	RegularGrid grid_regular_2D("test", Algorithm::regularGrid_2d, octree, 0.2f); 
		ResultSet result_regular_2D = generic.findFrontierCells(real_max, real_min, twoD, grid_regular_2D);	
		storage.storeAlgorithmResults(result_regular_2D);
		generic.drawAllLevels("OctomapFrontier_RegularGrid_2D_"+fileName, maxForDraw, minForDraw, 0.2f);
	  	RegularGrid grid_regular_3D("test", Algorithm::regularGrid_3d, octree, 0.2f);  
	  	ResultSet result_regular_3D = generic.findFrontierCells(real_max, real_min, threeD, grid_regular_3D);
		storage.storeAlgorithmResults(result_regular_3D);
		generic.drawAllLevels("OctomapFrontier_RegularGrid_3D_"+fileName, real_max, real_min, 0.2f);



	  	SparseGrid grid2D("test", Algorithm::sparseGrid_2d, octree, 0.2f); 
		ResultSet result_sparse_2D = generic.findFrontierCells(real_max, real_min, twoD, grid2D);
		storage.storeAlgorithmResults(result_sparse_2D);
		generic.drawAllLevels("OctomapFrontier_SparseGrid_2D_"+fileName, maxForDraw, minForDraw, 0.2f);
	  	SparseGrid grid3D("test", Algorithm::sparseGrid_3d, octree, 0.2f);  
	  	ResultSet result_sparse_3D = generic.findFrontierCells(real_max, real_min, threeD, grid3D);
		generic.drawAllLevels("OctomapFrontier_SparseGrid_3D_circle_"+fileName, real_max, real_min, 0.2f);
		storage.storeAlgorithmResults(result_sparse_3D);
	}

	// These are individual test that are actually check together to take less time
	/*TEST_F(GridBenchmarkTest, GridBenchmarkTest_Regular_2D_findFrontiers)
	{
	  	RegularGrid grid("test", Algorithm::regularGrid_2d, octree, 0.2f);  
		//ResultSet correctResult (71205, 2848.2, 245, 9.8f, 0.0639509, circle, regularGrid_2d);
		ASSERT_TRUE(isResultMatch(grid, twoD, correctResult_regular_2D));
	}

	TEST_F(GridBenchmarkTest, GridBenchmarkTest_Regular_3D_findFrontiers)
	{
	  	RegularGrid grid("test", Algorithm::regularGrid_3d, octree, 0.2f);  
		//ResultSet correctResult (356025, 2848.2, 12568, 100.544, 0.418576, circle, regularGrid_3d);
		ASSERT_TRUE(isResultMatch(grid, threeD, correctResult_regular_3D));
	}*/

	/*TEST_F(GridBenchmarkTest, GridBenchmarkTest_Regular_dimensionComparison)
	{
		GridBenchmark generic(circle, "circle", octree);

	  	// 2D
	  	RegularGrid grid2D("test", Algorithm::regularGrid_2d, octree, 0.2f); 
		ResultSet result2D = generic.findFrontierCells(real_max, real_min, twoD, grid2D);		
		ASSERT_TRUE( result2D.compareResults(correctResult_regular_2D) );

	  	// 3D
	  	RegularGrid grid3D("test", Algorithm::regularGrid_3d, octree, 0.2f);  
	  	ResultSet result3D = generic.findFrontierCells(real_max, real_min, threeD, grid3D);
	  	ASSERT_TRUE( result3D.compareResults(correctResult_regular_3D) );

	  	// Find frontier cells & check they are sound
		// --- Expected changes 2D vs 3D @ Regular ---
		// More cell iterations in 3D
		ASSERT_GT(result3D.cellIterations, result2D.cellIterations);
		// More frontier cells in 3D
		ASSERT_GT(result3D.frontierCellsFound, result2D.frontierCellsFound);
		// Higher execution time in 3D
		ASSERT_GT(result3D.executionTime, result2D.executionTime);
	}*/

	/*TEST_F(GridBenchmarkTest, GridBenchmarkTest_sparse_2D_findFrontiers)
	{
	  	SparseGrid grid("test", Algorithm::sparseGrid_2d, octree, 0.2f);  
		ASSERT_TRUE(isResultMatch(grid, twoD, correctResult_sparse_2D));
	}
	
	TEST_F(GridBenchmarkTest, GridBenchmarkTest_Sparse_3D_findFrontiers)
	{
	  	SparseGrid grid("test", Algorithm::sparseGrid_3d, octree, 0.2f);  
		ASSERT_TRUE(isResultMatch(grid, threeD, correctResult_sparse_3D));
	}*/

	/*TEST_F(GridBenchmarkTest, GridBenchmarkTest_Sparse_dimensionComparison)
	{
		GridBenchmark generic(circle, "circle", octree);

	  	// 2D
	  	SparseGrid grid2D("test", Algorithm::sparseGrid_2d, octree, 0.2f); 
		ResultSet result2D = generic.findFrontierCells(real_max, real_min, twoD, grid2D);		
		ASSERT_TRUE( result2D.compareResults(correctResult_sparse_2D) );

	  	// 3D
	  	SparseGrid grid3D("test", Algorithm::sparseGrid_3d, octree, 0.2f);  
	  	ResultSet result3D = generic.findFrontierCells(real_max, real_min, threeD, grid3D);
	  	ASSERT_TRUE( result3D.compareResults(correctResult_sparse_3D) );

	  	// Find frontier cells & check they are sound
		// --- Expected changes 2D vs 3D @ Sparse ---
		// More cell iterations in 3D
		ASSERT_GT(result3D.cellIterations, result2D.cellIterations);
		// More frontier cells in 3D
		ASSERT_GT(result3D.frontierCellsFound, result2D.frontierCellsFound);
		// Higher execution time in 3D
		ASSERT_GT(result3D.executionTime, result2D.executionTime);
		// More area in 3D
		ASSERT_GT(result3D.scenarioArea, result2D.scenarioArea);
	}*/

	TEST_F(GridBenchmarkTest, GridBenchmarkTest_Sparse_dimensionComparison)
	{
		GridBenchmark generic(circle, "circle", octree);

	  	// 2D
	  	SparseGrid grid2D("test", Algorithm::sparseGrid_2d, octree, 0.2f); 
		ResultSet result2D = generic.findFrontierCells(real_max, real_min, twoD, grid2D);		
		ASSERT_TRUE( result2D.compareResults(correctResult_sparse_2D) );

	  	// 3D
	  	SparseGrid grid3D("test", Algorithm::sparseGrid_3d, octree, 0.2f);  
	  	ResultSet result3D = generic.findFrontierCells(real_max_sparse3D, real_min_sparse3D, threeD, grid3D);
	  	ASSERT_TRUE( result3D.compareResults(correctResult_sparse_3D_thrun) );

		
		// --- Expected changes Sparse vs Sparse @ 2D ---
		// Less or equal frontier cells in sparse
		// 1m difference in frontier cell coverage
		// Less or equal iteration in sparse --> by a factor of ____
		// Less area covered --> by a factor of  ____
		// Less execution time --> by a factor of ____ult_sparse_2D));
		// --- Expected changes Sparse vs Sparse @ 3D ---
		// Less or equal frontier cells in sparse
		// 1m difference in frontier cell coverage
		// Less or equal iteration in sparse --> by a factor of ____
		// Less area covered --> by a factor of  ____
		// Less execution time --> by a factor of ____
	}

	/*TEST_F(GridBenchmarkTest, GridBenchmarkTest_AllSmallScenariosHaveSameDimensions)
	{
		runAllCombionations("circle_1m", circle);
		runAllCombionations("double_circle_1m", double_circle);
		runAllCombionations("s_1m", s);

		ASSERT_EQ(storage.getResult(circle, regularGrid_2d).scenarioArea, storage.getResult(double_circle, regularGrid_2d).scenarioArea);
		ASSERT_EQ(storage.getResult(s, regularGrid_2d).scenarioArea, storage.getResult(double_circle, regularGrid_2d).scenarioArea);
		ASSERT_EQ(storage.getResult(circle, regularGrid_2d).scenarioArea, storage.getResult(s, regularGrid_2d).scenarioArea);

		ASSERT_EQ(storage.getResult(circle, regularGrid_3d).scenarioArea, storage.getResult(double_circle, regularGrid_3d).scenarioArea);
		ASSERT_EQ(storage.getResult(s, regularGrid_3d).scenarioArea, storage.getResult(double_circle, regularGrid_3d).scenarioArea);
		ASSERT_EQ(storage.getResult(circle, regularGrid_3d).scenarioArea, storage.getResult(s, regularGrid_3d).scenarioArea);
	}*/
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}