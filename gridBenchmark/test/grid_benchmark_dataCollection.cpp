#include <grid_benchmark.h>
#include <gtest/gtest.h>

namespace mapper{

	class DataCollection : public ::testing::Test {
	protected:
	  	DataCollection() 
	  	: 
	  		correctResult_regular_2D(71205, 2848.2, 245,   9.8f, 0.0639509f, circle, regularGrid_2d),
	  		correctResult_sparse_2D ( 1208, 215.12, 226, 10.48f, 0.0108043f, circle, sparseGrid_2d),
	  		correctResult_regular_3D (356025, 2848.2, 12568, 100.544, 0.418576f, circle, regularGrid_3d),
	  		correctResult_sparse_3D  ( 19876, 432.68, 14997, 134.498f, 0.207306f, circle, sparseGrid_3d)
	  	{

			

	  	}

		virtual ~DataCollection() {
		// You can do clean-up work that doesn't throw exceptions here.
		}

		void runAllCombinations(std::string fileName, Scenario scenario);
		ResultSecretary storage;
		ResultSet correctResult_regular_2D, correctResult_regular_3D, correctResult_sparse_2D, correctResult_sparse_3D ;
	};


	/*TEST_F(DataCollection, RunCircle)
	{
		// If I uncomment this there were structural changes. Here is what you should need:
		octomath::Vector3 real_max, real_min, draw_2D_max, draw_2D_min, draw_3D_max, draw_3D_min;
			octomap::OcTree octree ("data/circle_1m.bt"),
			std::tuple<double, double, double> min, max;
	        octree.getMetricMin(std::get<0>(min), std::get<1>(min), std::get<2>(min));
	        octree.getMetricMax(std::get<0>(max), std::get<1>(max), std::get<2>(max));
			real_max  =octomath::Vector3 (std::get<0>(max)-55, std::get<1>(max)-50, 1);
			real_min = octomath::Vector3(std::get<0>(min)+60, std::get<1>(min)+40, 0);

		std::list<Scenario> scenarioOrder = {Scenario::circle};
		bool printTables = false;


		octomath::Vector3 maxForDraw (real_max.x(), real_max.x(), 1);
		octomath::Vector3 minForDraw (real_min.x(), real_min.x(), 1);
		GridBenchmark generic(circle, "circle", octree);


	  	RegularGrid grid_regular_2D("test", Algorithm::regularGrid_2d, octree, 0.2f); 
		ResultSet result_regular_2D = generic.findFrontierCells(real_max, real_min, twoD, grid_regular_2D);	
		storage.storeAlgorithmResults(result_regular_2D);
		generic.drawAllLevels("OctomapFrontier_RegularGrid_2D_circle_1m", maxForDraw, minForDraw);
		ASSERT_TRUE( result_regular_2D.compareResults(correctResult_regular_2D) );

	  	RegularGrid grid_regular_3D("test", Algorithm::regularGrid_3d, octree, 0.2f);  
	  	ResultSet result_regular_3D = generic.findFrontierCells(real_max, real_min, threeD, grid_regular_3D);
		storage.storeAlgorithmResults(result_regular_3D);
		generic.drawAllLevels("OctomapFrontier_RegularGrid_3D_circle_1m", real_max, real_min);
		ASSERT_TRUE( result_regular_3D.compareResults(correctResult_regular_3D) );



	  	SparseGrid grid2D("test", Algorithm::sparseGrid_2d, octree, 0.2f); 
		ResultSet result_sparse_2D = generic.findFrontierCells(real_max, real_min, twoD, grid2D);
		storage.storeAlgorithmResults(result_sparse_2D);
		generic.drawAllLevels("OctomapFrontier_SparseGrid_2D_circle_1m", maxForDraw, minForDraw);
		ASSERT_TRUE( result_sparse_2D.compareResults(correctResult_sparse_2D) );
	  	SparseGrid grid3D("test", Algorithm::sparseGrid_3d, octree, 0.2f);  
	  	ResultSet result_sparse_3D = generic.findFrontierCells(real_max, real_min, threeD, grid3D);
		generic.drawAllLevels("OctomapFrontier_SparseGrid_3D_circle_1m", real_max, real_min);
		storage.storeAlgorithmResults(result_sparse_3D);
		ASSERT_TRUE( result_sparse_3D.compareResults(correctResult_sparse_3D) );





		storage.exportCsv(scenarioOrder, printTables, "circle");
	}*/


	void DataCollection::runAllCombinations(std::string fileName, Scenario scenario)
	{
		ROS_WARN_STREAM(" -- "+fileName+" --");
		octomath::Vector3 real_max, real_min, draw_2D_max, draw_2D_min, draw_3D_max, draw_3D_min;
		octomap::OcTree octree ("data/"+fileName+".bt");
		float grid_resolution = octree.getResolution();

		std::tuple<double, double, double> min, max;
        octree.getMetricMin(std::get<0>(min), std::get<1>(min), std::get<2>(min));
        octree.getMetricMax(std::get<0>(max), std::get<1>(max), std::get<2>(max));
		real_max  =octomath::Vector3 (std::get<0>(max), std::get<1>(max), 1);
		real_min = octomath::Vector3(std::get<0>(min), std::get<1>(min), 0);
		//real_max  =octomath::Vector3 (2, 5, 1.5);
		//real_min = octomath::Vector3(0, 0, 1);
		draw_2D_max  =octomath::Vector3 (real_max.x(), real_max.y(), 1.1);
		draw_2D_min = octomath::Vector3(real_min.x(), real_min.y(), 1);
		draw_3D_max  =octomath::Vector3 (real_max.x(), real_max.y(), 1);
		draw_3D_min = octomath::Vector3(real_min.x(), real_min.y(), 0);
		ROS_WARN_STREAM("Scenario bounderies "<< real_min << " - " << real_max);

		GridBenchmark generic(scenario, fileName, octree);


	  	RegularGrid grid_regular_2D("test", Algorithm::regularGrid_2d, octree, grid_resolution); 
		ResultSet result_regular_2D = generic.findFrontierCells(real_max, real_min, twoD, grid_regular_2D);	
		storage.storeAlgorithmResults(result_regular_2D);
		generic.drawAllLevels("OctomapFrontier_RegularGrid_2D_"+fileName, draw_2D_max, draw_2D_min);
	  	
	  	RegularGrid grid_regular_3D("test", Algorithm::regularGrid_3d, octree, grid_resolution);  
	  	ResultSet result_regular_3D = generic.findFrontierCells(real_max, real_min, threeD, grid_regular_3D);
		storage.storeAlgorithmResults(result_regular_3D);
		generic.drawAllLevels("OctomapFrontier_RegularGrid_3D_"+fileName, draw_3D_max, draw_3D_min);



	  	SparseGrid grid2D("test", Algorithm::sparseGrid_2d, octree, grid_resolution); 
		ResultSet result_sparse_2D = generic.findFrontierCells(real_max, real_min, twoD, grid2D);
		storage.storeAlgorithmResults(result_sparse_2D);
		generic.drawAllLevels("OctomapFrontier_SparseGrid_2D_"+fileName, draw_2D_max, draw_2D_min);

	  	SparseGrid grid3D("test", Algorithm::sparseGrid_3d, octree, grid_resolution);  
	  	ResultSet result_sparse_3D = generic.findFrontierCells(real_max, real_min, threeD, grid3D);
		storage.storeAlgorithmResults(result_sparse_3D);
		generic.drawAllLevels("OctomapFrontier_SparseGrid_3D_"+fileName, draw_3D_max, draw_3D_min);
	}

	/*TEST_F(DataCollection, RunAllScenarios)
	{
		std::list<Scenario> scenarioOrder = {Scenario::circle, Scenario::double_circle, Scenario::s, Scenario::oil, Scenario::oil_larger};
		bool printTables = true;

		runAllCombinations("circle_1m", circle);
		runAllCombinations("double_circle_1m", double_circle);
		runAllCombinations("s_1m", s);
		runAllCombinations("offShoreOil_1m", oil);
		runAllCombinations("offShoreOil_4obstacles_1m", oil_larger);



		storage.exportCsv(scenarioOrder, printTables, "simulationScenarios");
	}*/

	TEST_F(DataCollection, RunExperimentalDataset)
	{
		std::list<Scenario> scenarioOrder = {Scenario::experimental};
		bool printTables = false;
		runAllCombinations("experimentalDataset", experimental);
		storage.exportCsv(scenarioOrder, printTables, "experimental");
	}


}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}