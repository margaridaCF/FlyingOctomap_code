

#include <frontier_cells_sandbox.h>
#include <gtest/gtest.h>

namespace mapper{

	class FrontierCellsSandboxTest : public ::testing::Test {
	protected:
	  	FrontierCellsSandboxTest() 
	  	{
	  	}

		virtual ~FrontierCellsSandboxTest() {
		// You can do clean-up work that doesn't throw exceptions here.
		}
	};

	void gatherOctomapData(std::string filename)
	{
		OctomapFrontierCellsSandbox explorationSandbox(Scenario::circle, "circle", "data/"+filename+".bt");
		std::tuple<double, double, double> min, max;
		explorationSandbox.getBounds(min, max);
		octomath::Vector3 real_max(std::get<0>(max), std::get<1>(max), 1.1);
		octomath::Vector3 real_min(std::get<0>(min), std::get<1>(min), 1);
		ROS_WARN_STREAM(" -- "+filename+" --");
		octomath::Vector3 dimensions = real_max - real_min;
		//ROS_WARN_STREAM("Area of "<<(dimensions.x()*dimensions.y()*dimensions.z())<<" - "<<dimensions);
		ROS_WARN_STREAM("octree has the following bounderies "<< real_max << "  -  " << real_min);

		std::chrono::high_resolution_clock::time_point t1, t2;
		std::chrono::duration<double> time_span;

		// Regular grid
		// explorationSandbox.findFrontierCells(real_max, real_min, twoD);
		// explorationSandbox.drawAllLevels("OctomapFrontier_RegularGrid_2D_"+filename, real_max, real_min);

		// real_max = octomath::Vector3 (std::get<0>(max)-55, std::get<1>(max)-50, 1.5);
		// real_min = octomath::Vector3 (std::get<0>(min)+60, std::get<1>(min)+40, 0.5);
		// explorationSandbox.findFrontierCells(real_max, real_min, threeD);
		// explorationSandbox.drawAllLevels("OctomapFrontier_RegularGrid_3D_"+filename, real_max, real_min);
		


		// Sparse grid
		real_max = octomath::Vector3 (std::get<0>(max), std::get<1>(max), 1);
		real_min = octomath::Vector3 (std::get<0>(min), std::get<1>(min), 1);
		t1 = std::chrono::high_resolution_clock::now();
		explorationSandbox.findFrontierCells_sparseIteration(real_max, real_min, twoD);
		t2 = std::chrono::high_resolution_clock::now();
		time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
		ROS_WARN_STREAM("Frontiers found in "<<time_span.count()<<" seconds.");
		//explorationSandbox.drawAllLevels("OctomapFrontier_SparseGrid_2D_"+filename, real_max, real_min);


		real_max = octomath::Vector3 (std::get<0>(max), std::get<1>(max), 1.5);
		real_min = octomath::Vector3 (std::get<0>(min), std::get<1>(min), 0.5);
		t1 = std::chrono::high_resolution_clock::now();
		explorationSandbox.findFrontierCells_sparseIteration(real_max, real_min, threeD);
		t2 = std::chrono::high_resolution_clock::now();
		time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
		ROS_WARN_STREAM("Frontiers found in "<<time_span.count()<<" seconds.");
		// //explorationSandbox.drawAllLevels("OctomapFrontier_SparseGrid_3D_"+filename, real_max, real_min);




		//explorationSandbox.exportCsv();
	}

	TEST_F(FrontierCellsSandboxTest, Voxel_is_InZlevel)
	{
		Voxel target (0, 0, 0, 0.2f);
		EXPECT_TRUE(target.isInZlevel(0));
	}
	TEST_F(FrontierCellsSandboxTest, Voxel_is_InZlevel_size)
	{
		Voxel target (0, 0, 0.5f, 2);
		EXPECT_TRUE(target.isInZlevel(0));
	}

	TEST_F(FrontierCellsSandboxTest, Voxel_not_InZlevel)
	{
		Voxel target (0, 0, 0, 0.2f);
		EXPECT_FALSE(target.isInZlevel(1));
	}
	TEST_F(FrontierCellsSandboxTest, Voxel_not_InZlevel_size)
	{
		Voxel target (0, 0, 1, 1);
		EXPECT_FALSE(target.isInZlevel(0));
	}

	// This is more for data collection then an actual test
	// TEST_F(FrontierCellsSandboxTest, OctomapFrontierTest_CompareSparseIteration_circle)
	// {
	// 	gatherOctomapData("circle_1m");
	// }
	// TEST_F(FrontierCellsSandboxTest, OctomapFrontierTest_CompareSparseIteration_doubleCircle)
	// {

	// 	gatherOctomapData("double_circle_1m");
	// }

	// TEST_F(FrontierCellsSandboxTest, OctomapFrontierTest_CompareSparseIteration_s)
	// {
	// 	gatherOctomapData("s_1m");
	// }
	// TEST_F(FrontierCellsSandboxTest, OctomapFrontierTest_CompareSparseIteration_offShoreOil)
	// {
	// 	gatherOctomapData("offShoreOil_1m");
	// }
	// TEST_F(FrontierCellsSandboxTest, OctomapFrontierTest_CompareSparseIteration_offShoreOil4obstacles)
	// {
	// 	gatherOctomapData("offShoreOil_4obstacles_1m");
	// }
	TEST_F(FrontierCellsSandboxTest, OctomapFrontierTest_CompareSparseIteration_offShoreOil4obstacles)
	{
		gatherOctomapData("experimentalDataset");
	}


}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
