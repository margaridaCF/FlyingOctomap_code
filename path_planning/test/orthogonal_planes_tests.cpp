#include <orthogonal_planes.h>
#include <ltStar_lib_ortho.h>
#include <gtest/gtest.h>

namespace LazyThetaStarOctree{
	void testCoordinateFrame(octomath::Vector3 start, octomath::Vector3 goal)
	{
		CoordinateFrame coordinate_frame = generateCoordinateFrame(start, goal);

		// All vectors are orthogonal
		ASSERT_NEAR(coordinate_frame.orthogonalA.dot(coordinate_frame.orthogonalB), 0, 0.0000001)  << "All unitary vector must be orthogonal. (dot product equals 0)";
		ASSERT_NEAR(coordinate_frame.orthogonalA.dot(coordinate_frame.direction), 0, 0.0000001) << "All unitary vector must be orthogonal. (dot product equals 0)";
		ASSERT_NEAR(coordinate_frame.orthogonalB.dot(coordinate_frame.direction), 0, 0.0000001) << "All unitary vector must be orthogonal. (dot product equals 0)";
		
		// All vectors are unitary
		ASSERT_NEAR(coordinate_frame.direction.norm(),   1, 0.0001) << "Unitary vector should have size 1";
		ASSERT_NEAR(coordinate_frame.orthogonalA.norm(), 1, 0.0001) << "Unitary vector should have size 1";
		ASSERT_NEAR(coordinate_frame.orthogonalB.norm(), 1, 0.0001) << "Unitary vector should have size 1";
	}


	TEST(OrthogonalPlanesTest, generateCoordinateFrame_originToYaxis)
	{
		octomath::Vector3 start (0, 0, 0);
		octomath::Vector3 goal  (0, 1, 0);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_originToNegatativeYaxis)
	{
		octomath::Vector3 start (0, 0, 0);
		octomath::Vector3 goal  (0, -1, 0);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_originToXaxis)
	{
		octomath::Vector3 start (0, 0, 0);
		octomath::Vector3 goal  (1, 0, 0);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_originToNegatativeXaxis)
	{
		octomath::Vector3 start (0, 0, 0);
		octomath::Vector3 goal  (-1, 0, 0);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_originToZaxis)
	{
		octomath::Vector3 start (0, 0, 0);
		octomath::Vector3 goal  (0, 0, 1);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_closeToOriginToZaxis)
	{
		octomath::Vector3 start (0, 0, 0);
		octomath::Vector3 goal  (0.1, 0, 1);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_originToNegatativeZaxis)
	{
		octomath::Vector3 start (0, 0, 0);
		octomath::Vector3 goal  (0, 0, -1);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_originToXaxisZero)
	{
		octomath::Vector3 start (0, 0, 0);
		octomath::Vector3 goal  (0, 1, 1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(0, -1, 1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(0, -1, -1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(0, 1, -1);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_originToYaxisZero)
	{
		octomath::Vector3 start (0, 0, 0);
		octomath::Vector3 goal  (1, 0, 1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(-1, 0, 1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(-1, 0, -1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(1, 0, -1);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_originToAllDiagonals)
	{
		octomath::Vector3 start (0, 0, 0);
		octomath::Vector3 goal  (1, 1, 1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(-1, 1, 1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(-1, 1, -1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(1, 1, -1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(1, -1, 1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(-1, -1, 1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(-1, -1, -1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(1, -1, -1);
		testCoordinateFrame(start, goal);
	}

	TEST(OrthogonalPlanesTest, generateCoordinateFrame_offset_originToYaxis)
	{
		octomath::Vector3 start (2, 5, -634);
		octomath::Vector3 goal  (0, 1, 0);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_offset_originToNegatativeYaxis)
	{
		octomath::Vector3 start (2, 5, -634);
		octomath::Vector3 goal  (0, -1, 0);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_offset_originToXaxis)
	{
		octomath::Vector3 start (2, 5, -634);
		octomath::Vector3 goal  (1, 0, 0);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_offset_originToNegatativeXaxis)
	{
		octomath::Vector3 start (2, 5, -634);
		octomath::Vector3 goal  (-1, 0, 0);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_offset_originToZaxis)
	{
		octomath::Vector3 start (2, 5, -634);
		octomath::Vector3 goal  (0, 0, 1);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_offset_closeToOriginToZaxis)
	{
		octomath::Vector3 start (2, 5, -634);
		octomath::Vector3 goal  (0.1, 0, 1);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_offset_originToNegatativeZaxis)
	{
		octomath::Vector3 start (2, 5, -634);
		octomath::Vector3 goal  (0, 0, -1);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_offset_originToXaxisZero)
	{
		octomath::Vector3 start (2, 5, -634);
		octomath::Vector3 goal  (0, 1, 1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(0, -1, 1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(0, -1, -1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(0, 1, -1);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_offset_originToYaxisZero)
	{
		octomath::Vector3 start (2, 5, -634);
		octomath::Vector3 goal  (1, 0, 1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(-1, 0, 1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(-1, 0, -1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(1, 0, -1);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_offset_originToAllDiagonals)
	{
		octomath::Vector3 start (2, 5, -634);
		octomath::Vector3 goal  (1, 1, 1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(-1, 1, 1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(-1, 1, -1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(1, 1, -1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(1, -1, 1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(-1, -1, 1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(-1, -1, -1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(1, -1, -1);
		testCoordinateFrame(start, goal);
	}



	TEST(OrthogonalPlanesTest, generateCoordinateFrame_monkeyTest)
	{
		octomath::Vector3 start (2, 5, -634);
		octomath::Vector3 goal  (20, -31, 40);
		testCoordinateFrame(start, goal);
	}

	TEST(OrthogonalPlanesTest, generateCoordinateFrame_monkeyTest1)
	{
		octomath::Vector3 start (2, 5, -634);
		octomath::Vector3 goal  (20, -31, 40);
		testCoordinateFrame(start, goal);
	}

	TEST(OrthogonalPlanesTest, generateCoordinateFrame_monkeyTest2)
	{
		octomath::Vector3 start (-2, 785, -634);
		octomath::Vector3 goal  (20, -31, 40);
		testCoordinateFrame(start, goal);
	}

	TEST(OrthogonalPlanesTest, generateCoordinateFrame_monkeyTest3)
	{
		octomath::Vector3 start (-2, 5, -634);
		octomath::Vector3 goal  (20, -31, -40);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_monkeyTest4)
	{
		octomath::Vector3 start (2, 25, -634);
		octomath::Vector3 goal  (20, -31, 40);
		testCoordinateFrame(start, goal);
	}

	TEST(OrthogonalPlanesTest, generateCoordinateFrame_monkeyTest5)
	{
		octomath::Vector3 start (2, -5, -634);
		octomath::Vector3 goal  (20, -31, 40);
		testCoordinateFrame(start, goal);
	}

	TEST(OrthogonalPlanesTest, generateCoordinateFrame_monkeyTest6)
	{
		octomath::Vector3 start (-2, 785, -634);
		octomath::Vector3 goal  (-20, -31, 40);
		testCoordinateFrame(start, goal);
	}

	TEST(OrthogonalPlanesTest, generateCoordinateFrame_monkeyTest7)
	{
		octomath::Vector3 start (-2, -5, -634);
		octomath::Vector3 goal  (-20, -31, -40);
		testCoordinateFrame(start, goal);
	}

	TEST(OrthogonalPlanesTest, generategoalWithDistance_)
	{
		octomath::Vector3 start (1, 1, 1);
		octomath::Vector3 goal  (-1, -1, -1);
		double margin = 2 * (start.norm() );
		ASSERT_TRUE (   equal( calculateGoalWithMargin(start, goal, margin), octomath::Vector3(-2, -2, -2) )   );
	}

	
	TEST(OrthogonalPlanesTest, generateCoordinateFrame)
	{
		octomath::Vector3 start (0, 0,0 );
		octomath::Vector3 goal (-0.333, 2, 40);
	    CoordinateFrame coord = generateCoordinateFrame(start, goal);
	    // ROS_WARN_STREAM("direction " << coord.direction);
	    // ROS_WARN_STREAM("orthogonalA " << coord.orthogonalA);
	    // ROS_WARN_STREAM("orthogonalB " << coord.orthogonalB);
	    testCoordinateFrame(start, goal);
	}

	TEST(OrthogonalPlanesTest, rotationTranslation_v2_rotation)
	{
		std::vector<Eigen::Vector3d> correct;
		correct.emplace(correct.end(), 0.911231,  -0.502983,  -0.408248);
		correct.emplace(correct.end(), 0.707107,  -0.707107,  0);
		correct.emplace(correct.end(), 0.502983,  -0.911231,  0.408248);
		correct.emplace(correct.end(), 0.761802,  0.0546949,  -0.816497);
		correct.emplace(correct.end(), 0.557678,  -0.149429,  -0.408248);
		correct.emplace(correct.end(), 0.353553,  -0.353553,  0);
		correct.emplace(correct.end(), 0.149429,  -0.557678,  0.408248);
		correct.emplace(correct.end(), -0.0546949,  -0.761802,  0.816497);
		correct.emplace(correct.end(), 0.408248,  0.408248,  -0.816497);
		correct.emplace(correct.end(), 0.204124,  0.204124,  -0.408248);
		correct.emplace(correct.end(), 0,  0,  0);
		correct.emplace(correct.end(), -0.204124,  -0.204124,  0.408248);
		correct.emplace(correct.end(), -0.408248,  -0.408248,  0.816497);
		correct.emplace(correct.end(), 0.0546949,  0.761802,  -0.816497);
		correct.emplace(correct.end(), -0.149429,  0.557678,  -0.408248);
		correct.emplace(correct.end(), -0.353553,  0.353553,  0);
		correct.emplace(correct.end(), -0.557678,  0.149429,  0.408248);
		correct.emplace(correct.end(), -0.761802,  -0.0546949,  0.816497);
		correct.emplace(correct.end(), -0.502983,  0.911231,  -0.408248);
		correct.emplace(correct.end(), -0.707107,  0.707107,  0);
		correct.emplace(correct.end(), -0.911231,  0.502983,  0.408248);


		// Initial conditions
		// octomath::Vector3 start (-0.333, 2, 40 );
		octomath::Vector3 start (0, 0, 0 );
		octomath::Vector3 goal (1, 1, 1);
		double margin = 1;
		double resolution = 0.5;

		// On initialization		

	// Eigen::MatrixXd  startOffsets;
	// Eigen::MatrixXd  goalOffsets;
		generateOffsets(margin, resolution, dephtZero, dephtZero );
		
		// For each start and goal
		CoordinateFrame coordinate_frame = generateCoordinateFrame(start, goal);
		Eigen::MatrixXd transformation_matrix = generateRotationTranslationMatrix(coordinate_frame, start);
		Eigen::MatrixXd points_around_start = transformation_matrix * startOffsets;

	 //    for (int i = 0; i < startOffsets.cols(); ++i)
	 //    {
	 //    	Eigen::Vector3d point (points_around_start(0, i), points_around_start(1, i), points_around_start(2, i));
	 //    	EXPECT_TRUE ( correct[i].isApprox(point, 0.00001) ) << "Correct: " << correct[i] << " - But computed instead: " << point ;
	 //    }
	}


	// TEST(OrthogonalPlanesTest, rotationTranslation_v2)
	// {
	// 	//ARRANGE
	// 	std::vector<Eigen::Vector3d> correct;
	// 	correct.emplace(correct.end(), 0.166708,  3.00011,  39.9914);
	// 	correct.emplace(correct.end(), -0.333, 2.99967, 39.9744);
	// 	correct.emplace(correct.end(), -0.832708, 2.99923, 39.9573);
	// 	correct.emplace(correct.end(), 0.666417, 2.50071, 40.0213);
	// 	correct.emplace(correct.end(), 0.166708, 2.50027, 40.0043);
	// 	correct.emplace(correct.end(), -0.333, 2.49984, 39.9872);
	// 	correct.emplace(correct.end(), -0.832708, 2.4994, 39.9701);
	// 	correct.emplace(correct.end(), -1.33242, 2.49896, 39.953);
	// 	correct.emplace(correct.end(), 0.666417, 2.00088, 40.0341);
	// 	correct.emplace(correct.end(), 0.166708, 2.00044, 40.0171);
	// 	correct.emplace(correct.end(), -0.333, 2, 40);
	// 	correct.emplace(correct.end(), -0.832708, 1.99956, 39.9829);
	// 	correct.emplace(correct.end(), -1.33242, 1.99912, 39.9659);
	// 	correct.emplace(correct.end(), 0.666417, 1.50104, 40.047);
	// 	correct.emplace(correct.end(), 0.166708, 1.5006, 40.0299);
	// 	correct.emplace(correct.end(), -0.333, 1.50016, 40.0128);
	// 	correct.emplace(correct.end(), -0.832708, 1.49973, 39.9957);
	// 	correct.emplace(correct.end(), -1.33242, 1.49929, 39.9787);
	// 	correct.emplace(correct.end(), 0.166708, 1.00077, 40.0427);
	// 	correct.emplace(correct.end(), -0.333, 1.00033, 40.0256);
	// 	correct.emplace(correct.end(), -0.832708, 0.999891, 40.0086);

	// 	// Initial conditions
	// 	octomath::Vector3 start (-0.333, 2, 40 );
	// 	octomath::Vector3 goal (1, 1, 1);
	// 	double margin = 1;
	// 	double resolution = 0.5;
		
	// 	// ACT
	// 	// On initialization	
	// Eigen::MatrixXd  startOffsets;
	// Eigen::MatrixXd  goalOffsets;	
	// 	generateOffsets(margin, resolution, dephtZero, dephtZero);
	// 	// For each start and goal
	// 	CoordinateFrame coordinate_frame = generateCoordinateFrame(start, goal);
	// 	Eigen::MatrixXd transformation_matrix = generateRotationTranslationMatrix(coordinate_frame, start);
	// 	Eigen::MatrixXd points_around_start = transformation_matrix * startOffsets;

	// 	// ASSERT
	//     for (int i = 0; i < startOffsets.cols(); ++i)
	//     {
	//     	Eigen::Vector3d point (points_around_start(0, i), points_around_start(1, i), points_around_start(2, i));
	//     	EXPECT_TRUE ( correct[i].isApprox(point, 0.00001) ) << i << " - Correct: " << correct[i] << " - But computed instead: " << point ;
	//     }
	// }
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}