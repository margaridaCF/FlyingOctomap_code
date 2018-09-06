#include <orthogonal_planes.h>
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

	// TEST(OrthogonalPlanesTest, generategoalWithDistance_)
	// {
	// 	octomath::Vector3 start (1, 1, 1);
	// 	octomath::Vector3 goal  (-1, -1, -1);
	// 	double margin = 2 * (start.norm() );
	// 	ASSERT_TRUE (   equal( calculateGoalWithMargin(start, goal, margin), octomath::Vector3(-2, -2, -2) )   );
	// }

	// TEST(OrthogonalPlanesTest, generatePlaneIndexes_16)
	// {
	// 	double margin = 1;
	// 	double resolution = 0.5;
	// 	std::vector<octomath::Vector3> plane = {};
	// 	generateRectanglePlaneIndexes(margin, resolution, plane);
	// 	ASSERT_EQ(16, plane.size());
	// }

	// TEST(OrthogonalPlanesTest, generatePlaneIndexes_25)
	// {
	// 	double margin = 1.25;
	// 	double resolution = 0.5;
	// 	std::vector<octomath::Vector3> plane = {};
	// 	generateRectanglePlaneIndexes(margin, resolution, plane);
	// 	ASSERT_EQ(25, plane.size());
	// }

	// TEST(OrthogonalPlanesTest, generateCircle)
	// {
	// 	double margin = 1;
	// 	double resolution = 0.5;
	// 	std::vector<octomath::Vector3> plane = {};
	// 	generateCirclePlaneIndexes(margin, resolution, plane);
	// 	ASSERT_EQ(21, plane.size());
	// 	for (std::vector<octomath::Vector3>::iterator i = plane.begin(); i != plane.end(); ++i)
	// 	{
	// 		ROS_WARN_STREAM(*i);	
	// 	}
	// }

	// TEST(OrthogonalPlanesTest, generateCircle)
	// {
	// 	double margin = 1;
	// 	double resolution = 0.5;
	// 	std::vector<octomath::Vector3> plane = {};
	// 	generateCirclePlaneIndexes(margin, resolution, plane);
	// 	ASSERT_EQ(21, plane.size());
	// 	for (std::vector<octomath::Vector3>::iterator i = plane.begin(); i != plane.end(); ++i)
	// 	{
	// 		ROS_WARN_STREAM(*i);	
	// 	}
	// }

	// TEST(OrthogonalPlanesTest, generateSemiSphere)
	// {
	// 	double margin = 1;
	// 	double resolution = 0.5;
	// 	std::vector<octomath::Vector3> plane = {};
	// 	std::vector<octomath::Vector3> semiSphere = {};
	// 	generateCirclePlaneIndexes(margin, resolution, plane);
	// 	generateSemiSphereOut(margin, resolution, plane, semiSphere);
	// 	ASSERT_EQ(21, plane.size());
	// 	ASSERT_EQ(21, semiSphere.size());
	// 	for (std::vector<octomath::Vector3>::iterator i = semiSphere.begin(); i != semiSphere.end(); ++i)
	// 	{
	// 		ROS_WARN_STREAM(*i);	
	// 	}
	// }

	
	// TEST(OrthogonalPlanesTest, generateCoordinateFrame)
	// {
	// 	octomath::Vector3 start (0, 0,0 );
	// 	octomath::Vector3 goal (-0.333, 2, 40);
	//     CoordinateFrame coord = generateCoordinateFrame(start, goal);
	//     // ROS_WARN_STREAM("direction " << coord.direction);
	//     // ROS_WARN_STREAM("orthogonalA " << coord.orthogonalA);
	//     // ROS_WARN_STREAM("orthogonalB " << coord.orthogonalB);
	//     testCoordinateFrame(start, goal);
	// }

	// TEST(OrthogonalPlanesTest, rotation)
	// {
	// 	// octomath::Vector3 start (-0.333, 2, 40 );
	// 	octomath::Vector3 start (0, 0, 0 );
	// 	octomath::Vector3 goal (1, 1, 1);
	// 	CoordinateFrame coordinate_frame = generateCoordinateFrame(start, goal);

	// 	double margin = 1;
	// 	double resolution = 0.5;
		
	// 	std::vector<Eigen::Vector3d> plane = {};
	// 	generateCirclePlaneIndexes(margin, resolution, plane);
		
	// 	Eigen::MatrixXd point_matrix (3, plane.size());
	// 	int index = 0;
	// 	for (std::vector<Eigen::Vector3d>::iterator i = plane.begin(); i != plane.end(); ++i)
	// 	{
	// 		point_matrix(0, index) = i->x();
	// 		point_matrix(1, index) = i->y();
	// 		point_matrix(2, index) = i->z();
	// 		index++;
	// 	}

	// 	// ROS_WARN_STREAM(point_matrix);

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


	//     Eigen::MatrixXd rotated_points = rotate_many(coordinate_frame, point_matrix);
	//     Eigen::Vector3d start_eigen ((double)start.x(), (double) start.y(), (double) start.z());
	//     for (int i = 0; i < plane.size(); ++i)
	//     {
	//     	Eigen::Vector3d point(rotated_points(0, i), rotated_points(1, i), rotated_points(2, i) ) ;
	//     	point = translateStartGoal(point, start_eigen);
	//     	ROS_WARN_STREAM("correct.emplace(correct.end(), " << point[0] << ",  " << point[1] << ",  " << point[2] << ");");

	//     	// ASSERT_TRUE ( correct[i].isApprox(point, 0.00001) ) << i;
	//     }

	// }


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
		Eigen::MatrixXd circle_plane_matrix = generateCirclePlaneMatrix(margin, resolution);
		
		// For each start and goal
		CoordinateFrame coordinate_frame = generateCoordinateFrame(start, goal);
		Eigen::MatrixXd transformation_matrix = generateRotationTranslationMatrix(coordinate_frame, start);
		Eigen::MatrixXd points_around_start = transformation_matrix * circle_plane_matrix;

	    for (int i = 0; i < circle_plane_matrix.cols(); ++i)
	    {
	    	Eigen::Vector3d point (points_around_start(0, i), points_around_start(1, i), points_around_start(2, i));
	    	EXPECT_TRUE ( correct[i].isApprox(point, 0.00001) ) << "Correct: " << correct[i] << " - But computed instead: " << point ;
	    }
	}


	// TEST(OrthogonalPlanesTest, rotationTranslation_v2)
	// {
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
	// 	// octomath::Vector3 start (-0.333, 2, 40 );
	// 	octomath::Vector3 start (0, 0, 0 );
	// 	octomath::Vector3 goal (1, 1, 1);
	// 	double margin = 1;
	// 	double resolution = 0.5;
		

	// 	// On initialization		
	// 	Eigen::MatrixXd circle_plane_matrix = generateCirclePlaneMatrix(margin, resolution);
	// 	ROS_WARN_STREAM(circle_plane_matrix(0) );
		
	// 	// For each start and goal
	// 	CoordinateFrame coordinate_frame = generateCoordinateFrame(start, goal);
	// 	Eigen::MatrixXd transformation_matrix = generateRotationTranslationMatrix(coordinate_frame, start);
	// 	Eigen::MatrixXd points_around_start = transformation_matrix * circle_plane_matrix;
	// 	ROS_WARN_STREAM(transformation_matrix);

	//     for (int i = 0; i < circle_plane_matrix.cols(); ++i)
	//     {
	//     	Eigen::Vector3d point (points_around_start(0, i), points_around_start(1, i), points_around_start(2, i));
	//     	EXPECT_TRUE ( correct[i].isApprox(point, 0.00001) ) << "Correct: " << correct[i] << " - But computed instead: " << point ;
	//     }

	//     // ROS_WARN_STREAM(" Correct " << correct);
	//     ROS_WARN_STREAM("Result " << points_around_start);
	// }


	// TEST(OrthogonalPlanesTest, rotationTranslationMatrix)
	// {
	// 	// Initial conditions
	// 	octomath::Vector3 start (-0.333, 2, 40 );
	// 	octomath::Vector3 goal (1, 1, 1);
	// 	double margin = 1;
	// 	double resolution = 0.5;
	// 	CoordinateFrame coordinate_frame = generateCoordinateFrame(start, goal);

	// 	Eigen::MatrixXd point_matrix (3, 3);
	// 	Eigen::MatrixXd rotated_points = rotate_many(coordinate_frame, point_matrix);
	// 	Eigen::MatrixXd transformation_matrix = generateRotationTranslationMatrix(coordinate_frame, start);
	// 	ROS_WARN_STREAM(transformation_matrix);

	//     // for (int i = 0; i < circle_plane_matrix.cols(); ++i)
	//     // {
	//     // 	Eigen::Vector3d point (points_around_start(0, i), points_around_start(1, i), points_around_start(2, i));
	//     // 	EXPECT_TRUE ( correct[i].isApprox(point, 0.00001) ) << "Correct: " << correct[i] << " - But computed instead: " << point ;
	//     // }

	//     // // ROS_WARN_STREAM(" Correct " << correct);
	//     // ROS_WARN_STREAM("Result " << points_around_start);
	// }
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}