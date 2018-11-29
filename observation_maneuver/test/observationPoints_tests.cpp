	#include <gtest/gtest.h>
#include <observation_maneuver.h>


namespace ObservationManeuver
{
	double tolerance = 0.0000000001;
	TEST(ObservationManeuverTest, generateCircle_12)
	{
		int point_number = 12;
		Eigen::MatrixXd circle_points (3, point_number);
		generateCirclePoints(point_number, circle_points);

		ASSERT_EQ(circle_points.cols(), point_number);

		ASSERT_NEAR	(circle_points(0, 1), std::sqrt(3)/2, 	tolerance);
		ASSERT_NEAR	(circle_points(1, 1), 1.0/2.0, 			tolerance);
		ASSERT_EQ	(circle_points(2, 1), 0);

		ASSERT_NEAR	(circle_points(0, 2), 1.0/2.0, 			tolerance);
		ASSERT_NEAR	(circle_points(1, 2), std::sqrt(3)/2, 	tolerance);
		ASSERT_EQ	(circle_points(2, 2), 0);

		ASSERT_NEAR	(circle_points(0, 4), -1.0/2.0, 		tolerance);
		ASSERT_NEAR	(circle_points(1, 4), std::sqrt(3)/2, 	tolerance);
		ASSERT_EQ	(circle_points(2, 4), 0);

		ASSERT_NEAR	(circle_points(0, 5), -std::sqrt(3)/2, 	tolerance);
		ASSERT_NEAR	(circle_points(1, 5), 1.0/2.0, 			tolerance);
		ASSERT_EQ	(circle_points(2, 5), 0);

		ASSERT_NEAR	(circle_points(0, 6), -1, 				tolerance);
		ASSERT_NEAR	(circle_points(1, 6), 0, 				tolerance);
		ASSERT_EQ	(circle_points(2, 6), 0);

		ASSERT_NEAR	(circle_points(0, 7), -std::sqrt(3)/2, 	tolerance);
		ASSERT_NEAR	(circle_points(1, 7), -1.0/2.0, 		tolerance);
		ASSERT_EQ	(circle_points(2, 7), 0);

		ASSERT_NEAR	(circle_points(0, 8), -1.0/2.0, 		tolerance);
		ASSERT_NEAR	(circle_points(1, 8), -std::sqrt(3)/2, 	tolerance);
		ASSERT_EQ	(circle_points(2, 8), 0);

		ASSERT_NEAR	(circle_points(0, 9), 0, 				tolerance);
		ASSERT_NEAR	(circle_points(1, 9), -1, 				tolerance);
		ASSERT_EQ	(circle_points(2, 9), 0);

		ASSERT_NEAR	(circle_points(0, 10), 1.0/2.0,			tolerance);
		ASSERT_NEAR	(circle_points(1, 10), -std::sqrt(3)/2,	tolerance);
		ASSERT_EQ	(circle_points(2, 10), 0);

		ASSERT_NEAR	(circle_points(0, 11), std::sqrt(3)/2, 	tolerance);
		ASSERT_NEAR	(circle_points(1, 11), -1.0/2.0,		tolerance);
		ASSERT_EQ	(circle_points(2, 11), 0);
	}

	TEST(ObservationManeuverTest, generateCircle_4)
	{
		// double tolerance = 0.0000000001;
		int point_number = 4;
		Eigen::MatrixXd circle_points (3, point_number);
		generateCirclePoints(point_number, circle_points);

		ASSERT_EQ(circle_points.cols(), point_number);
		ASSERT_EQ	(circle_points(0, 0), 1);
		ASSERT_NEAR	(circle_points(1, 0), 0, tolerance);
		ASSERT_EQ	(circle_points(2, 0), 0);

		ASSERT_NEAR	(circle_points(0, 1), 0, tolerance);
		ASSERT_NEAR	(circle_points(1, 1), 1, tolerance);
		ASSERT_EQ	(circle_points(2, 1), 0);
		
		ASSERT_NEAR	(circle_points(0, 2), -1, tolerance);
		ASSERT_NEAR	(circle_points(1, 2), 0, tolerance);
		ASSERT_EQ	(circle_points(2, 2), 0);
		
		ASSERT_NEAR	(circle_points(0, 3), 0, tolerance);
		ASSERT_NEAR	(circle_points(1, 3), -1, tolerance);
		ASSERT_EQ	(circle_points(2, 3), 0);
	}
	
	TEST(ObservationManeuverTest, direction_q1)
	{

		Eigen::Vector3d point_circle (0.5, 0.87, 0);
		Eigen::Vector3d frontier(0, 12, 0);
		Eigen::Vector3d uav_pos(0, 0, 0);
		Eigen::Vector3d directionTest = calculateDirectionTest(point_circle, frontier, uav_pos);
		ASSERT_NEAR(directionTest(0), -0.87, tolerance);
		ASSERT_NEAR(directionTest(1), 0.5, tolerance);
		ASSERT_NEAR(directionTest(2), 0, tolerance);
	}


	TEST(ObservationManeuverTest, direction_q2)
	{

		Eigen::Vector3d point_circle (-0.5, 0.87, 0);
		Eigen::Vector3d frontier(0, 12, 0);
		Eigen::Vector3d uav_pos(0, 0, 0);
		Eigen::Vector3d directionTest = calculateDirectionTest(point_circle, frontier, uav_pos);
		ASSERT_NEAR(directionTest(0), 0.87, tolerance);
		ASSERT_NEAR(directionTest(1), 0.5, tolerance);
		ASSERT_NEAR(directionTest(2), 0, tolerance);
	}


	TEST(ObservationManeuverTest, direction_q3)
	{

		Eigen::Vector3d point_circle (-0.5, -0.87, 0);
		Eigen::Vector3d frontier(0, 12, 0);
		Eigen::Vector3d uav_pos(0, 0, 0);
		Eigen::Vector3d directionTest = calculateDirectionTest(point_circle, frontier, uav_pos);
		ASSERT_NEAR(directionTest(0), -0.87, tolerance);
		ASSERT_NEAR(directionTest(1), 0.5, tolerance);
		ASSERT_NEAR(directionTest(2), 0, tolerance);
	}


	TEST(ObservationManeuverTest, direction_q4)
	{

		Eigen::Vector3d point_circle (0.5, -0.87, 0);
		Eigen::Vector3d frontier(0, 12, 0);
		Eigen::Vector3d uav_pos(0, 0, 0);
		Eigen::Vector3d directionTest = calculateDirectionTest(point_circle, frontier, uav_pos);
		ASSERT_NEAR(directionTest(0), 0.87, tolerance);
		ASSERT_NEAR(directionTest(1), 0.5, tolerance);
		ASSERT_NEAR(directionTest(2), 0, tolerance);
	}

	TEST(ObservationManeuverTest, observationStart_frontier)
	{
		Eigen::Vector3d trig_circle_point (1, 0, 0);
		Eigen::Vector3d frontier(0, 12, 0);
		Eigen::Vector3d uav_pos(0, 0, 0);
		double distance = 1;
		Eigen::Vector3d observationStart = calculatePointTranslation(trig_circle_point, frontier, uav_pos, distance, calculateTrigStart);
		ASSERT_NEAR(1, observationStart(0), tolerance);
		ASSERT_NEAR(11, observationStart(1), tolerance);
		ASSERT_NEAR(0, observationStart(2), tolerance);
	}

	TEST(ObservationManeuverTest, observationEnd_frontier)
	{
		Eigen::Vector3d trig_circle_point (1, 0, 0);
		Eigen::Vector3d frontier(0, 12, 0);
		Eigen::Vector3d uav_pos(0, 0, 0);
		double distance = 1;
		Eigen::Vector3d observationStart = calculatePointTranslation(trig_circle_point, frontier, uav_pos, distance, calculateTrigEnd);
		ASSERT_NEAR(1, observationStart(0), tolerance);
		ASSERT_NEAR(13, observationStart(1), tolerance);
		ASSERT_NEAR(0, observationStart(2), tolerance);
	}
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}