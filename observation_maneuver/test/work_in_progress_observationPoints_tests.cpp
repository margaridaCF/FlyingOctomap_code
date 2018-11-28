#include <gtest/gtest.h>
#include <observation_maneuver.h>


namespace ObservationManeuver
{
	double tolerance = 0.0000000001;

	TEST(ObservationManeuverTest, direction_q1)
	{

		Eigen::Vector3d point_circle (0.5, 0.87, 0);
		Eigen::Vector3d frontier(0, 12, 0);
		Eigen::Vector3d uav_pos(0, 0, 0);
		Eigen::Vector3d directionTest = calculateDirectionTest(point_circle, frontier, uav_pos);
		ASSERT_NEAR(directionTest(0), -0.87, tolerance);
		ASSERT_NEAR(directionTest(1), 0.5, tolerance);
		ASSERT_NEAR(directionTest(2), 0, tolerance);

		// ROS_INFO_STREAM(directionTest);
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

		// ROS_INFO_STREAM(directionTest);
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

		// ROS_INFO_STREAM(directionTest);
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

		// ROS_INFO_STREAM(directionTest);
	}

}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}