#include <gtest/gtest.h>
#include <observation_maneuver.h>


namespace observation_lib
{
	double tolerance = 0.0000000001;

	TEST(ObservationManeuverTest, directions)
	{
		double radius = 1;
		int point_number = 4;
		double distance_inFront = 1;
		double distance_behind = 1;
		Eigen::MatrixXd starts_zero (3, point_number);
		Eigen::MatrixXd ends_zero(3, point_number);
		Eigen::MatrixXd directions_zero(3, point_number);
		// Eigen::Vector3d point_circle (0.5, 0.87, 0);
		Eigen::Vector3d frontier(0, 12, 0);
		Eigen::Vector3d uav_pos(0, 0, 0);
		// Eigen::Vector3d directionTest = calculateDirectionTest(point_circle, frontier, uav_pos);
		precalculation (radius, point_number, distance_inFront, distance_behind, starts_zero, ends_zero, directions_zero);
		EXPECT_NEAR(directions_zero(0, 0), 0, tolerance);
		EXPECT_NEAR(directions_zero(1, 0), 1, tolerance);
		EXPECT_NEAR(directions_zero(2, 0), 0, tolerance);

		EXPECT_NEAR(directions_zero(0, 1), -1, tolerance);
		EXPECT_NEAR(directions_zero(1, 1), 0, tolerance);
		EXPECT_NEAR(directions_zero(2, 1), 0, tolerance);


		EXPECT_NEAR(directions_zero(0, 2), 0, tolerance);
		EXPECT_NEAR(directions_zero(1, 2), -1, tolerance);
		EXPECT_NEAR(directions_zero(2, 2), 0, tolerance);

		EXPECT_NEAR(directions_zero(0, 3), 1, tolerance);
		EXPECT_NEAR(directions_zero(1, 3), 0, tolerance);
		EXPECT_NEAR(directions_zero(2, 3), 0, tolerance);
	}
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}