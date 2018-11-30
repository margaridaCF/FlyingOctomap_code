#include <gtest/gtest.h>
#include <observation_maneuver.h>


namespace ObservationManeuver
{
	double tolerance = 0.0000000001;

	TEST(ObservationManeuverTest, observationStart_auto)
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

	TEST(ObservationManeuverTest, observationEnd_auto)
	{
		Eigen::Vector3d trig_circle_point (1, 0, 0);
		Eigen::Vector3d frontier(0, 12, 0);
		Eigen::Vector3d uav_pos(0, 0, 0);
		double distance = 1;
		Eigen::Vector3d observationEnd = calculatePointTranslation(trig_circle_point, frontier, uav_pos, distance, calculateTrigEnd);
		ASSERT_NEAR(1, observationEnd(0), tolerance);
		ASSERT_NEAR(13, observationEnd(1), tolerance);
		ASSERT_NEAR(0, observationEnd(2), tolerance);
	}
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}