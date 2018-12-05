#include <observation_maneuver.h>


namespace observation_lib
{

	void generateCirclePoints(int point_number, Eigen::MatrixXd & point_matrix)
	{
		double interval_angle_rad = (2*pi)/point_number;
		// ROS_INFO_STREAM("Angle iteration " << interval_angle_rad << " rad");
		point_matrix = Eigen::MatrixXd (3, point_number);
		for (int index = 0; index < point_number; ++index)
		{
			point_matrix(0, index) = std::cos(interval_angle_rad*index);
			point_matrix(1, index) = std::sin(interval_angle_rad*index);
			point_matrix(2, index) = 0;
		}
	}

	Eigen::Vector3d calculateDirectionTest(Eigen::Vector3d const& point_circle, Eigen::Vector3d const& frontier, Eigen::Vector3d const& uav_pos)
	{
		Eigen::Vector3d frontierToUav = frontier - uav_pos;
		Eigen::Vector3d k (0, 0, 1);
		Eigen::Vector3d directionTest = k.cross(point_circle);
		double check = directionTest.dot(frontierToUav);
		if(check < 0)
		{
			directionTest = -directionTest;
		}
		return directionTest;
	}

	Eigen::Vector3d calculateTrigStart(Eigen::Vector3d const& trig_point_test, Eigen::Vector3d const& directionTest, double distance)
	{
		return trig_point_test - distance * directionTest;
	}

	Eigen::Vector3d calculateTrigEnd(Eigen::Vector3d const& trig_point_test, Eigen::Vector3d const& directionTest, double distance)
	{
		return trig_point_test + distance * directionTest;
	}

	Eigen::Vector3d calculatePointTranslation(Eigen::Vector3d const& trig_circle_point, Eigen::Vector3d const& frontier, Eigen::Vector3d const& uav_pos, double distance, translationCalculation translationOp)
	{
		Eigen::Vector3d frontier_circle_point = frontier + trig_circle_point;
		Eigen::Vector3d directionFlyBy = calculateDirectionTest(trig_circle_point, frontier, uav_pos);
		return translationOp(frontier_circle_point, directionFlyBy, distance);
	}

	void precalculation (double radius, int point_number, double distance_inFront, double distance_behind, Eigen::MatrixXd & starts_zero, Eigen::MatrixXd & ends_zero, Eigen::MatrixXd & directions_zero)
	{
		Eigen::MatrixXd circle_unitary (3, point_number);
		generateCirclePoints(point_number, circle_unitary);

		ROS_INFO_STREAM(circle_unitary);

		Eigen::MatrixXd circle_radius(3, point_number);
		circle_radius = radius * circle_unitary;

		Eigen::Vector3d k(0, 0, 1);
		for (int i = 0; i < point_number; ++i)
		{
			Eigen::Vector3d point = circle_unitary.col(i);
			directions_zero.col(i) = k.cross(point);
		}

		starts_zero = circle_radius - distance_behind *directions_zero;
		ends_zero   = circle_radius + distance_inFront*directions_zero;


	}
}
