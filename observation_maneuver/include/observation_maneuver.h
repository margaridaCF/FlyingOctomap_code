#ifndef OBSERVATION_MANEUVER_H
#define OBSERVATION_MANEUVER_H
#include <ros/ros.h>
#include <Eigen/Dense>

namespace observation_lib
{
	typedef Eigen::Vector3d (*translationCalculation)(Eigen::Vector3d const& trig_point_test, Eigen::Vector3d const& directionTest, double distance) ;
	const double pi = std::acos(-1);
	void generateCirclePoints(int point_number, Eigen::MatrixXd & circle_points);
	Eigen::Vector3d calculateDirectionTest(Eigen::Vector3d const& point_circle, Eigen::Vector3d const& frontier, Eigen::Vector3d const& uav_pos);
	Eigen::Vector3d calculateTrigStart(Eigen::Vector3d const& trig_point_test, Eigen::Vector3d const& directionTest, double distance);
	Eigen::Vector3d calculateTrigEnd(Eigen::Vector3d const& trig_point_test, Eigen::Vector3d const& directionTest, double distance);
	Eigen::Vector3d calculatePointTranslation(Eigen::Vector3d const& trig_circle_point, Eigen::Vector3d const& frontier, Eigen::Vector3d const& uav_pos, double distance, translationCalculation translationOp);
}


#endif // OBSERVATION_MANEUVER_H