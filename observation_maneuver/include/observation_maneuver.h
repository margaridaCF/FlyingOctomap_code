#ifndef OBSERVATION_MANEUVER_H
#define OBSERVATION_MANEUVER_H
#include <ros/ros.h>
#include <Eigen/Dense>

namespace ObservationManeuver
{
	const double pi = std::acos(-1);
	void generateCirclePoints(int point_number, Eigen::MatrixXd & circle_points);
}


#endif // OBSERVATION_MANEUVER_H