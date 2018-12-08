#ifndef OBSERVATION_MANEUVER_H
#define OBSERVATION_MANEUVER_H
#include <ros/ros.h>
#include <Eigen/Dense>
#include <marker_publishing_utils.h>

namespace observation_lib
{
	class OPPair
	{
	public:
		Eigen::Vector3d start;
		Eigen::Vector3d end;
	};

	class OPPairs
	{
		Eigen::Vector3d motion_direction;
		int circle_divisions;
		int index;
		Eigen::Vector3d frontier;
	public:
		OPPairs(int circle_divisions, double distance_toTarget, double distance_inFront, double distance_behind);
		void NewFrontier(Eigen::Vector3d new_frontier, Eigen::Vector3d uav_position, rviz_interface::PublishingInput pi);
		bool NextOPPair(OPPair & nextOPPair, rviz_interface::PublishingInput pi);


	};
	typedef Eigen::Vector3d (*translationCalculation)(Eigen::Vector3d const& trig_point_test, Eigen::Vector3d const& directionTest, double distance) ;
	const double pi = std::acos(-1);
	void generateCirclePoints(int point_number, Eigen::MatrixXd & circle_points);
	Eigen::Vector3d calculateDirectionTest(Eigen::Vector3d const& point_circle, Eigen::Vector3d const& frontier, Eigen::Vector3d const& uav_pos);
	Eigen::Vector3d calculateTrigStart(Eigen::Vector3d const& trig_point_test, Eigen::Vector3d const& directionTest, double distance);
	Eigen::Vector3d calculateTrigEnd(Eigen::Vector3d const& trig_point_test, Eigen::Vector3d const& directionTest, double distance);
	Eigen::Vector3d calculatePointTranslation(Eigen::Vector3d const& trig_circle_point, Eigen::Vector3d const& frontier, Eigen::Vector3d const& uav_pos, double distance, translationCalculation translationOp);


	void precalculation (double radius, int point_number, double distance_inFront, double distance_behind, Eigen::MatrixXd & starts_zero, Eigen::MatrixXd & ends_zero, Eigen::MatrixXd & directions_zero);
	void translate( Eigen::Vector3d const& motion_direction, Eigen::Vector3d const& start_zero, Eigen::Vector3d const& end_zero, Eigen::Vector3d const& direction_zero, Eigen::Vector3d const& frontier, Eigen::Vector3d & start, Eigen::Vector3d & end);
}


#endif // OBSERVATION_MANEUVER_H