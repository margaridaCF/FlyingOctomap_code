#ifndef OBSERVATION_MANEUVER_H
#define OBSERVATION_MANEUVER_H
#include <ros/ros.h>
#include <Eigen/Dense>
#include <marker_publishing_utils.h>


namespace observation_lib
{
	typedef void (*translate_func_ptr)( Eigen::Vector3d const& motion_direction, Eigen::Vector3d const& start_zero, Eigen::Vector3d const& end_zero, Eigen::Vector3d const& direction_zero, Eigen::Vector3d const& frontier, Eigen::Vector3d & start, Eigen::Vector3d & end);


	class OPPair
	{
	public:
		Eigen::Vector3d start;
		Eigen::Vector3d end;
	};

	class OPPairs
	{
		// == Algorithm constantes ==
		Eigen::MatrixXd starts_zero;
		Eigen::MatrixXd ends_zero;
		Eigen::MatrixXd directions_zero;
		Eigen::Vector3d motion_direction;
		int circle_divisions;
		int index;
		Eigen::Vector3d frontier;
		OPPair current;
		translate_func_ptr translate_func;
	public:
		OPPairs(int circle_divisions, double distance_toTarget, double distance_inFront, double distance_behind, translate_func_ptr translate_func);
		OPPairs(){}
		void NewFrontier(Eigen::Vector3d new_frontier, Eigen::Vector3d uav_position, rviz_interface::PublishingInput pi);
		bool Next();
		Eigen::Vector3d get_current_start();
		Eigen::Vector3d get_current_end();
		Eigen::Vector3d get_frontier();



	};
	const double pi = std::acos(-1);
	void generateCirclePoints(int point_number, Eigen::MatrixXd & circle_points);
	void precalculation (double radius, int point_number, double distance_inFront, double distance_behind, Eigen::MatrixXd & starts_zero, Eigen::MatrixXd & ends_zero, Eigen::MatrixXd & directions_zero);
	void translate( Eigen::Vector3d const& motion_direction, Eigen::Vector3d const& start_zero, Eigen::Vector3d const& end_zero, Eigen::Vector3d const& direction_zero, Eigen::Vector3d const& frontier, Eigen::Vector3d & start, Eigen::Vector3d & end);
	void translateAdjustDirection( Eigen::Vector3d const& motion_direction, Eigen::Vector3d const& start_zero, Eigen::Vector3d const& end_zero, Eigen::Vector3d const& direction_zero, Eigen::Vector3d const& frontier, Eigen::Vector3d & start, Eigen::Vector3d & end);
}


#endif // OBSERVATION_MANEUVER_H