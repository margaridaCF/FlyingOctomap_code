#include <observation_maneuver.h>


#include <sstream>
#include <fstream>

#define SAVE_LOG 1


namespace observation_lib
{


    std::ofstream log_file;

	OPPairs::OPPairs(int circle_divisions, double distance_toTarget, double distance_inFront, double distance_behind)
		: circle_divisions(circle_divisions)
	{
		current = OPPair();
		starts_zero     = Eigen::MatrixXd (3, circle_divisions);
		ends_zero       = Eigen::MatrixXd (3, circle_divisions);
		directions_zero = Eigen::MatrixXd (3, circle_divisions);
		observation_lib::precalculation (distance_toTarget, circle_divisions, distance_inFront, distance_behind, starts_zero, ends_zero, directions_zero);
	}

	void OPPairs::NewFrontier(Eigen::Vector3d new_frontier, Eigen::Vector3d uav_position, rviz_interface::PublishingInput pi)
	{
		frontier = new_frontier;
		index = 0;
		motion_direction = frontier - uav_position;
		// Next();
	}

	bool OPPairs::Next()
	{	
		#ifdef SAVE_LOG
		log_file.open ("/home/mfaria/Flying_Octomap_code/src/data/current/oppair.log", std::ofstream::app);
		#endif
		log_file << "[OPPairs] Next. circle_divisions " << circle_divisions << std::endl;
		if (index < circle_divisions )
		{
			observation_lib::translate(motion_direction, starts_zero.col(index), ends_zero.col(index), directions_zero.col(index), frontier, current.start, current.end);
			#ifdef SAVE_LOG
			log_file << "[OPPairs] [" << index << "] (" << current.start(0) << ", " << current.start(1) << ", " << current.start(2) << ") --> (" << current.end(0) << ", " << current.end(1) << ", " << current.end(2) << ")" << std::endl;
            log_file.close();
			#endif
			index++;
			return true;
		}
		else
		{
			#ifdef SAVE_LOG
            log_file.close();
			#endif
			return false;
		}
	}

	Eigen::Vector3d OPPairs::get_current_start()
	{
		return current.start;
	}

	Eigen::Vector3d OPPairs::get_current_end()
	{
		return current.end;
	}

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

	void precalculation (double radius, int point_number, double distance_inFront, double distance_behind, Eigen::MatrixXd & starts_zero, Eigen::MatrixXd & ends_zero, Eigen::MatrixXd & directions_zero)
	{
		Eigen::MatrixXd circle_unitary (3, point_number);
		generateCirclePoints(point_number, circle_unitary);
		Eigen::MatrixXd circle_radius(3, point_number);
		circle_radius = circle_unitary * radius;
		Eigen::Vector3d k(0, 0, 1);
		for (int i = 0; i < point_number; ++i)
		{
			Eigen::Vector3d point = circle_unitary.col(i);
			directions_zero.col(i) = k.cross(point);
		}

		starts_zero = circle_radius - distance_behind *directions_zero;
		ends_zero   = circle_radius + distance_inFront*directions_zero;
	}

	void translate( Eigen::Vector3d const& motion_direction, Eigen::Vector3d const& start_zero, Eigen::Vector3d const& end_zero, Eigen::Vector3d const& direction_zero, Eigen::Vector3d const& frontier, Eigen::Vector3d & start, Eigen::Vector3d & end)
	{
		double check = direction_zero.dot(motion_direction);
		if(check >= 0)
		{
			start = frontier + start_zero;
			end   = frontier + end_zero;
		}
		else
		{
			end   = frontier + start_zero;
			start = frontier + end_zero;
		}
	}

	
}
