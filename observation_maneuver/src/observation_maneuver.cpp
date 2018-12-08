#include <observation_maneuver.h>


namespace observation_lib
{
	// == Algorithm constantes ==
	Eigen::MatrixXd starts_zero;
	Eigen::MatrixXd ends_zero;
	Eigen::MatrixXd directions_zero;


	OPPairs::OPPairs(int circle_divisions, double distance_toTarget, double distance_inFront, double distance_behind)
		: circle_divisions(circle_divisions)
	{
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
	}

	bool OPPairs::NextOPPair(OPPair & nextOPPair, rviz_interface::PublishingInput pi)
	{	
		if (index < circle_divisions )
		{
			observation_lib::translate(motion_direction, starts_zero.col(index), ends_zero.col(index), directions_zero.col(index), frontier, nextOPPair.start, nextOPPair.end);
			// ROS_ERROR_STREAM("[" << index << "] From (" << nextOPPair.start(0)<<", "<<nextOPPair.start(1)<<", "<<nextOPPair.start(2) << ") to (" << nextOPPair.end(0)<<", "<<nextOPPair.end(1)<<", "<<nextOPPair.end(2) << ")" );
			index++;
			if(pi.publish)
			{
				octomath::Vector3 start_octoVec (nextOPPair.start(0),  nextOPPair.start(1),  nextOPPair.start(2));
				octomath::Vector3 end_octoVec   (nextOPPair.end(0),    nextOPPair.end(1),    nextOPPair.end(2));
				visualization_msgs::Marker  marker;
				marker = visualization_msgs::Marker();
				int marker_id = index;
				int red_base = 1;
	    		rviz_interface::build_arrow_path(start_octoVec, end_octoVec, 100+marker_id, marker, 1+0.5, "oppair_direction");
				pi.waypoint_array.markers.push_back( marker );
				pi.marker_pub.publish(pi.waypoint_array);
			}
			return true;
		}
		else
		{
			return false;
		}
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
