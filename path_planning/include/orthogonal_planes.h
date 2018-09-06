#ifndef ORTHOGONAL_PLANES_H
#define ORTHOGONAL_PLANES_H

#include <octomap/math/Vector3.h>
#include <ltStarOctree_common.h>
#include <ros/ros.h>

#include <Eigen/Dense>


namespace LazyThetaStarOctree{

	class CoordinateFrame
	{
	public:
		const octomath::Vector3 direction;
		const octomath::Vector3 orthogonalA;
		const octomath::Vector3 orthogonalB;
		CoordinateFrame (octomath::Vector3 direction, octomath::Vector3 orthogonalA, octomath::Vector3 orthogonalB)
			: direction(direction), orthogonalA(orthogonalA), orthogonalB(orthogonalB)
			{}
	};

	CoordinateFrame generateCoordinateFrame(octomath::Vector3 const& start, octomath::Vector3 const& goal)
	{
		const octomath::Vector3 xAxis (1, 0, 0);
		const octomath::Vector3 zAxis (0, 0, 1);
		const octomath::Vector3 zero  (0, 0, 0);
		octomath::Vector3 oA, oB;


		if(equal(start, goal))
		{
			CoordinateFrame coord (octomath::Vector3(1, 0, 0), octomath::Vector3(0, 1, 0), octomath::Vector3(0, 0, 1));
			return coord;
		}


		octomath::Vector3 direction = goal - start;
		direction.normalize();

		if(  std::abs( direction.dot(zAxis) ) <= 0.9 )
		{
			oA = zAxis.cross(direction);
		}
		else
		{
			oA = direction.cross(xAxis);
		}

		oA.normalize();
		oB = direction.cross(oA);

		CoordinateFrame coord (direction, oA, oB);
		return coord;
	}

	octomath::Vector3 calculateGoalWithMargin(octomath::Vector3 const& start, octomath::Vector3 const& goal, double margin)
	{
		octomath::Vector3 direction = goal - start;
		double distance = direction.norm();
		direction.normalize();
		octomath::Vector3 scale = direction * (margin/2);
		return goal + scale;
	}

	void generateRectanglePlaneIndexes(double margin, double resolution, std::vector<octomath::Vector3> & plane)
	{
	}
	Eigen::MatrixXd generateRotationTranslationMatrix(CoordinateFrame coordinate_frame, octomath::Vector3 translationOffset)
	{
		Eigen::MatrixXd m(4, 4);
		m(0, 0) = coordinate_frame.direction.x();
		m(1, 0) = coordinate_frame.direction.y();
		m(2, 0) = coordinate_frame.direction.z();
		m(3, 0) = 0;

		m(0, 1) = coordinate_frame.orthogonalA.x();
		m(1, 1) = coordinate_frame.orthogonalA.y();
		m(2, 1) = coordinate_frame.orthogonalA.z();
		m(3, 1) = 0;

		m(0, 2) = coordinate_frame.orthogonalB.x();
		m(1, 2) = coordinate_frame.orthogonalB.y();
		m(2, 2) = coordinate_frame.orthogonalB.z();
		m(3, 2) = 0;

		m(0, 3) = translationOffset.x();
		m(1, 3) = translationOffset.y();
		m(2, 3) = translationOffset.z();
		m(3, 3) = 1;

		return m;
	}


	Eigen::MatrixXd generateCirclePlaneMatrix(double margin, double resolution)
	{
		std::vector<Eigen::Vector3d> plane = {};
		int n = margin/ resolution;
		// ROS_WARN_STREAM("N " << n);
		int loop_count =   n * 2  + 1;
		int array_index = 0;
		double  x, y, y_, z_;
		x = y = y_ = z_ = 0;

		double rectSquare = ( margin / resolution ) * ( margin / resolution );

		// ROS_WARN_STREAM("rectSquare " << rectSquare);

		for (int i = 0; i < loop_count; ++i)
		{
			x = std::abs (i - n);
			for (int j = 0; j < loop_count; ++j)
			{
				y = std::abs(j - n);

				if( x*x + y*y  <= rectSquare )
				{
					y_ = (i - n) * resolution;
					z_ = (j - n) * resolution;
					plane.emplace(plane.end(), 0, y_, z_);
					array_index++;
				}
			}
		}


		Eigen::MatrixXd point_matrix (4, plane.size());
		int index = 0;
		for (std::vector<Eigen::Vector3d>::iterator i = plane.begin(); i != plane.end(); ++i)
		{
			point_matrix(0, index) = i->x();
			point_matrix(1, index) = i->y();
			point_matrix(2, index) = i->z();
			point_matrix(3, index) = 1;
			index++;
		}

		return point_matrix;
	}


	Eigen::MatrixXd generateOffsetMatrix(double margin, double resolution, double (*calculateDepth)(double, double, double)  )
	{
		std::vector<Eigen::Vector3d> plane = {};
		int n = margin/ resolution;
		// ROS_WARN_STREAM("N " << n);
		int loop_count =   n * 2  + 1;
		int array_index = 0;
		double  x, y, y_, z_, depth;
		x = y = y_ = z_ = 0;

		double rectSquare = ( margin / resolution ) * ( margin / resolution );

		// ROS_WARN_STREAM("rectSquare " << rectSquare);

		for (int i = 0; i < loop_count; ++i)
		{
			x = std::abs (i - n);
			for (int j = 0; j < loop_count; ++j)
			{
				y = std::abs(j - n);

				if( x*x + y*y  <= rectSquare )
				{
					y_ = (i - n) * resolution;
					z_ = (j - n) * resolution;
					depth = calculateDepth( margin, y_, z_);
					plane.emplace(plane.end(), depth, y_, z_);
					array_index++;
				}
			}
		}


		Eigen::MatrixXd point_matrix (4, plane.size());
		int index = 0;
		for (std::vector<Eigen::Vector3d>::iterator i = plane.begin(); i != plane.end(); ++i)
		{
			point_matrix(0, index) = i->x();
			point_matrix(1, index) = i->y();
			point_matrix(2, index) = i->z();
			point_matrix(3, index) = 1;
			index++;
		}

		return point_matrix;
	}

	double dephtZero(double margin, double y, double z)
	{
		return 0;
	}

	double semiSphereOut(double margin, double y, double z)
	{
		return 0.05 + std::sqrt( margin * margin  -  y*y  -   z*z);
	}

	double semiSphereIn(double margin, double y, double z)
	{
		return -semiSphereOut(margin, y, z);
	}



}

#endif // ORTHOGONAL_PLANES_H