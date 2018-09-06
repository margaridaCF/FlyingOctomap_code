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

		// ROS_WARN_STREAM("direction: " << direction);
		// ROS_WARN_STREAM("orthogonalA: " << oA);
		// ROS_WARN_STREAM(direction << ".dot(" << oA << ") =  " << direction.dot(oA));

		oB = direction.cross(oA);
		// ROS_WARN_STREAM("orthogonalB: " << oB);


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

	Eigen::Vector3d rotate(CoordinateFrame coordinate_frame, Eigen::Vector3d offsetPoint)
	{
		Eigen::MatrixXd m(3, 3);
		m(0, 0) = coordinate_frame.direction.x();
		m(1, 0) = coordinate_frame.direction.y();
		m(2, 0) = coordinate_frame.direction.z();

		m(0, 1) = coordinate_frame.orthogonalA.x();
		m(1, 1) = coordinate_frame.orthogonalA.y();
		m(2, 1) = coordinate_frame.orthogonalA.z();

		m(0, 2) = coordinate_frame.orthogonalB.x();
		m(1, 2) = coordinate_frame.orthogonalB.y();
		m(2, 2) = coordinate_frame.orthogonalB.z();
		// Matrix r = (direction.x, orthogonalA.x, orthogonalB.x )
		//            (direction.y, orthogonalA.y, orthogonalB.y )
		//            (direction.z, orthogonalA.z, orthogonalB.z )

		// ROS_WARN_STREAM(m);
		return m * offsetPoint;
	}

	Eigen::MatrixXd rotate_many(CoordinateFrame coordinate_frame, Eigen::MatrixXd offsetPoints)
	{
		Eigen::MatrixXd m(3, 3);
		m(0, 0) = coordinate_frame.direction.x();
		m(1, 0) = coordinate_frame.direction.y();
		m(2, 0) = coordinate_frame.direction.z();

		m(0, 1) = coordinate_frame.orthogonalA.x();
		m(1, 1) = coordinate_frame.orthogonalA.y();
		m(2, 1) = coordinate_frame.orthogonalA.z();

		m(0, 2) = coordinate_frame.orthogonalB.x();
		m(1, 2) = coordinate_frame.orthogonalB.y();
		m(2, 2) = coordinate_frame.orthogonalB.z();
		// Matrix r = (direction.x, orthogonalA.x, orthogonalB.x )
		//            (direction.y, orthogonalA.y, orthogonalB.y )
		//            (direction.z, orthogonalA.z, orthogonalB.z )



		Eigen::MatrixXd result =  m * offsetPoints;
		return result;
	}

	Eigen::MatrixXd translateStartGoal(Eigen::Vector3d rotated_points, Eigen::Vector3d offset)
	{

		return  rotated_points + offset;
	}


	void generateRectanglePlaneIndexes(double margin, double resolution, std::vector<octomath::Vector3> & plane)
	{
	}

	void generateCirclePlaneIndexes(double margin, double resolution, std::vector<Eigen::Vector3d> & plane)
	{
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
			x = std::abs (i - n) - 0.5;
			for (int j = 0; j < loop_count; ++j)
			{
				y = std::abs(j - n) - 0.5;

				if( x*x + y*y  <= rectSquare )
				{
					y_ = (i - n) * resolution;
					z_ = (j - n) * resolution;
					plane.emplace(plane.end(), 0, y_, z_);
					array_index++;
				}
			}
		}
	}

	void generateSemiSphereOut(double margin, double resolution, std::vector<octomath::Vector3> & plane, std::vector<octomath::Vector3> & semiSphere)
	{
		double depth;
		double safety_range = margin + 0.5 * resolution;
		for (std::vector<octomath::Vector3>::iterator i = plane.begin(); i != plane.end(); ++i)
		{
			depth = std::sqrt( safety_range * safety_range  -  i->y()*i->y()  -   i->z()*i->z());
			semiSphere.emplace(semiSphere.end(), depth, i->y(), i->z());
		}
	} 

	void generateSemiSphereIn(double margin, double resolution, std::vector<octomath::Vector3> & plane, std::vector<octomath::Vector3> & semiSphere)
	{
		double depth;
		double safety_range = margin + 0.5 * resolution;
		for (std::vector<octomath::Vector3>::iterator i = plane.begin(); i != plane.end(); ++i)
		{
			depth = std::sqrt( safety_range * safety_range  -  i->y()*i->y()  -   i->z()*i->z());
			semiSphere.emplace(semiSphere.end(), -depth, i->y(), i->z());
		}
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
			x = std::abs (i - n) - 0.5;
			for (int j = 0; j < loop_count; ++j)
			{
				y = std::abs(j - n) - 0.5;

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
			point_matrix(3, index) = 0;
			index++;
		}

		return point_matrix;
	}

}

#endif // ORTHOGONAL_PLANES_H