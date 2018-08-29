#ifndef ORTHOGONAL_PLANES_H
#define ORTHOGONAL_PLANES_H

#include <octomap/math/Vector3.h>
#include <ltStarOctree_common.h>
#include <ros/ros.h>



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

	octomath::Vector3 rotateAndTranslate(CoordinateFrame coordinate_frame, octomath::Vector3 offsetPoint, octomath::Vector3 start, octomath::Vector3 goalWithMargin)
	{
		// Matrix r = (direction.x, orthogonalA.x, orthogonalB.x )
		//            (direction.y, orthogonalA.y, orthogonalB.y )
		//            (direction.z, orthogonalA.z, orthogonalB.z )

		// rotated_point = r * offsetPoint;
		// rotated_start = rotated_point + start;
		// rotated_goalWithMargin  = rotated_point + goalWithMargin ;
	}


	

	void generateRectanglePlaneIndexes(double margin, double resolution, std::vector<octomath::Vector3> & plane)
	{
		int loop_count = (margin/resolution) * 2;
		double x = -margin + resolution/2;
		double z = x;
		int array_index = 0;
		for (int i = 0; i < loop_count; ++i)
		{
			for (int j = 0; j < loop_count; ++j)
			{
				plane.emplace(plane.end(), 0, x + i*resolution, z + j*resolution);
				array_index++;
			}
		}
	}
}

#endif // ORTHOGONAL_PLANES_H