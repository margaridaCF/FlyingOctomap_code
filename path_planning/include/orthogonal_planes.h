#ifndef OPEN_H
#define OPEN_H

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
		const octomath::Vector3 zAxis (0, 0, 1);
		const octomath::Vector3 zero  (0, 0, 0);
		octomath::Vector3 direction = goal - start;
		direction.normalize();
		octomath::Vector3 oA = direction.cross(zAxis);
		oA.normalize();

		// ROS_WARN_STREAM("direction: " << direction);
		// ROS_WARN_STREAM("orthogonalA: " << oA);
		// ROS_WARN_STREAM(direction << ".dot(" << oA << ") =  " << direction.dot(oA));

		if ( equal(oA, zero, 0.1) )
		{
			// Direction is close to parallel to zAxis
			octomath::Vector3 yAxis (0, 1, 0);
			oA = direction.cross(yAxis);
			oA.normalize();
			// ROS_WARN_STREAM("orthogonalA: " << oA);

		}

		octomath::Vector3 oB = direction.cross(oA);
		oB.normalize();
		// ROS_WARN_STREAM("orthogonalB: " << oB);


		CoordinateFrame coord (direction, oA, oB);
		return coord;
	}
}

#endif // OPEN_H