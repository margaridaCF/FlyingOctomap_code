#ifndef ARCHITECTURE_MATH_H
#define ARCHITECTURE_MATH_H
#include <Eigen/Dense>
#include <ros/ros.h>

# define M_PI       3.14159265358979323846  /* pi */

namespace architecture_math
{


	double calculateOrientation(Eigen::Vector2d start, Eigen::Vector2d end)
	{
	   	double result;
		Eigen::Vector2d d = end - start;
		double hipotenuse = d.norm();
		double adjacent = end.x() - start.x();
		if(d.norm() == 0)
		{
			result = 0;
		} 	
		else
		{
			result = adjacent/hipotenuse ;
			result = std::acos (result);
			d.normalize();
			if(d.y() < 0)
			{
				result = -result; 
				result += M_PI;
			}
		} 
		ROS_INFO_STREAM("calculateOrientation( (" << start.x() << ", " << start.y() << "), (" << end.x() << ", " << end.y() << ") = " << result << " = " << result*180/M_PI);				
		return result;
	}


}
#endif // ARCHITECTURE_MATH_h