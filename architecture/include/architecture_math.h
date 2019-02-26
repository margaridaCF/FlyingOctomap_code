#ifndef ARCHITECTURE_MATH_H
#define ARCHITECTURE_MATH_H
#include <Eigen/Dense>
#include <octomap/math/Vector3.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>

# define M_PI       3.14159265358979323846  /* pi */

namespace architecture_math
{


    struct Vector3Hash
    {
        std::size_t operator()(const octomath::Vector3 & v) const 
        {
            int scale = 0.00001;
            std::size_t hx = std::hash<float>{}( (int)(v.x() / scale) * scale );
            std::size_t hy = std::hash<float>{}( (int)(v.y() / scale) * scale );
            std::size_t hz = std::hash<float>{}( (int)(v.z() / scale) * scale );
            std::size_t return_value = ((hx 
               ^ (hy << 1)) >> 1)
               ^ (hz << 1);
            return return_value;
        }
    };

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
		// ROS_INFO_STREAM("calculateOrientation( (" << start.x() << ", " << start.y() << "), (" << end.x() << ", " << end.y() << ") = " << result << " = " << result*180/M_PI);				
		return result;
	}


}
#endif // ARCHITECTURE_MATH_h