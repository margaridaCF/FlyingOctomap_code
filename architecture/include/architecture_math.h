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
		d.normalize();
		double adjacent = d.x();
		double hipotenuse = d.norm();
		ROS_INFO_STREAM("calculateOrientation (" << start.x() << ", " << start.y() << "), (" << end.x() << ", " << end.y() << ")" );				
		ROS_INFO_STREAM("calculateOrientation [1] direction (" << end.x() << ", " << end.y() << ")" << " - (" << start.x() << ", " << start.y() << ") = (" << d.x() << ", " << d.y() << ")");
		// ROS_INFO_STREAM("calculateOrientation [3] direction.x    = adjacent   = " << adjacent);
		// ROS_INFO_STREAM("calculateOrientation [2] direction norm = hipotenuse = " << hipotenuse);
		if(hipotenuse == 0)
		{
			result = 0;
		} 	
		else
		{
			result = adjacent/hipotenuse ;
			result = std::acos (result);
			ROS_INFO_STREAM("calculateOrientation [4] acos(adjacent/hipotenuse) = acos(" << adjacent << " / " << hipotenuse << ") = acos(" << (adjacent/hipotenuse) << ") = " <<  result << " <=> " <<  result*180/M_PI );
			if(d.y() < 0)
			{
				// if(d.x() < 0) result = result + M_PI/2;
				// else		  result = result - M_PI/2;
				// if(d.x() < 0)
				// {
					result = 2*M_PI-result;	
					ROS_INFO_STREAM("calculateOrientation [5] q3/q4 because "<< d.y() << " < 0 " << " . Result = " << result << " = " << result*180/M_PI );
				// } 
				// else
				// {
				// 	result = 2*M_PI-result ;
				// 	ROS_INFO_STREAM("calculateOrientation [5] q4 because "<< d.y() << " < 0 && " << d.x() << " >= 0 . Result = " << result << " = " << result*180/M_PI  );
				// }
			}
		} 
		return result ;
		// return result + M_PI;
	}


}
#endif // ARCHITECTURE_MATH_h