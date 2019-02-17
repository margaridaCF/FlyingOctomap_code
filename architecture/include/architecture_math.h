#ifndef ARCHITECTURE_MATH_H
#define ARCHITECTURE_MATH_H
#include <Eigen/Dense>


namespace architecture_math
{


	double calculateOrientation(Eigen::Vector3d start, Eigen::Vector3d end)
    {
        Eigen::Vector3d d = end - start;
        d.normalize();
        return std::acos (1/d.norm());
    }

}
#endif // ARCHITECTURE_MATH_h