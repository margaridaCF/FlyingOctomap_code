#ifndef FRONTIERS_COMMON_H
#define FRONTIERS_COMMON_H

#include <vector>
#include <cmath>
#include <sstream>
#include <iostream>
#include <fstream>
#include <chrono>
#include <algorithm>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <marker_publishing_utils.h>
#include <octomap/OcTree.h>
#include <octomap/math/Vector3.h>
#include <frontiers_msgs/FindFrontiers.h>

// #define BASELINE 1
// #define RUNNING_ROS 1


namespace Frontiers{


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

    struct VectorComparatorEqual // for unordered_map
    { 
        bool operator () (const octomath::Vector3 & lhs, const octomath::Vector3 & rhs) const 
        { 
            double scale = 0.0001;
            // ROS_WARN_STREAM(   std::setprecision(8) << "Distance from " << lhs << " and  " << rhs << " is " << lhs.distance(rhs) << " <= " << scale << " returning " << (lhs.distance(rhs) <= scale)   );
            return lhs.distance(rhs) <= scale;
            // returns !0 if the two container object keys passed as arguments are to be considered equal.
        } 
    };


    class Voxel
        {
        public:
            double x, y, z, size;
            Voxel()
                : x(0), y(0), z(0), size(0)
                {}
            Voxel(double x, double y, double z, double size)
                : x(x), y(y), z(z), size(size)
                {}
            Voxel(frontiers_msgs::VoxelMsg const& msg)
                : x(msg.xyz_m.x), y(msg.xyz_m.y), z(msg.xyz_m.z), size(msg.size)
                {}
            bool isInZlevel(float z_level) const
            {
                float max_z = z + (size/2); 
                float min_z = z -(size/2);
                if(max_z >= z_level && min_z <= z_level)
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }
            ///  Operators
            bool operator==(Voxel const& otherVoxel) const
            {
                return (x == otherVoxel.x && y == otherVoxel.y 
                    && z == otherVoxel.z && size == otherVoxel.size);
            }

            ///  Display  and  <<
            std::string displayString() const
            {
              return "(" + std::to_string(x) + "; "+ std::to_string(y) + " )";
            }
            std::ostream& displayString(std::ostream& stream_out) const
            {
              stream_out << "(" << x << "; " << y << "; "<< z << " ) x "<<size ;
              stream_out.precision(3);
              return stream_out;
            }
            frontiers_msgs::VoxelMsg toMsg(frontiers_msgs::VoxelMsg& msg)
            {
                msg.xyz_m.x = x;
                msg.xyz_m.y = y;
                msg.xyz_m.z = z;
                msg.size = size;
            }

        };
}
#endif //FRONTIERS_COMMON_H
