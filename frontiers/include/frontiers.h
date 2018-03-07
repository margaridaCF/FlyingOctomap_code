#ifndef FRONTIERS_H
#define FRONTIERS_H

#include <vector>
#include <cmath>
#include <sstream>
#include <iostream>
#include <fstream>

//#include <math.h>
#include <ros/ros.h>
#include <octomap/OcTree.h>
#include <octomap/math/Vector3.h>
#include <chrono>
#include <algorithm>

#include <geometry_msgs/Point.h>

#include <frontiers_msgs/FrontierReply.h>
#include <frontiers_msgs/FrontierRequest.h>

namespace Frontiers{

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
    const int d3 = 3;
    bool processFrontiersRequest(octomap::OcTree const& octree, frontiers_msgs::FrontierRequest const& request, frontiers_msgs::FrontierReply& reply);

    bool isOccupied(octomath::Vector3 const& grid_coordinates_toTest, octomap::OcTree const& octree);
    bool isExplored(octomath::Vector3 const& grid_coordinates_toTest, octomap::OcTree const& octree);


}
#endif // FRONTIERS_H
