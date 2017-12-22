#ifndef MAPPER2D_H
#define MAPPER2D_H

#include <point2d_lib.h>
#include <geometry_msgs/Vector3.h>
#include <octomap/math/Vector3.h>
#include <octomap/OcTree.h>
#include <ros/ros.h>

#include <list>

namespace mapper
{
      enum directions { front = 0, left = 1, right = 2, back = 3};
      enum InformationGain { unexplored, explored };
      // Point2d & geometry_msgs::Vector3
	class Mapper2D
	{
	public:
            /////  VARIABLES
            //     --  SENSOR  --
            static constexpr float range = 3.f;
            static constexpr float resolution = 0.1f; // it has to be static because it affects the dimension of the std::array informationGain
            static constexpr float world_min_safety_distance = 1.5;
            static constexpr float world_next_step_distance = Mapper2D::resolution * 2;
            float world_near_distance = 1;
            //     --  POSITIONING  --
            const Point2d world_robot_position;
            Point2d grid_robot_position;
            const float z_plane;
            //     --  VALUE FUNCTION  --
            static constexpr float COST_OFF_LIMITS = 7000000;
            static constexpr float COST_MAXIMUM    = 1000000;
            static const int value_func_dim = (range*2/resolution);

            /////  FUNCTIONS
            Mapper2D(Point2d world_robot_position, float z_plane);
            float       calculateValueFunction(Point2d const& world_direction, const octomap::OcTree* octree) const;
            float       calculateValueFunction_v2(Point2d const& world_target_point, const octomap::OcTree* octree) const;
            void        iterateValueFunction(const octomap::OcTree* octree);
            void        updateInformationGain(Point2d const& grid_coordinates, InformationGain value);
            void        updateValueFunction(Point2d const& grid_coordinates, float value);
            float       getValueFunction(Point2d const& grid_coordinates) const;

            bool        findFrontierCells(std::list<Point2d>& frontierCells) const;

            Point2d     toWorldCoordinates(Point2d const& grid_coordinates) const;
            int         toGridDistance(float world_distance) const;
            Point2d     toGridCoordinates(Point2d const& world_coordinates) const;
            Point2d     toGridCoordinatesDEBUG(Point2d const& world_coordinates) const;
            bool        validateGridCoordinates(Point2d grid_coordinates) const;
            // These are just utility functions related with octree
            void        printInformationGainMatrix() const;
            void        printValueFunctionMatrix() const;
            static void printOccupancyMatrixRealityCheck(
                  const geometry_msgs::Vector3 translation, 
                  const octomap::OcTree* octree) ;
            static void printOccupancyMatrix(
                  const geometry_msgs::Vector3 translation, 
                  const octomap::OcTree* octree) ;
            // Corner stone functions turned static to be tested with unit tests
            static bool isOccupied(
                  const Point2d origin, 
                  const float z,
                  const octomap::OcTree* octree,  
                  octomath::Vector3& end, 
                  const octomath::Vector3 direction = octomath::Vector3(0, 1, 0)) ;
            static float displacementCostXY(Point2d const& curr_position, Point2d const& destination);

	private:
            const std::array<Point2d, 8> rayDirections;
            Point2d grid_offset;
            float        valueFunctionByGridCoordinates(Point2d const& grid_coordinates) const;
            std::array< std::array<float, value_func_dim>, value_func_dim> valueFunction;
            std::array< std::array<InformationGain, value_func_dim>, value_func_dim> informationGain;
	};
}

#endif // MAPPER2D_H
