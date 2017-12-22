#define TEST_DIR "@TEST_WITH_DATA_TEST_DIR@"

#include <mapper2d.h>
#include <gtest/gtest.h>
#include <octomap/OcTree.h>
#include <geometry_msgs/Vector3.h>

#include <iostream>
#include <fstream>

namespace mapper {
	class Mapper2DIntegrationTest : public ::testing::Test {
	protected:
	  	Mapper2DIntegrationTest() 
		  	: octree("data/20161122_octomap_tunnel_2obstacles_res02_sample100.bt"),	// One obstacle in range and another out of range
		  	mapper (Point2d(0.4f, 0.4f), 0.837664f),
		  	both_frontDirection(1.f, 0.f, 0.f), both_leftDirection(0.f, 1.f, 0.f),
		  	both_backwardsDirection(-1.f, 0.f, 0.f), both_rightDirection(0.f, -1.f, 0.f),
			directions {{Point2d( both_frontDirection.x(), both_frontDirection.y()), 
				Point2d(both_rightDirection.x(), both_rightDirection.y()), 
				Point2d(both_backwardsDirection.x(), both_backwardsDirection.y()), 
				Point2d(both_leftDirection.x(), both_leftDirection.y())}},
				z_plane(0.837664f)
	  	{
			robotPosition.x = 0.4f;
			robotPosition.y = 0.4f;
			robotPosition.z = z_plane;	
			//mapper (robotPosition);
	  	}

		virtual ~Mapper2DIntegrationTest() {
		// You can do clean-up work that doesn't throw exceptions here.
		}

	  
		const octomap::OcTree octree;
		Mapper2D mapper;
		geometry_msgs::Vector3 robotPosition;
		octomath::Vector3 both_frontDirection;
		octomath::Vector3 both_backwardsDirection;
		octomath::Vector3 both_leftDirection;
		octomath::Vector3 both_rightDirection;
		std::array <Point2d, 4> directions;
		const float z_plane;
	};

	TEST_F(Mapper2DIntegrationTest, DetectsEmpty) {
        octomath::Vector3 end;
		bool occupied = Mapper2D::isOccupied(Point2d(robotPosition.x, robotPosition.y), z_plane, &octree, end, both_rightDirection);
		EXPECT_FALSE(occupied);
		occupied = Mapper2D::isOccupied(Point2d(robotPosition.x, robotPosition.y), z_plane, &octree, end, both_backwardsDirection);
		EXPECT_FALSE(occupied);
	}

	TEST_F(Mapper2DIntegrationTest, DetectsOccupied) {
        octomath::Vector3 end;
		bool occupied = Mapper2D::isOccupied(Point2d(robotPosition.x, robotPosition.y), z_plane, &octree, end, both_leftDirection);
		EXPECT_TRUE(occupied);
		occupied = Mapper2D::isOccupied(Point2d(robotPosition.x, robotPosition.y), z_plane, &octree, end, both_frontDirection);
		EXPECT_TRUE(occupied);
	}

	TEST_F(Mapper2DIntegrationTest, DetectsNotOffLimitsForObstacle) {
        octomath::Vector3 end;
		bool occupied = Mapper2D::isOccupied(Point2d(robotPosition.x, robotPosition.y), z_plane, &octree, end, both_leftDirection);
		EXPECT_TRUE(occupied);
		Point2d world_left_direction (both_leftDirection.x(), both_leftDirection.y());
		float leftValue = mapper.calculateValueFunction(world_left_direction, &octree);
		float maxCost = Mapper2D::COST_MAXIMUM;
		EXPECT_EQ(maxCost, leftValue);
	}

	TEST_F(Mapper2DIntegrationTest, MovingForwardCost) {
		float cost =  Mapper2D::displacementCostXY(Point2d (0.f, 0.f), Point2d (0.f, 1.f));
		EXPECT_EQ(cost, 1);
	}   

	TEST_F(Mapper2DIntegrationTest, CalculateValue_ForObstacle){
		//Mapper2D::printOccupancyMatrix(robotPosition, &octree);
		Point2d world_front_direction (both_frontDirection.x(), both_frontDirection.y());
		float frontValue = mapper.calculateValueFunction(world_front_direction, &octree);
		float dueToUndefinedReferenceError = Mapper2D::COST_OFF_LIMITS;
		EXPECT_EQ(dueToUndefinedReferenceError, frontValue);
	} 

	TEST_F(Mapper2DIntegrationTest, CalculateValue_ForEmpty){
		Point2d j (both_backwardsDirection.x(), both_backwardsDirection.y());
		float value = mapper.calculateValueFunction(j, &octree);
		float dueToUndefinedReferenceError = Mapper2D::COST_MAXIMUM;
		EXPECT_EQ(dueToUndefinedReferenceError, value);
	} 

	TEST_F(Mapper2DIntegrationTest, CalculateValue_For4RaysWithObstacles){
		float value;
		Point2d j;
		for(const Point2d& direction : directions)
		{
			j = Point2d(direction.x, direction.y);
			value = std::max(value, mapper.calculateValueFunction(direction, &octree));
		}
		float dueToUndefinedReferenceError = Mapper2D::COST_OFF_LIMITS;
		EXPECT_EQ(dueToUndefinedReferenceError, value);
	} 

	TEST_F(Mapper2DIntegrationTest, UpdateMatrix_AllCells){
		float off_limits = Mapper2D::COST_OFF_LIMITS;
		float maximum = Mapper2D::COST_MAXIMUM;
		for (int y = 0; y < Mapper2D::value_func_dim; ++y)
		{
			for (int x = 0; x < Mapper2D::value_func_dim; ++x)
			{
				octomath::Vector3 grid_point (x, y, 0.f);
				octomath::Vector3 grid_robot_position3D (mapper.grid_robot_position.x, mapper.grid_robot_position.y,  0.f);
				octomath::Vector3 normalized_direction = grid_point - grid_robot_position3D;
				normalized_direction = normalized_direction.normalize();
				float value = mapper.calculateValueFunction_v2(mapper.toWorldCoordinates(Point2d(x, y)), &octree);
				mapper.updateValueFunction(Point2d(x, y), value);

				if(normalized_direction == both_frontDirection)
				{
					EXPECT_EQ(value, off_limits);
				}
				else if(normalized_direction == both_leftDirection)
				{
					EXPECT_EQ(value, maximum);
				}
				else if(normalized_direction == both_rightDirection)
				{
					EXPECT_EQ(value, maximum);
				}
				else if(normalized_direction == both_backwardsDirection)
				{
					EXPECT_EQ(value, maximum);
				}
			}
		}
	}

	TEST_F(Mapper2DIntegrationTest, UpdateMatrix_DedicatedFunction){
		float off_limits = Mapper2D::COST_OFF_LIMITS;
		float maximum = Mapper2D::COST_MAXIMUM;

		mapper.iterateValueFunction(&octree);

		for (int y = 0; y < Mapper2D::value_func_dim; ++y)
		{
			for (int x = 0; x < Mapper2D::value_func_dim; ++x)
			{
				octomath::Vector3 grid_point (x, y, 0.f);
				octomath::Vector3 grid_robot_position3D (mapper.grid_robot_position.x, mapper.grid_robot_position.y,  0.f);
				octomath::Vector3 normalized_direction = grid_point - grid_robot_position3D;
				normalized_direction = normalized_direction.normalize();
				float value = mapper.getValueFunction(Point2d(x, y));

				if(normalized_direction == both_frontDirection)
				{
					EXPECT_EQ(value, off_limits);
				}
				else if(normalized_direction == both_leftDirection)
				{
					EXPECT_EQ(value, maximum);
				}
				else if(normalized_direction == both_rightDirection)
				{
					EXPECT_EQ(value, maximum);
				}
				else if(normalized_direction == both_backwardsDirection)
				{
					EXPECT_EQ(value, maximum);
				}
			}
		}
	}

	TEST_F(Mapper2DIntegrationTest, UpdateMatrix_0_0){
		float off_limits = Mapper2D::COST_OFF_LIMITS;
		float maximum = Mapper2D::COST_MAXIMUM;
		Point2d grid_origin (0.f, 0.f);
		Point2d world_origin = mapper.toWorldCoordinates(grid_origin);
		//ROS_INFO_STREAM("Converted world origin "<<world_origin);
		float value = mapper.calculateValueFunction_v2(world_origin, &octree);
		//mapper.updateValueFunction(Point2d(x, y), value);
		EXPECT_EQ(value, maximum);		
	}

	TEST_F(Mapper2DIntegrationTest, UpdateMatrix_Front){
		float off_limits = Mapper2D::COST_OFF_LIMITS;
		float maximum = Mapper2D::COST_MAXIMUM;

		Point2d world_obstacle (2.f, 0.5f);
		float value = mapper.calculateValueFunction_v2(world_obstacle, &octree);
		//ROS_INFO_STREAM("Obstacle in grid coordinates "<< mapper.toGridCoordinates(world_obstacle));
		EXPECT_EQ(value, off_limits);		
	}


// More tests 
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
