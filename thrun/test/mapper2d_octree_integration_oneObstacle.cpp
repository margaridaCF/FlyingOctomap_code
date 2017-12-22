#define TEST_DIR "@TEST_WITH_DATA_TEST_DIR@"

#include <mapper2d.h>
#include <gtest/gtest.h>
#include <octomap/OcTree.h>
#include <geometry_msgs/Vector3.h>

#include <iostream>
#include <fstream>

namespace mapper {
	class Mapper2DIntegrationOneObstacleTest : public ::testing::Test {
	protected:
	  	Mapper2DIntegrationOneObstacleTest() 
		  	: octree ("data/octomap_tunnel_res02_sample100.bt"),  // 430 ms is the time to load from file 
		  														  // the obstacle is at a distance of 2.1. so out of range
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

		virtual ~Mapper2DIntegrationOneObstacleTest() {
		// You can do clean-up work that doesn't throw exceptions here.
		}

	  
		const octomap::OcTree octree;
		const Mapper2D mapper;
		geometry_msgs::Vector3 robotPosition;
		octomath::Vector3 both_frontDirection;
		octomath::Vector3 both_backwardsDirection;
		octomath::Vector3 both_leftDirection;
		octomath::Vector3 both_rightDirection;
		std::array <Point2d, 4> directions;
		const float z_plane;
	};

	TEST_F(Mapper2DIntegrationOneObstacleTest, DetectsObstacleInFront_Specific) {
        octomath::Vector3 end;

		bool occupied = Mapper2D::isOccupied(Point2d(robotPosition.x, robotPosition.y), 
			z_plane,
			&octree, end, both_frontDirection);
		EXPECT_TRUE(occupied);
		EXPECT_EQ(end.x(), 2.45f);
		EXPECT_EQ(end.y(), 0.45f);
	}

	TEST_F(Mapper2DIntegrationOneObstacleTest, DetectsEmpty) {
        octomath::Vector3 end;
		bool occupied = Mapper2D::isOccupied(Point2d(robotPosition.x, robotPosition.y), z_plane, &octree, end, both_rightDirection);
		EXPECT_FALSE(occupied);
		occupied = Mapper2D::isOccupied(Point2d(robotPosition.x, robotPosition.y), z_plane, &octree, end, both_backwardsDirection);
		EXPECT_FALSE(occupied);
	}

	TEST_F(Mapper2DIntegrationOneObstacleTest, DetectsOccupied) {
        octomath::Vector3 end;
		bool occupied = Mapper2D::isOccupied(Point2d(robotPosition.x, robotPosition.y), z_plane, &octree, end, both_frontDirection);
		EXPECT_TRUE(occupied);
	}

	TEST_F(Mapper2DIntegrationOneObstacleTest, MovingForwardCost) {
		float cost =  Mapper2D::displacementCostXY(Point2d (0.f, 0.f), Point2d (0.f, 1.f));
		EXPECT_EQ(cost, 1);
	}   

	TEST_F(Mapper2DIntegrationOneObstacleTest, CalculateValue_ForOutOfRangeObstacle){
		//Mapper2D::printOccupancyMatrix(robotPosition, &octree);
		Point2d world_front_direction (both_frontDirection.x(), both_frontDirection.y());
		float frontValue = mapper.calculateValueFunction(world_front_direction, &octree);
		float maxCost = Mapper2D::COST_MAXIMUM;
		EXPECT_EQ(maxCost, frontValue);
	} 

	TEST_F(Mapper2DIntegrationOneObstacleTest, CalculateValue_ForEmpty){
		Point2d j (both_backwardsDirection.x(), both_backwardsDirection.y());
		float value = mapper.calculateValueFunction(j, &octree);
		float dueToUndefinedReferenceError = Mapper2D::COST_MAXIMUM;
		EXPECT_EQ(dueToUndefinedReferenceError, value);
	} 


	TEST_F(Mapper2DIntegrationOneObstacleTest, CalculateValue_For4RaysWithObstaclesOutOfRange){
		float value;
		Point2d j;
		for(const Point2d& direction : directions)
		{
			j = Point2d(direction.x, direction.y);
			value = std::max(value, mapper.calculateValueFunction(direction, &octree));
		}
		float maxCost = Mapper2D::COST_MAXIMUM;
		EXPECT_EQ(maxCost, value);
	} 

// More tests 
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
