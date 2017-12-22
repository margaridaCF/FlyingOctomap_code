#define TEST_DIR "@TEST_WITH_DATA_TEST_DIR@"

#include <mapper2d.h>
#include <gtest/gtest.h>
#include <octomap/OcTree.h>
#include <geometry_msgs/Vector3.h>

#include <iostream>
#include <fstream>
#include <cmath>

namespace mapper {
	class Mapper2DTest : public ::testing::Test {
	protected:
	  	Mapper2DTest() 
		  	: mapper (Point2d(0.4f, 0.4f), 0.837664f),
		  	frontDirection(1.f, 0.f, 0.f), leftDirection(0.f, -1.f, 0.f),
		  	backwardsDirection(-1.f, 0.f, 0.f), rightDirection(0.f, 1.f, 0.f),
			directions {{Point2d( frontDirection.x(), frontDirection.y()), 
				Point2d(rightDirection.x(), rightDirection.y()), 
				Point2d(backwardsDirection.x(), backwardsDirection.y()), 
				Point2d(leftDirection.x(), leftDirection.y())}},
				z_plane(0.837664f)
	  	{
			robotPosition.x = 0.4f;
			robotPosition.y = 0.4f;
			robotPosition.z = z_plane;	
			//mapper (robotPosition);
	  	}

		virtual ~Mapper2DTest() {
		// You can do clean-up work that doesn't throw exceptions here.
		}

	  
		const Mapper2D mapper;
		geometry_msgs::Vector3 robotPosition;
		octomath::Vector3 frontDirection;
		octomath::Vector3 backwardsDirection;
		octomath::Vector3 leftDirection;
		octomath::Vector3 rightDirection;
		std::array <Point2d, 4> directions;
		const float z_plane;
	};

	TEST_F(Mapper2DTest, ConvertGridOrigin_WorldCoordinates_2_GridIndexes){
		float r = Mapper2D::range;
		Point2d world_gridOrigin (mapper.world_robot_position - r);
		Point2d grid_gridOrigin = mapper.toGridCoordinates(world_gridOrigin);
		EXPECT_EQ(grid_gridOrigin, Point2d(0.f, 0.f));
	} 

	// TEST_F(Mapper2DTest, ConvertRobotPositionSpecific_WorldCoordinates_2_GridIndexes){
	// 	float r = Mapper2D::range;
	// 	float res = Mapper2D::resolution;
	// 	EXPECT_EQ(Point2d(0.4f, 0.4f), mapper.world_robot_position);
	// 	EXPECT_EQ(r, 3.f);
	// 	EXPECT_EQ(res, 0.1f);
	// 	Point2d grid_robotPosition = mapper.toGridCoordinates(mapper.world_robot_position);
	// 	EXPECT_EQ(Point2d(30, 30), grid_robotPosition);
	// }

	TEST_F(Mapper2DTest, ConvertRobotPosition_WorldCoordinates_2_GridIndexes){
		float r = Mapper2D::range;
		float res = Mapper2D::resolution;
		int dim = Mapper2D::value_func_dim;
		// the robot position is exactly in the beginning of the next cell
		Point2d grid_robotPosition_reference = Point2d(dim/2, dim/2);
		Point2d grid_robotPosition_result = mapper.toGridCoordinates(mapper.world_robot_position);
		EXPECT_EQ(grid_robotPosition_reference, grid_robotPosition_result);
	}

	TEST_F(Mapper2DTest, ConvertMaxPosition_WorldCoordinates_2_GridIndexes){
		float r = Mapper2D::range;
		int dim = Mapper2D::value_func_dim;
		float res = Mapper2D::resolution;
		Point2d world_maxPosition = (mapper.world_robot_position + r );
		Point2d grid_max_position = mapper.toGridCoordinates(world_maxPosition);
		EXPECT_EQ(Point2d(dim-1, dim-1), grid_max_position);
	}

	// TEST_F(Mapper2DTest, ConvertWorldDistance_Specific_2_Grid){
	// 	float res = Mapper2D::resolution;
	// 	EXPECT_EQ(res, 0.1f);

	// 	EXPECT_EQ(mapper.toGridDistance(1.f), 10);
	// }

	TEST_F(Mapper2DTest, ConvertWorldDistance_2_Grid){
		float res = Mapper2D::resolution;
		float world_distance = 1.f;
		EXPECT_EQ(mapper.toGridDistance(world_distance), world_distance / res);
	}


	TEST_F(Mapper2DTest, ConvertFrontDirCenterPosition_WorldCoordinates_2_GridIndexes){
		// the robot position is exactly in the beginning of the next cell
		int grid_middle = Mapper2D::value_func_dim / 2 ;
		Point2d world_middle_front_result = mapper.world_robot_position + Point2d(1.f, 0.f);
		Point2d grid_middle_front_result = mapper.toGridCoordinates(world_middle_front_result); 

		//ROS_WARN_STREAM(" [ConvertFrontDirCenterPosition] From "<<world_middle_front_result << " to " << grid_middle_front_result);
		EXPECT_EQ(grid_middle_front_result, Point2d(grid_middle+mapper.toGridDistance(1), grid_middle));
	}

	TEST_F(Mapper2DTest, ConvertWorldOrigin_GridCoordinates_2_WorldIndexes){
		float r = Mapper2D::range;
		Point2d world_gridOrigin (mapper.world_robot_position - r);
		Point2d grid_gridOrigin  (0.f, 0.f);
		EXPECT_EQ(mapper.toWorldCoordinates(grid_gridOrigin), world_gridOrigin);
	} 
	TEST_F(Mapper2DTest, ConvertRobotPosition_GridCoordinates_2_WorldIndexes){
		int dim = Mapper2D::value_func_dim;
		// the robot position is exactly in the beginning of the next cell
		Point2d world_robotPosition_reference = Point2d(0.4, 0.4);
		Point2d world_robotPosition_result = mapper.toWorldCoordinates(Point2d(dim/2, dim/2));
		EXPECT_LT(std::fabs(world_robotPosition_reference.x- world_robotPosition_result.x), 0.01f);
		EXPECT_LT(std::fabs(world_robotPosition_reference.y- world_robotPosition_result.y), 0.01f);
	}

	TEST_F(Mapper2DTest, ConvertMaxPosition_GridCoordinates_2_WorldIndexes){
		float r = Mapper2D::range;
		int dim = Mapper2D::value_func_dim;
		float res = Mapper2D::resolution;
		Point2d world_maxPosition_reference = (mapper.world_robot_position + r ) - res;
		Point2d world_max_position_result = mapper.toWorldCoordinates(Point2d(dim-1, dim-1));
		EXPECT_LT(std::fabs(world_maxPosition_reference.x- world_max_position_result.x), 0.01f);
		EXPECT_LT(std::fabs(world_maxPosition_reference.y- world_max_position_result.y), 0.01f);
	}

	TEST_F(Mapper2DTest, ConvertMaxPosition_X_GridCoordinates_2_WorldIndexes){
		float r = Mapper2D::range;
		int dim = Mapper2D::value_func_dim;
		float res = Mapper2D::resolution;
		Point2d world_maxPosition_reference = (mapper.world_robot_position + r ) - res;
		world_maxPosition_reference.y = (mapper.world_robot_position - r ).y;
		Point2d world_max_position_result = mapper.toWorldCoordinates(Point2d(dim-1, 0));
		EXPECT_LT(std::fabs(world_maxPosition_reference.x- world_max_position_result.x), 0.01f);
		EXPECT_LT(std::fabs(world_maxPosition_reference.y- world_max_position_result.y), 0.01f);
	}

// More tests 
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
