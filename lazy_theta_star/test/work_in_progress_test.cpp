#include <ltStar_lib_ortho.h>
#include <gtest/gtest.h>
#include <queue>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <visualization_msgs/Marker.h>


#include <chrono>

namespace LazyThetaStarOctree
{
	

	
	TEST(OpenTest, CalculateHeuristicAccrossZeroTest)
	{
		// The heuristic function for (0.1; -11.7; 0.5 ) @ 0.2; g = 0.8; to final = 0.2 parent is 0x7e358df8 ==> 10.10-11.700.50
		// is 10.10 
		// instead of the next bext thing that is around 1.07967
		// 
		//  --> solution heuristics is actually 1, but x is 0.10 ==> as there is decimal part to the number the problem happens
		//  		==> just switched the building key to every number having the same precision
		
		// buildKey(*node)
		
		// ARRANGE
		int initial_object_count = ThetaStarNode::OustandingObjects();
		octomath::Vector3 final_voxel_center(700,0,0);
		Open open (final_voxel_center);
		octomath::Vector3 parent_coordinates(-0.7, -11.7, 0.5 );
		std::shared_ptr<ThetaStarNode> parent = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (parent_coordinates) , 0.2, 0, 1) ;	
		octomath::Vector3 coordinates(0.1, -11.7, 0.5 );
		std::shared_ptr<ThetaStarNode> toInsert = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (coordinates) , 0.2, 0.8, 0.2) ;	
		toInsert->parentNode = parent;
		open.insert(toInsert);
		open.printNodes("========= toInsert ");
		octomath::Vector3 opposite_direction_neighbor_coordinates (-0.9, -11.7, 0.5 );
		std::shared_ptr<ThetaStarNode> opposite_direction_neighbor = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (opposite_direction_neighbor_coordinates) , 0.2, 0.2, 1.2) ;	
		opposite_direction_neighbor->parentNode = parent;
		open.insert(opposite_direction_neighbor);
		open.printNodes("========= opposite_direction_neighbor ");
		// ACT
		// There is no direct way to generate the heuristics of a node as it is a private method
		// ASSERT
		// open.printNodes();
		std::shared_ptr<ThetaStarNode> popped = open.pop();
		ASSERT_TRUE(popped->hasSameCoordinates(toInsert, 0.2));

		open.clear();
		parent = toInsert = opposite_direction_neighbor = popped = NULL;
		ASSERT_EQ(initial_object_count, ThetaStarNode::OustandingObjects()); 
		
	}
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}