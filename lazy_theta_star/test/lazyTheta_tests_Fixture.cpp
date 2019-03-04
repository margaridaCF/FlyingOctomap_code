#include <ltStar_lib_ortho.h>
#include <gtest/gtest.h>


namespace LazyThetaStarOctree{

	class LazyThetaStarTests : public ::testing::Test 
	{
	public:
	protected:
	  	LazyThetaStarTests() 
	  		:final_voxel_center(700,0,0), open(final_voxel_center)
	  	{
	  		
	  		x_start = std::make_shared<ThetaStarNode>(
				std::make_shared<octomath::Vector3>(octomath::Vector3 (0, 0, 0)), 
				0.2f);
	  		x_s1 = std::make_shared<ThetaStarNode> (
				std::make_shared<octomath::Vector3>(octomath::Vector3 (1, 0, 0)), 
				0.2f);
	  		x_goal  = std::make_shared<ThetaStarNode> (
				std::make_shared<octomath::Vector3>(octomath::Vector3 (5, 0, 0)), 
				0.2f);
	  		x_distantNode = std::make_shared<ThetaStarNode>  (
				std::make_shared<octomath::Vector3>(octomath::Vector3 (20, 0, 0)), 
				0.2f);
			/// For x axis updateVertex
			x_start->distanceFromInitialPoint = 0.f;
			x_start->lineDistanceToFinalPoint = weightedDistance(  *(x_start->coordinates), *(x_goal->coordinates)  );
			x_start->parentNode = x_start;
			x_s1->parentNode = x_start;
			x_s1->distanceFromInitialPoint = weightedDistance(  *(x_start->coordinates), *(x_s1->coordinates)  );
			x_s1->lineDistanceToFinalPoint =  weightedDistance(  *(x_s1->coordinates), *(x_goal->coordinates)  );
			x_goal->lineDistanceToFinalPoint = 0.f;
			x_distantNode->lineDistanceToFinalPoint = weightedDistance(  *(x_distantNode->coordinates), *(x_goal->coordinates)  );
			open.insert(x_start);
			open.insert(x_s1);

			starting_nodes_count = ThetaStarNode::OustandingObjects();
	  	}
		virtual ~LazyThetaStarTests() {}

		std::shared_ptr<ThetaStarNode> x_start, x_s1, x_goal, x_distantNode;
		int starting_nodes_count;
		octomath::Vector3 final_voxel_center;
		Open open ;		
	};

	
	TEST_F(LazyThetaStarTests, ComputeCostStraightLineTest)
	{
		std::shared_ptr<ThetaStarNode> s_neighbour = x_distantNode;
		std::shared_ptr<ThetaStarNode> target = x_s1;
		// ACT
		float cost = CalculateCost(*target, *s_neighbour);
		// ASSERT
		ASSERT_EQ(starting_nodes_count, ThetaStarNode::OustandingObjects());
		// ln 32 g(s') := g(parent(s)) + c(parent(s), s');
		float new_g = target->parentNode->distanceFromInitialPoint + weightedDistance(  *(target->parentNode->coordinates), *(s_neighbour->coordinates)  );
		ASSERT_EQ(cost, new_g);
	}


	// Stops when reaching start point
	// Add in the correct order
	// Includes both start and end nodes supplied
	TEST_F(LazyThetaStarTests, ExtractPathTest)
	{
		std::set<std::shared_ptr<ThetaStarNode>> nodes_to_free;
		// ARRANGE
		std::list<octomath::Vector3> path;
		std::shared_ptr<ThetaStarNode> start = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (octomath::Vector3(0.f, 0.f, -10.f)), 0.4f, 0.f, 0.f);
		start->parentNode      = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (octomath::Vector3(-1.f, 0.f, 0.f)), 0.4f, 0.f, 0.f);
		std::shared_ptr<ThetaStarNode> end   = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (octomath::Vector3(10.f, 0.f, -10.f)), 0.4f, 0.f, 0.f);
		nodes_to_free.insert(start);
		nodes_to_free.insert(end);
		nodes_to_free.insert(start->parentNode );
		std::list <octomath::Vector3> points_for_path {
			octomath::Vector3  (9.f, 0.f, 0.f),
			octomath::Vector3  (8.f, 0.f, 0.f),
			octomath::Vector3  (7.f, 0.f, 0.f),
			octomath::Vector3  (6.f, 0.f, 0.f),
			octomath::Vector3  (5.f, 0.f, 0.f),
			octomath::Vector3  (4.f, 0.f, 0.f),
			octomath::Vector3  (3.f, 0.f, 0.f),
			octomath::Vector3  (2.f, 0.f, 0.f),
			octomath::Vector3  (1.f, 0.f, 0.f),
		};
		std::shared_ptr<ThetaStarNode>  new_point ;
		std::shared_ptr<ThetaStarNode> current = end;
		for(octomath::Vector3 coordinates : points_for_path)
		{			
			new_point = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3>(coordinates), 0.2f, 0.f, 0.f);
			current->parentNode = new_point;
			current = new_point;
			nodes_to_free.insert(new_point);
		}	
		new_point->parentNode = start;
		// ACT
		extractPath(path, *start, *end);
		// ASSERT
		octomath::Vector3 first = *path.begin();
		EXPECT_EQ(path.size(), points_for_path.size()+2);
		EXPECT_TRUE(first == *(start->coordinates));
		octomath::Vector3 last = *path.rbegin();
		EXPECT_TRUE(last == *(end->coordinates));
		int aux_count = 0;
		for(octomath::Vector3 coordinates : path)
		{
			EXPECT_EQ(coordinates.x(), aux_count);
			aux_count++;

		}
		nodes_to_free.clear();
		start->parentNode = end = new_point = current = NULL;
		ASSERT_EQ(starting_nodes_count, ThetaStarNode::OustandingObjects());
	}


	// TODO - TEST CASES 
	// test for when s has as parent node a node that is not the starting point ==> needs obstacle


	TEST_F(LazyThetaStarTests, UpdateVertex_NewVertex_addFirstNeighbor_Test)
	{
		// ARRANGE
		open.erase(*x_s1);
		open.erase(*x_start);
		x_s1->distanceFromInitialPoint = 1000000;
		x_s1->parentNode = NULL;
		ASSERT_TRUE(open.empty());
		ASSERT_EQ(x_start->distanceFromInitialPoint, 0);
		ASSERT_GT(x_s1->distanceFromInitialPoint, 100000);
		ASSERT_EQ(x_s1->lineDistanceToFinalPoint, weightedDistance(  *(x_s1->coordinates), *(x_goal->coordinates)  ));
		ASSERT_FALSE(x_s1->parentNode);
		ASSERT_TRUE( (x_start->parentNode) != NULL);
		// ACT
		UpdateVertex(*x_start, x_s1, open);
		// ASSERT
		ASSERT_FALSE(open.empty());
		ASSERT_EQ(x_start->distanceFromInitialPoint, 0);
		ASSERT_EQ(x_s1->distanceFromInitialPoint, weightedDistance(  *(x_start->coordinates), *(x_s1->coordinates)  ));
		ASSERT_EQ(x_s1->lineDistanceToFinalPoint, weightedDistance(  *(x_s1->coordinates), *(x_goal->coordinates)  ));
		ASSERT_EQ(x_s1->parentNode, x_start);
		std::shared_ptr<ThetaStarNode> poped = open.pop();
		EXPECT_TRUE( *(poped->coordinates) == *(x_s1->coordinates) );
		ASSERT_EQ(starting_nodes_count, ThetaStarNode::OustandingObjects());
	}


	TEST_F(LazyThetaStarTests, UpdateVertex_NewVertex_addSecondNeighbor_Test)
	{
		// ARRANGE
		open.erase(*x_s1);
		std::shared_ptr<ThetaStarNode> x_s2 = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3>(octomath::Vector3 (-1, 0, 0)), 0.2f);
		open.erase(*x_start);
		x_s1->parentNode = NULL;
		x_s1->distanceFromInitialPoint = 1000000;
		x_s2->parentNode = NULL;
		x_s2->distanceFromInitialPoint = 1000000;
		x_s2->lineDistanceToFinalPoint = weightedDistance(  *(x_s2->coordinates), *(x_goal->coordinates)  );
		ASSERT_TRUE( (x_start->parentNode) != NULL);
		UpdateVertex(*x_start, x_s1, open);
		ASSERT_FALSE(open.empty());
		ASSERT_EQ(x_start->distanceFromInitialPoint, 0);
		ASSERT_EQ(x_s1->distanceFromInitialPoint, weightedDistance(  *(x_start->coordinates), *(x_s1->coordinates)  ));
		ASSERT_EQ(x_s1->lineDistanceToFinalPoint, weightedDistance(  *(x_s1->coordinates), *(x_goal->coordinates)  ));
		ASSERT_EQ(x_s1->parentNode, x_start);
		ASSERT_GT(x_s2->distanceFromInitialPoint, 100000);
		ASSERT_EQ(x_s2->lineDistanceToFinalPoint, weightedDistance(  *(x_s2->coordinates), *(x_goal->coordinates)  ));
		ASSERT_FALSE(x_s2->parentNode);


		// ACT
		UpdateVertex(*x_start, x_s2, open);


		// ASSERT
		ASSERT_EQ(open.size(), 2);
		ASSERT_EQ(x_start->distanceFromInitialPoint, 0);
		// x_s1
		ASSERT_EQ(x_s1->distanceFromInitialPoint, weightedDistance(  *(x_start->coordinates), *(x_s1->coordinates)  ));
		ASSERT_EQ(x_s1->lineDistanceToFinalPoint, weightedDistance(  *(x_s1->coordinates), *(x_goal->coordinates)  ));
		ASSERT_EQ(x_s1->parentNode, x_start);
		// x_s2
		ASSERT_EQ(x_s2->distanceFromInitialPoint, weightedDistance(  *(x_start->coordinates), *(x_s2->coordinates)  ));
		ASSERT_EQ(x_s2->lineDistanceToFinalPoint, weightedDistance(  *(x_s2->coordinates), *(x_goal->coordinates)  ));
		ASSERT_EQ(x_s2->parentNode, x_start);
		std::shared_ptr<ThetaStarNode> poped = open.pop();
		EXPECT_FALSE( *(poped->coordinates) == *(x_s2->coordinates) );
		poped = open.pop();
		EXPECT_TRUE( *(poped->coordinates) == *(x_s2->coordinates) );

	}

	// 2nd N.       START & CURRENT	     1sr N. 	  3rd N.				GOAL
	TEST_F(LazyThetaStarTests, UpdateVertex_NewVertex_addThirdNeighbor_Test)
	{
		// ARRANGE
		open.erase(*x_s1);
		open.erase(*x_start);
		x_s1->parentNode = NULL;
		x_s1->distanceFromInitialPoint = 1000000;
		std::shared_ptr<ThetaStarNode>  x_s2 = std::make_shared<ThetaStarNode>( std::make_shared<octomath::Vector3> (octomath::Vector3 (-1, 0, 0)), 0.2f);
		x_s2->parentNode = NULL;
		x_s2->distanceFromInitialPoint = 1000000;
		x_s2->lineDistanceToFinalPoint = weightedDistance(  *(x_s2->coordinates), *(x_goal->coordinates)  );
		std::shared_ptr<ThetaStarNode>  x_s3 = std::make_shared<ThetaStarNode>( std::make_shared<octomath::Vector3> (octomath::Vector3 (3, 0, 0)), 0.2f);
		x_s3->parentNode = NULL;
		x_s3->distanceFromInitialPoint = 1000000;
		x_s3->lineDistanceToFinalPoint = weightedDistance(  *(x_s3->coordinates), *(x_goal->coordinates)  );
		ASSERT_EQ(open.size(), 0);
		UpdateVertex(*x_start, x_s1, open);
		ASSERT_FALSE(open.empty());
		ASSERT_EQ(open.size(), 1);
		ASSERT_EQ(x_start->distanceFromInitialPoint, 0);
		ASSERT_EQ(x_s1->distanceFromInitialPoint, weightedDistance(  *(x_start->coordinates), *(x_s1->coordinates)  ));
		ASSERT_EQ(x_s1->lineDistanceToFinalPoint, weightedDistance(  *(x_s1->coordinates), *(x_goal->coordinates)  ));
		ASSERT_EQ(x_s1->parentNode, x_start);
		// N_2
		ASSERT_GT(x_s2->distanceFromInitialPoint, 100000);
		ASSERT_EQ(x_s2->lineDistanceToFinalPoint, weightedDistance(  *(x_s2->coordinates), *(x_goal->coordinates)  ));
		ASSERT_FALSE(x_s2->parentNode);
		UpdateVertex(*x_start, x_s2, open);
		ASSERT_EQ(open.size(), 2);
		ASSERT_EQ(x_start->distanceFromInitialPoint, 0);
		// x_s1
		ASSERT_EQ(x_s1->distanceFromInitialPoint, weightedDistance(  *(x_start->coordinates), *(x_s1->coordinates)  ));
		ASSERT_EQ(x_s1->lineDistanceToFinalPoint, weightedDistance(  *(x_s1->coordinates), *(x_goal->coordinates)  ));
		ASSERT_EQ(x_s1->parentNode, x_start);
		// x_s2
		ASSERT_EQ(x_s2->distanceFromInitialPoint, weightedDistance(  *(x_start->coordinates), *(x_s2->coordinates)  ));
		ASSERT_EQ(x_s2->lineDistanceToFinalPoint, weightedDistance(  *(x_s2->coordinates), *(x_goal->coordinates)  ));
		ASSERT_EQ(x_s2->parentNode, x_start);
		// N 3

		ASSERT_GT(x_s3->distanceFromInitialPoint, 100000);
		ASSERT_EQ(x_s3->lineDistanceToFinalPoint, weightedDistance(  *(x_s3->coordinates), *(x_goal->coordinates)  ));
		ASSERT_FALSE(x_s3->parentNode);
		x_s3->parentNode = NULL;
		x_s3->distanceFromInitialPoint = 1000000;
		x_s3->lineDistanceToFinalPoint = weightedDistance(  *(x_s3->coordinates), *(x_goal->coordinates)  );


		// ACT
		UpdateVertex(*x_start, x_s3, open);

		// ASSERT		
		ASSERT_EQ(open.size(), 3);
		ASSERT_EQ(x_start->distanceFromInitialPoint, 0);
		// n 3 update
		ASSERT_EQ(x_s3->distanceFromInitialPoint, weightedDistance(  *(x_start->coordinates), *(x_s3->coordinates)  ));
		ASSERT_EQ(x_s3->lineDistanceToFinalPoint, weightedDistance(  *(x_s3->coordinates), *(x_goal->coordinates)  ));
		ASSERT_EQ(x_s3->parentNode, x_start);
		
		std::shared_ptr<ThetaStarNode> poped = open.pop();
		EXPECT_FALSE( *(poped->coordinates) == *(x_s2->coordinates) );
		poped = open.pop();
		EXPECT_FALSE( *(poped->coordinates) == *(x_s2->coordinates) );
		poped = open.pop();
		EXPECT_TRUE( *(poped->coordinates) == *(x_s2->coordinates) );

		ASSERT_EQ(starting_nodes_count+2, ThetaStarNode::OustandingObjects());
	}

	// 2nd N.       START & CURRENT	     1sr N. 	 			 4rd N.		GOAL
	TEST_F(LazyThetaStarTests, UpdateVertex_NewVertex_addFirstNeighbor_PopingPrevNeighbor_Test)
	{
		// ARRANGE
		open.erase(*x_s1);
		open.erase(*x_start);
		x_s1->parentNode = NULL;
		x_s1->distanceFromInitialPoint = 1000000;
		std::shared_ptr<ThetaStarNode>  x_s2  = std::make_shared<ThetaStarNode>( std::make_shared<octomath::Vector3> (octomath::Vector3 (-1, 0, 0)), 0.2f);
		x_s2->parentNode = NULL;
		x_s2->distanceFromInitialPoint = 1000000;
		x_s2->lineDistanceToFinalPoint = weightedDistance(  *(x_s2->coordinates), *(x_goal->coordinates)  );
		UpdateVertex(*x_start, x_s1, open);
		UpdateVertex(*x_start, x_s2, open);
		std::shared_ptr<ThetaStarNode>  x_s4  = std::make_shared<ThetaStarNode> ( std::make_shared<octomath::Vector3> (octomath::Vector3 (4, 0, 0)), 0.2f);
		x_s4->parentNode = NULL;
		x_s4->distanceFromInitialPoint = 1000000;
		x_s4->lineDistanceToFinalPoint = weightedDistance(  *(x_s4->coordinates), *(x_goal->coordinates)  );


		// ACT
		// ln 7
		std::shared_ptr<ThetaStarNode> poped = open.pop();
		EXPECT_TRUE( *(poped->coordinates) == *(x_s1->coordinates) );
		// (the path is not found, we are now analyzing neighbors)
		// ln 17
		UpdateVertex(*poped, x_s4, open);


		// ASSERT
		ASSERT_EQ(open.size(), 2);
		// n 4 update
		ASSERT_EQ(x_s4->distanceFromInitialPoint, weightedDistance(  *(x_start->coordinates), *(x_s4->coordinates)  ));
		ASSERT_EQ(x_s4->lineDistanceToFinalPoint, weightedDistance(  *(x_s4->coordinates), *(x_goal->coordinates)  ));
		ASSERT_EQ(x_s4->parentNode, x_start);
		// open order
		poped = open.pop();
		EXPECT_TRUE( *(poped->coordinates) == *(x_s4->coordinates) );
		poped = open.pop();
		EXPECT_TRUE( *(poped->coordinates) == *(x_s2->coordinates) );
		
		ASSERT_EQ(starting_nodes_count+2, ThetaStarNode::OustandingObjects());
	}
	
	// Test memory leaks in pointers of neighbors
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}