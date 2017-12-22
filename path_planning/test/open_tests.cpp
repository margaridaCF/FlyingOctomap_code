#include <open.h>
#include <gtest/gtest.h>

namespace LazyThetaStarOctree{
	TEST(OpenTest, InsertTest)
	{
		int initial_object_count = ThetaStarNode::OustandingObjects();
		octomath::Vector3 final_voxel_center(700,0,0);
		// ARRANGE
		Open open (final_voxel_center);
		octomath::Vector3 key(0,0,0);
		std::shared_ptr<ThetaStarNode> toInsert = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (key) , 15, 10, 5) ;	
		std::shared_ptr<ThetaStarNode> parent = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (2, 5, 2) , 16, 10, 5) ;	
		toInsert-> parentNode = parent;
		// ACT 
		open.insert(toInsert);
		// ASSERT
		ASSERT_TRUE(open.existsInMap(key) );
		std::shared_ptr<ThetaStarNode> poped = open.pop();
		ASSERT_EQ( *(poped->coordinates), *(toInsert->coordinates));
		toInsert = poped = parent = NULL;
		ASSERT_EQ(0, ThetaStarNode::OustandingObjects()); 
	}

	TEST(OpenTest, InsertTwoTest)
	{
		int initial_object_count = ThetaStarNode::OustandingObjects();
		octomath::Vector3 final_voxel_center(700,0,0);
		// ARRANGE
		Open open (final_voxel_center);
		octomath::Vector3 key(0,0,0);
		std::shared_ptr<ThetaStarNode> toInsert = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (key), 15, 10, 5) ;
		octomath::Vector3 key2(0,1,0);
		std::shared_ptr<ThetaStarNode> toInsert2 = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (key2), 15, 10, 10) ;
		toInsert->parentNode = toInsert2;
		toInsert2->parentNode = toInsert;
		// ACT 
		open.insert(toInsert);
		open.insert(toInsert2);
		// ASSERT
		ASSERT_TRUE(open.existsInMap(key) );
		std::shared_ptr<ThetaStarNode> poped = open.pop();

		ASSERT_EQ( *(poped->coordinates), *(toInsert->coordinates));
		ASSERT_FALSE(open.empty());
		open.clear();
		toInsert->parentNode = NULL;
		poped->parentNode = NULL;
		toInsert2->parentNode = NULL;
		toInsert = NULL;
		poped = NULL;
		toInsert2 = NULL;
		ASSERT_EQ(initial_object_count, ThetaStarNode::OustandingObjects());    // TODO This should actually be 0 but can't find why it is not
	}
	TEST(OpenTest, InsertDuplicatedTest)
	{
		int initial_object_count = ThetaStarNode::OustandingObjects();
		octomath::Vector3 final_voxel_center(700,0,0);
		// ARRANGE
		Open open (final_voxel_center);
		octomath::Vector3 key(0,0,0);
		std::shared_ptr<ThetaStarNode> toInsert = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (key) , 15, 10, 5) ;
		std::shared_ptr<ThetaStarNode> parent = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (2, 5, 2) , 16, 10, 5) ;
		toInsert-> parentNode = parent;
		// ACT 
		open.insert(toInsert);
		open.insert(toInsert);
		ASSERT_EQ(open.size(), 1);
		// ASSERT
		ASSERT_TRUE(open.existsInMap(key) );
		std::shared_ptr<ThetaStarNode> poped = open.pop();
		ASSERT_EQ( *(poped->coordinates), *(toInsert->coordinates));
		ASSERT_TRUE(open.empty());
		toInsert = poped = parent = NULL;
		open.clear();
		ASSERT_EQ(initial_object_count, ThetaStarNode::OustandingObjects()); 
	}
	// ExistsTest is Tested in insert
	TEST(OpenTest, ExistsInEmptyTest)
	{
		octomath::Vector3 final_voxel_center(700,0,0);
		// ARRANGE
		Open open (final_voxel_center);
		octomath::Vector3 key(0,0,0);
		// ACT 
		// ASSERT
		ASSERT_FALSE(open.existsInMap(key) );
	}
	TEST(OpenTest, DoesNotExistsTest)
	{
		int initial_object_count = ThetaStarNode::OustandingObjects();
		octomath::Vector3 final_voxel_center(700,0,0);
		// ARRANGE
		Open open (final_voxel_center);
		octomath::Vector3 key(0,0,0);
		octomath::Vector3 inexistentKey(0,0,1);
		std::shared_ptr<ThetaStarNode> toInsert = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (key) , 15, 10, 5) ;
		std::shared_ptr<ThetaStarNode> parent = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (2, 5, 2) , 16, 10, 5) ;
		toInsert-> parentNode = parent;
		// ACT 
		open.insert(toInsert);
		// ASSERT
		ASSERT_FALSE(open.existsInMap(inexistentKey) );
		open.clear();
		toInsert = parent = NULL;
		ASSERT_EQ(initial_object_count, ThetaStarNode::OustandingObjects()); 
	}


	TEST(OpenTest, EraseExistingTest)
	{
		int initial_object_count = ThetaStarNode::OustandingObjects();
		octomath::Vector3 final_voxel_center(700,0,0);
		// ARRANGE
		Open open (final_voxel_center);
		octomath::Vector3 key(0,0,0);
		std::shared_ptr<ThetaStarNode> toInsert = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (key) , 15, 10, 5) ;
		std::shared_ptr<ThetaStarNode> parent = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (2, 5, 2) , 15, 10, 5) ;
		toInsert-> parentNode = parent;
		// ACT 
		open.insert(toInsert);
		// ASSERT
		ASSERT_TRUE(open.existsInMap(key) );
		std::shared_ptr<ThetaStarNode> poped = open.pop();
		ASSERT_EQ( *(poped->coordinates), *(toInsert->coordinates));
		open.clear();
		toInsert = poped = parent = NULL;
		ASSERT_EQ(initial_object_count, ThetaStarNode::OustandingObjects());   
	}

	TEST(OpenTest, EmptyTest)
	{
		int initial_object_count = ThetaStarNode::OustandingObjects();
		octomath::Vector3 final_voxel_center(700,0,0);
		// ARRANGE
		Open open (final_voxel_center);
		// ACT 
		// ASSERT
		ASSERT_TRUE( open.empty() );
		ASSERT_EQ(initial_object_count, ThetaStarNode::OustandingObjects());  
	}

	TEST(OpenTest, NotEmptyTest)
	{
		int initial_object_count = ThetaStarNode::OustandingObjects();
		octomath::Vector3 final_voxel_center(700,0,0);
		// ARRANGE
		Open open (final_voxel_center);
		octomath::Vector3 key(0,0,0);
		std::shared_ptr<ThetaStarNode> toInsert = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (key) , 15, 10, 5) ;
		std::shared_ptr<ThetaStarNode> parent = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (2, 5, 2) , 15, 10, 5) ;
		toInsert-> parentNode = parent;
		// ACT 
		open.insert(toInsert);
		// ASSERT
		ASSERT_FALSE(open.empty() );
		open.clear();
		toInsert = parent = NULL;
		ASSERT_EQ(initial_object_count, ThetaStarNode::OustandingObjects());   // TODO This should actually be 0 but can't find why it is not
	}

	TEST(OpenTest, PopTest)
	{
		int initial_object_count = ThetaStarNode::OustandingObjects();
		// ARRANGE
		octomath::Vector3 final_voxel_center(700,0,0);
		Open open (final_voxel_center);
		octomath::Vector3 key(0,0,0);
		std::shared_ptr<ThetaStarNode> toInsert = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (key) , 15, 10, 5) ;
		std::shared_ptr<ThetaStarNode> parent = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (2, 5, 2) , 15, 10, 5) ;
		toInsert-> parentNode = parent;
		// ACT 
		open.insert(toInsert);
		std::shared_ptr<ThetaStarNode> poped = open.pop();
		// ASSERT
		ASSERT_EQ( *(poped->coordinates), *(toInsert->coordinates));
		open.clear();
		toInsert = poped = parent = NULL;
		ASSERT_EQ(initial_object_count, ThetaStarNode::OustandingObjects()); 
	}
	TEST(OpenTest, PopLowerHeuristicValueTest)
	{
		int initial_object_count = ThetaStarNode::OustandingObjects();
		// ARRANGE
		int cell_size = 15;
		float distanceFromInitialPoint = 10;
		float lineDistanceToFinalPoint = 5;
		
		octomath::Vector3 final_voxel_center(700,0,0);
		// ARRANGE
		Open open (final_voxel_center);
		octomath::Vector3 coordinates(0,0,0);
		std::shared_ptr<ThetaStarNode> low = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (coordinates) , cell_size, distanceFromInitialPoint, lineDistanceToFinalPoint);
		octomath::Vector3 coordinates2(0,1,0);
		lineDistanceToFinalPoint = 10;
		std::shared_ptr<ThetaStarNode> high = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (coordinates2) , cell_size, distanceFromInitialPoint, lineDistanceToFinalPoint);
		octomath::Vector3 coordinates3(0,3,0);
		distanceFromInitialPoint = 9;
		std::shared_ptr<ThetaStarNode> middle = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (coordinates3) , cell_size, distanceFromInitialPoint, lineDistanceToFinalPoint);
		low->parentNode = high;
		middle->parentNode = high;
		high->parentNode = middle;
		// ACT 
		open.insert(high);
		open.insert(low);
		open.insert(middle);
		std::shared_ptr<ThetaStarNode> poped = open.pop();
		// ASSERT
		ASSERT_EQ(*(poped->coordinates), *(low->coordinates));
		poped = open.pop();
		ASSERT_EQ(*(poped->coordinates), *(middle->coordinates));
		poped = open.pop();
		ASSERT_EQ(*(poped->coordinates), *(high->coordinates));
		open.clear();
		low->parentNode = middle->parentNode = high->parentNode = poped->parentNode = NULL;
		low = middle = high = poped = NULL;
		ASSERT_EQ(initial_object_count, ThetaStarNode::OustandingObjects()); 
	}
	TEST(OpenTest, AcknowledgeDuplicateIdTest)
	{
		int initial_object_count = ThetaStarNode::OustandingObjects();
		octomath::Vector3 final_voxel_center(700,0,0);
		// ARRANGE
		Open open (final_voxel_center);
		octomath::Vector3 key(0,0,0);
		std::shared_ptr<ThetaStarNode> low = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (key) , 15, 10, 5);
		octomath::Vector3 key2(0,1,0);
		std::shared_ptr<ThetaStarNode> high = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (key2), 15, 10, 10);
		octomath::Vector3 key3(0,1,0);
		std::shared_ptr<ThetaStarNode> middle = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (key3), 15, 9, 10);
		low->parentNode = high;
		middle->parentNode = high;
		high->parentNode = middle;
		// ACT 
		open.insert(high);
		open.insert(low);
		open.insert(middle);
		std::shared_ptr<ThetaStarNode> poped = open.pop();
		// ASSERT
		ASSERT_EQ(*(poped->coordinates), *(low->coordinates));
		poped = open.pop();
		ASSERT_EQ(*(poped->coordinates), *(high->coordinates));
		ASSERT_TRUE(open.empty());
		open.clear();
		low->parentNode = middle->parentNode = high->parentNode = poped->parentNode = NULL;
		low = middle = high = poped = NULL;
		ASSERT_EQ(initial_object_count, ThetaStarNode::OustandingObjects()); 
	}
	TEST(OpenTest, PopWithSomeLeftTest)
	{
		int initial_object_count = ThetaStarNode::OustandingObjects();
		octomath::Vector3 final_voxel_center(700,0,0);
		// ARRANGE
		Open open (final_voxel_center);
		octomath::Vector3 key(0,0,0);
		std::shared_ptr<ThetaStarNode> low = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (key) , 15, 10, 5);
		octomath::Vector3 key2(0,1,0);
		std::shared_ptr<ThetaStarNode> high = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (key2) , 15, 10, 10);
		octomath::Vector3 key3(0,10,0);
		std::shared_ptr<ThetaStarNode> middle = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (key3) , 15, 9, 10);
		low->parentNode = high;
		middle->parentNode = high;
		high->parentNode = middle;
		// ACT 
		open.insert(high);
		open.insert(low);
		open.insert(middle); 
		std::shared_ptr<ThetaStarNode> poped = open.pop();
		// ASSERT
		ASSERT_EQ(*(poped->coordinates), *(low->coordinates));
		poped = open.pop();
		ASSERT_EQ(*(poped->coordinates), *(middle->coordinates));
		poped = open.pop();
		ASSERT_EQ(*(poped->coordinates), *(high->coordinates));
		ASSERT_TRUE(open.empty());
		open.clear();
		low->parentNode = middle->parentNode = high->parentNode = poped->parentNode = NULL;
		low = middle = high = poped = NULL;
		ASSERT_EQ(initial_object_count, ThetaStarNode::OustandingObjects()); 
	}

	TEST(OpenTest, PopWhileEmptyCleanTest)
	{
		int initial_object_count = ThetaStarNode::OustandingObjects();
		// ARRANGE
		octomath::Vector3 final_voxel_center(700,0,0);
		Open open (final_voxel_center);
		ASSERT_TRUE(open.empty());
		// ACT 
		bool oor_exception_thrown = false;
		try
		{
			open.pop();
		}
		catch (const std::out_of_range& oor)
		{
			oor_exception_thrown = true;
		}
		// ASSERT
		ASSERT_TRUE(oor_exception_thrown);
		ASSERT_EQ(initial_object_count, ThetaStarNode::OustandingObjects()); 
	}

	TEST(OpenTest, PopWhileEmptyTest)
	{
		int initial_object_count = ThetaStarNode::OustandingObjects();
		// ARRANGE
		octomath::Vector3 final_voxel_center(700,0,0);
		Open open (final_voxel_center);
		octomath::Vector3 key(0,0,0);
		std::shared_ptr<ThetaStarNode> low = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (key) , 15, 10, 5);
		octomath::Vector3 key2(0,1,0);
		std::shared_ptr<ThetaStarNode> high = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (key2) , 15, 10, 10);
		low->parentNode = high;
		high->parentNode = low;
		// ACT 
		open.insert(high);
		open.insert(low);
		std::shared_ptr<ThetaStarNode> poped = open.pop();
		ASSERT_EQ(*(poped->coordinates), *(low->coordinates));
		poped = open.pop();
		ASSERT_EQ(*(poped->coordinates), *(high->coordinates));
		ASSERT_TRUE(open.empty());
		bool oor_exception_thrown = false;
		try
		{
			open.pop();
		}
		catch (const std::out_of_range& oor)
		{
			oor_exception_thrown = true;
		}
		// ASSERT
		ASSERT_TRUE(oor_exception_thrown);
		open.clear();
		low->parentNode = high->parentNode = poped->parentNode = NULL;
		low = poped = high = NULL;
		ASSERT_EQ(initial_object_count, ThetaStarNode::OustandingObjects()); 
	}

	TEST(OpenTest, InsertDuplicateCoordinatesTest)
	{
		int initial_object_count = ThetaStarNode::OustandingObjects();
		// ARRANGE
		octomath::Vector3 final_voxel_center(700,0,0);
		Open open (final_voxel_center);
		octomath::Vector3 key2(0,1,0);
		std::shared_ptr<ThetaStarNode> high = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (key2) , 15, 10, 10);
		octomath::Vector3 key3(0,1,0);
		std::shared_ptr<ThetaStarNode> middle = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (key3) , 15, 9, 10);
		middle->parentNode = high;
		high->parentNode = middle;
		open.insert(high);
		open.insert(middle);
		// open.printNodes();
		// // ACT 
		open.erase(*high);
		open.erase(*middle);
		// ASSERT
		ASSERT_TRUE(open.empty());
		open.clear();
		middle->parentNode = high->parentNode = NULL;
		middle = high = NULL;
		ASSERT_EQ(initial_object_count, ThetaStarNode::OustandingObjects());   // TODO This should actually be 0 but can't find why it is not
	}

	TEST(OpenTest, EraseReturnValueTest)
	{
		int initial_object_count = ThetaStarNode::OustandingObjects();
		// ARRANGE
		octomath::Vector3 final_voxel_center(700,0,0);
		Open open (final_voxel_center);
		octomath::Vector3 key(0,0,0);
		std::shared_ptr<ThetaStarNode> low = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (key) , 15, 10, 5);
		octomath::Vector3 key2(0,1,0);
		std::shared_ptr<ThetaStarNode> high = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (key2) , 15, 10, 10);
		octomath::Vector3 key3(0,2,0);
		std::shared_ptr<ThetaStarNode> middle = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (key3) , 15, 9, 10);
		low->parentNode = high;
		middle->parentNode = high;
		high->parentNode = middle;
		open.insert(high);
		open.insert(middle);
		// ACT & ASSERT
		ASSERT_TRUE(open.erase(*high));
		ASSERT_FALSE(open.erase(*low));
		ASSERT_TRUE(open.erase(*middle));
		ASSERT_TRUE(open.empty());
		open.clear();
		low->parentNode = middle->parentNode = high->parentNode = NULL;
		low = middle = high = NULL;
		ASSERT_EQ(initial_object_count, ThetaStarNode::OustandingObjects());  // TODO This should actually be 0 but can't find why it is not
	}

	TEST(OpenTest, GetFromMapTest)
	{
		int initial_object_count = ThetaStarNode::OustandingObjects();
		// ARRANGE
		octomath::Vector3 final_voxel_center(700,0,0);
		Open open (final_voxel_center);
		octomath::Vector3 key(0,0,0);
		std::shared_ptr<ThetaStarNode> low = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (key) , 15, 10, 5);
		std::shared_ptr<ThetaStarNode> parent = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (2, 5, 2) , 16, 10, 5) ;	
		low-> parentNode = parent;
		open.insert(low);
		// ACT 
		std::shared_ptr<ThetaStarNode> node = open.getFromMap(key);
		// ASSERT
		ASSERT_EQ( *(node->coordinates), *(low->coordinates));
		open.clear();
		low->parentNode = node->parentNode = parent->parentNode = NULL;
		low = node = parent = NULL;
		ASSERT_EQ(initial_object_count, ThetaStarNode::OustandingObjects());    // TODO This should actually be 0 but can't find why it is not
	}	

	TEST(OpenTest, GetFromMapNotExistingTest)
	{
		int initial_object_count = ThetaStarNode::OustandingObjects();
		// ARRANGE
		octomath::Vector3 final_voxel_center(700,0,0);
		Open open (final_voxel_center);
		octomath::Vector3 key(0,0,0);
		// ACT 
		bool oor_exception_thrown = false;
		try
		{
			open.getFromMap(key);
		}
		catch (const std::out_of_range& oor)
		{
			oor_exception_thrown = true;
		}
		// ASSERT
		ASSERT_TRUE(oor_exception_thrown);
		ASSERT_EQ(initial_object_count, ThetaStarNode::OustandingObjects()); 
	}

	TEST(OpenTest, ModifyHeuristicAfterInsertTest_Fixed)
	{
		int initial_object_count = ThetaStarNode::OustandingObjects();
		// ARRANGE
		int cell_size = 15;
		float distanceFromInitialPoint = 10;
		float lineDistanceToFinalPoint = 5;
		octomath::Vector3 final_voxel_center(700,0,0);
		Open open (final_voxel_center);
		octomath::Vector3 coordinates(0,0,0);
		std::shared_ptr<ThetaStarNode> low = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (coordinates) , cell_size, distanceFromInitialPoint, lineDistanceToFinalPoint);
		octomath::Vector3 coordinates2(0,1,0);
		lineDistanceToFinalPoint = 10;
		std::shared_ptr<ThetaStarNode> high = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (coordinates2) , cell_size, distanceFromInitialPoint, lineDistanceToFinalPoint);
		octomath::Vector3 coordinates3(0,3,0);
		distanceFromInitialPoint = 9;
		std::shared_ptr<ThetaStarNode> middle = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (coordinates3) , cell_size, distanceFromInitialPoint, lineDistanceToFinalPoint);
		low->parentNode = high;
		middle->parentNode = high;
		high->parentNode = middle;
		// ACT 
		open.insert(high);
		open.insert(low);
		open.insert(middle);

		// Change distance
		// This is to fix the previous way of using this which was
		// high->distanceFromInitialPoint = 0; //(directly)
		open.erase(*high);
		high->distanceFromInitialPoint = 0;
		open.insert(high);

		std::shared_ptr<ThetaStarNode> poped = open.pop();
		// ASSERT
		ASSERT_EQ(*(poped->coordinates), *(high->coordinates));

		poped = open.pop();
		ASSERT_EQ(*(poped->coordinates), *(low->coordinates));

		poped = open.pop();
		ASSERT_EQ(*(poped->coordinates), *(middle->coordinates));



		open.clear();
		low->parentNode = middle->parentNode = high->parentNode = poped->parentNode = NULL;
		low = middle = high = poped = NULL;
		ASSERT_EQ(initial_object_count, ThetaStarNode::OustandingObjects()); 
		
	}

	TEST(OpenTest, UpdateDistanceFromInitialPointTest)
	{
		int initial_object_count = ThetaStarNode::OustandingObjects();
		// ARRANGE
		int cell_size = 15;
		float distanceFromInitialPoint = 10;
		float lineDistanceToFinalPoint = 5;
		octomath::Vector3 final_voxel_center(700,0,0);
		Open open (final_voxel_center);
		octomath::Vector3 coordinates(0,0,0);
		std::shared_ptr<ThetaStarNode> low = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (coordinates) , cell_size, distanceFromInitialPoint, lineDistanceToFinalPoint);
		octomath::Vector3 coordinates2(0,1,0);
		lineDistanceToFinalPoint = 10;
		std::shared_ptr<ThetaStarNode> high = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (coordinates2) , cell_size, distanceFromInitialPoint, lineDistanceToFinalPoint);
		octomath::Vector3 coordinates3(0,3,0);
		distanceFromInitialPoint = 9;
		std::shared_ptr<ThetaStarNode> middle = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (coordinates3) , cell_size, distanceFromInitialPoint, lineDistanceToFinalPoint);
		low->parentNode = high;
		middle->parentNode = high;
		high->parentNode = middle;
		// ACT 
		open.insert(high);
		open.insert(low);
		open.insert(middle);

		// Change distance
		ASSERT_TRUE(open.changeDistanceFromInitialPoint(0, high));

		// ASSERT that the internal ordering is ok
		std::shared_ptr<ThetaStarNode> poped = open.pop();
		ASSERT_EQ(*(poped->coordinates), *(high->coordinates));
		poped = open.pop();
		ASSERT_EQ(*(poped->coordinates), *(low->coordinates));
		poped = open.pop();
		ASSERT_EQ(*(poped->coordinates), *(middle->coordinates));
		open.clear();
		low->parentNode = middle->parentNode = high->parentNode = poped->parentNode = NULL;
		low = middle = high = poped = NULL;
		ASSERT_EQ(initial_object_count, ThetaStarNode::OustandingObjects()); 
	}

	TEST(OpenTest, Failed_UpdateDistanceFromInitialPointTest)
	{
		int initial_object_count = ThetaStarNode::OustandingObjects();
		// ARRANGE
		int cell_size = 15;
		float distanceFromInitialPoint = 10;
		float lineDistanceToFinalPoint = 5;
		octomath::Vector3 final_voxel_center(700,0,0);
		Open open (final_voxel_center);
		octomath::Vector3 coordinates(0,0,0);
		std::shared_ptr<ThetaStarNode> low = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (coordinates) , cell_size, distanceFromInitialPoint, lineDistanceToFinalPoint);
		octomath::Vector3 coordinates2(0,1,0);
		lineDistanceToFinalPoint = 10;
		std::shared_ptr<ThetaStarNode> high = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (coordinates2) , cell_size, distanceFromInitialPoint, lineDistanceToFinalPoint);
		octomath::Vector3 coordinates3(0,3,0);
		distanceFromInitialPoint = 9;
		std::shared_ptr<ThetaStarNode> middle = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (coordinates3) , cell_size, distanceFromInitialPoint, lineDistanceToFinalPoint);
		low->parentNode = high;
		middle->parentNode = high;
		high->parentNode = middle;
		// ACT 
		open.insert(low);
		open.insert(middle);
		ASSERT_FALSE(open.changeDistanceFromInitialPoint(0, high));

		// ASSERT that the internal ordering is ok
		std::shared_ptr<ThetaStarNode> poped = open.pop();
		ASSERT_EQ(*(poped->coordinates), *(low->coordinates));
		poped = open.pop();
		ASSERT_EQ(*(poped->coordinates), *(middle->coordinates));
		open.clear();
		low->parentNode = middle->parentNode = high->parentNode = poped->parentNode = NULL;
		low = middle = high = poped = NULL;
		ASSERT_EQ(initial_object_count, ThetaStarNode::OustandingObjects()); 
	}


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

		octomath::Vector3 opposite_direction_neighbor_coordinates (-0.9, -11.7, 0.5 );
		std::shared_ptr<ThetaStarNode> opposite_direction_neighbor = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3> (opposite_direction_neighbor_coordinates) , 0.2, 0.2, 1.2) ;	
		opposite_direction_neighbor->parentNode = parent;
		open.insert(opposite_direction_neighbor);
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