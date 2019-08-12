#include <ltStar_lib_ortho.h>
#include <ltStarOctree_common.h>
#include <gtest/gtest.h>
#include <queue>


namespace LazyThetaStarOctree{

	void testStraightLinesForwardNoObstacles(octomap::OcTree & octree, octomath::Vector3 disc_initial, octomath::Vector3 disc_final,
		int const& max_time_secs = 55, double safety_margin = 0.1)
	{
		double sidelength_lookup_table  [octree.getTreeDepth()];
	   	LazyThetaStarOctree::fillLookupTable(octree.getResolution(), octree.getTreeDepth(), sidelength_lookup_table); 
		ros::Publisher marker_pub;
		
		// Initial node is not occupied
		octomap::OcTreeNode* originNode = octree.search(disc_initial);
		ASSERT_TRUE(originNode) << "Start node in unknown space";
		ASSERT_FALSE(octree.isNodeOccupied(originNode));
		// Final node is not occupied
		octomap::OcTreeNode* finalNode = octree.search(disc_final);
		ASSERT_TRUE(finalNode) << "Final node in unknown space";
		ASSERT_FALSE(octree.isNodeOccupied(finalNode));
		// The path is clear from start to finish
		octomath::Vector3 direction (1, 0, 0);
		octomath::Vector3 end;
		bool isOccupied = octree.castRay(disc_initial, direction, end, false, weightedDistance(disc_initial, disc_final));
		ASSERT_FALSE(isOccupied); // false if the maximum range or octree bounds are reached, or if an unknown node was hit.

		ResultSet statistical_data;
		generateOffsets(octree.getResolution(), safety_margin, dephtZero, semiSphereOut );
		InputData input( octree, disc_initial, disc_final, safety_margin);
		std::list<octomath::Vector3> resulting_path = lazyThetaStar_(input, statistical_data, sidelength_lookup_table, rviz_interface::PublishingInput( marker_pub, false), max_time_secs, true);
		// NO PATH
		ASSERT_NE(resulting_path.size(), 0);
		// CANONICAL: straight line, no issues
		// 2 waypoints: The center of start voxel & The center of the goal voxel
		double cell_size_goal = -1;
		octomath::Vector3 cell_center_coordinates_goal = disc_final;
		updateToCellCenterAndFindSize( cell_center_coordinates_goal, octree, cell_size_goal, sidelength_lookup_table);
		ASSERT_LT(      resulting_path.back().distance( cell_center_coordinates_goal ),  cell_size_goal   );
		double cell_size_start = -1;
		octomath::Vector3 cell_center_coordinates_start = disc_initial;
		updateToCellCenterAndFindSize( cell_center_coordinates_start, octree, cell_size_start, sidelength_lookup_table);
		ASSERT_LT(      resulting_path.begin()->distance( cell_center_coordinates_start ),  cell_size_start   );
		// LONG PATHS: 
		if(resulting_path.size() > 2)
		{
			std::list<octomath::Vector3>::iterator it = resulting_path.begin();
			octomath::Vector3 previous = *it;
			++it;
			// Check that there are no redundant parts in the path
			while(it != resulting_path.end())
			{
				ROS_INFO_STREAM("Step: " << *it);
				double diffY = std::abs(previous.y() - it->y());
				double diffZ = std::abs(previous.z() - it->z());
				bool dimensions_y_or_z_change = (diffY > 0.0001) || (diffZ > 0.0001);


				EXPECT_FALSE(dimensions_y_or_z_change) << "(" << previous.y() << " != " << it->y() << ") || (" << previous.z() << " != " << it->z() << ")"  << " this should be a straight line. Find out why parent node is being replaced.";
				previous = *it;
				++it;
			}
		}
		ASSERT_EQ(0, ThetaStarNode::OustandingObjects());
	}

	void testSucessfullQuery(octomap::OcTree octree, octomath::Vector3 disc_initial, octomath::Vector3 disc_final,
		int const& max_time_secs = 55, double safety_margin = 0.1)
	{
		ros::Publisher marker_pub;
		ResultSet statistical_data;
		double sidelength_lookup_table  [octree.getTreeDepth()];
		rviz_interface::PublishingInput publish_input( marker_pub, false);
		InputData input( octree, disc_initial, disc_final, safety_margin);
	   	LazyThetaStarOctree::fillLookupTable( octree.getResolution(), octree.getTreeDepth(), sidelength_lookup_table); 
		generateOffsets(octree.getResolution(), safety_margin, dephtZero, semiSphereOut );
		
		// Initial node is not occupied
		octomap::OcTreeNode* originNode = octree.search(disc_initial);
		ASSERT_TRUE(originNode) << "Start node in unknown space";
		ASSERT_FALSE(octree.isNodeOccupied(originNode));
		// Final node is not occupied
		octomap::OcTreeNode* finalNode = octree.search(disc_final);
		ASSERT_TRUE(finalNode) << "Final node in unknown space";
		ASSERT_FALSE(octree.isNodeOccupied(finalNode));
		// The path is clear from start to finish  
		// ASSERT_FALSE( is_flight_corridor_free	(input, publish_input, false) ) << "This is a test with obstacles but there are none.";

		std::list<octomath::Vector3> resulting_path = lazyThetaStar_(input, statistical_data, sidelength_lookup_table, publish_input, max_time_secs, true);
		// NO PATH
		ASSERT_NE(resulting_path.size(), 0);
		// CANONICAL: straight line, no issues
		// 2 waypoints: The center of start voxel & The center of the goal voxel
		double cell_size_goal = -1;
		octomath::Vector3 cell_center_coordinates_goal = disc_final;
		updateToCellCenterAndFindSize( cell_center_coordinates_goal, octree, cell_size_goal, sidelength_lookup_table);
		ASSERT_LT(      resulting_path.back().distance( cell_center_coordinates_goal ),  cell_size_goal   );
		double cell_size_start = -1;
		octomath::Vector3 cell_center_coordinates_start = disc_initial;
		updateToCellCenterAndFindSize( cell_center_coordinates_start, octree, cell_size_start, sidelength_lookup_table);
		ASSERT_LT(      resulting_path.begin()->distance( cell_center_coordinates_start ),  cell_size_start   );
		ASSERT_EQ(0, ThetaStarNode::OustandingObjects());
	}

	TEST(LazyThetaStarTests, ClosedInsert_Test)
	{
		octomath::Vector3 key_inside(-6.900000095, -2.5, 0.5);
		std::shared_ptr<ThetaStarNode> dummy_node = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3>(key_inside), 0, 0, 0);
		std::unordered_map<octomath::Vector3, std::shared_ptr<ThetaStarNode>, Vector3Hash, VectorComparatorEqual> closed;
		closed.insert( std::pair<octomath::Vector3, std::shared_ptr<ThetaStarNode>>(key_inside, dummy_node));

		ASSERT_EQ(closed.size(), 1);

		bool not_found = closed.find(key_inside) == closed.end();
		ASSERT_FALSE(not_found);

		octomath::Vector3 toFind(-7, -1.599999905, 0.200000003);
		auto search_closed_result= closed.find(toFind);
		ASSERT_EQ(search_closed_result, closed.end()) << search_closed_result->first;
	}

	TEST(LazyThetaStarTests, Map_Insert_Duplicate_Test)
	{
		std::unordered_map<octomath::Vector3, std::shared_ptr<ThetaStarNode>, Vector3Hash, VectorComparatorEqual> closed;

		std::shared_ptr <octomath::Vector3> v1 = std::make_shared <octomath::Vector3>(-6.3000002, -3.0999999, 0.5000001);
		std::shared_ptr<ThetaStarNode> s1 = std::make_shared<ThetaStarNode>(
							v1, 
							0.2, 
							0.1999998, 
							0.8000002 // lineDistanceToFinalPoint
							);
		closed.insert( std::pair<octomath::Vector3, std::shared_ptr<ThetaStarNode>>( *v1, s1));

		std::shared_ptr <octomath::Vector3> v2 = std::make_shared <octomath::Vector3>(-6.3000002, -3.0999999, 0.5000000);
		std::shared_ptr<ThetaStarNode> s2 = std::make_shared<ThetaStarNode>(
							v2, 
							0.2, 
							0.1999998, 
							0.8000002 // lineDistanceToFinalPoint
							);
		closed.insert( std::pair<octomath::Vector3, std::shared_ptr<ThetaStarNode>>( *v2, s2));

		ASSERT_EQ(closed.size(), 1);
	}

	TEST(LazyThetaStarTests, LazyThetaStar_StraighLine_NoSolution_MemoryLeak_Test)
	{
		ros::Publisher marker_pub;
		octomap::OcTree octree ("data/offShoreOil_1m.bt");
		double sidelength_lookup_table  [octree.getTreeDepth()];
	   	LazyThetaStarOctree::fillLookupTable(octree.getResolution(), octree.getTreeDepth(), sidelength_lookup_table); 
		octomath::Vector3 disc_initial(-4.3000001907348633, -4.6999998092651367, 0.5);
		octomath::Vector3 disc_final  (-3.3000001907348633, -4.6999998092651367, 0.5);
		// Initial node is not occupied
		octomap::OcTreeNode* originNode = octree.search(disc_initial);
		ASSERT_TRUE(originNode);
		ASSERT_FALSE(octree.isNodeOccupied(originNode));
		// Final node is not occupied
		octomap::OcTreeNode* finalNode = octree.search(disc_final);
		ASSERT_TRUE(finalNode);
		ASSERT_FALSE(octree.isNodeOccupied(finalNode));
		// The path is clear from start to finish
		octomath::Vector3 direction (1, 0, 0);
		octomath::Vector3 end;
		bool isOccupied = octree.castRay(disc_initial, direction, end, false, weightedDistance(disc_initial, disc_final));
		ASSERT_FALSE(isOccupied); // false if the maximum range or octree bounds are reached, or if an unknown node was hit.
		
        EXPECT_EQ( 0, ThetaStarNode::OustandingObjects()) << "From  " << disc_initial << " to  " << disc_final;

        int count_under10 = 0;
        int count_80 = 0;
        int count_over80 = 0;
        ResultSet statistical_data;
        double safety_margin = 1;
        InputData input( octree, disc_initial, disc_final, safety_margin);
		generateOffsets(octree.getResolution(), safety_margin, dephtZero, semiSphereOut );
        std::list<octomath::Vector3> resulting_path = lazyThetaStar_(input, statistical_data, sidelength_lookup_table, rviz_interface::PublishingInput( marker_pub));
        EXPECT_EQ( 0, ThetaStarNode::OustandingObjects()) << "From  " << disc_initial << " to  " << disc_final;
	}
	TEST(LazyThetaStarTests, LazyThetaStar_NoSolution_NegativeInstanceCount_Test)
	{
		ros::Publisher marker_pub;
		octomap::OcTree octree ("data/offShoreOil_1m.bt");
		double sidelength_lookup_table  [octree.getTreeDepth()];
	   	LazyThetaStarOctree::fillLookupTable(octree.getResolution(), octree.getTreeDepth(), sidelength_lookup_table); 
		octomath::Vector3 disc_initial(-6.9000000953674316, -9.6999998092651367, 0.5);
		octomath::Vector3 disc_final  (-5.9000000953674316, -9.6999998092651367, 0.5);
		ASSERT_EQ( 0, ThetaStarNode::OustandingObjects());
        int count_under10 = 0;
        int count_80 = 0;
        int count_over80 = 0;

        double safety_margin = 1;

        int memory_leak_initial_count = ThetaStarNode::OustandingObjects();
        // Initial node is not occupied
        octomap::OcTreeNode* originNode = octree.search(disc_initial);
        ASSERT_TRUE(originNode);
        ASSERT_FALSE(octree.isNodeOccupied(originNode));
        // Final node is not occupied
        octomap::OcTreeNode* finalNode = octree.search(disc_final);
        ASSERT_TRUE(finalNode);
        ASSERT_FALSE(octree.isNodeOccupied(finalNode));
        // The path is clear from start to finish
        octomath::Vector3 direction (1, 0, 0);
        octomath::Vector3 end;
        bool isOccupied = octree.castRay(disc_initial, direction, end, false, weightedDistance(disc_initial, disc_final));
        ASSERT_FALSE(isOccupied); // false if the maximum range or octree bounds are reached, or if an unknown node was hit.

        // std::cout << "Starting lazy theta from " << disc_initial << " to " << disc_final << std::endl;
        ResultSet statistical_data;
        InputData input( octree, disc_initial, disc_final, safety_margin);
		generateOffsets(octree.getResolution(), safety_margin, dephtZero, semiSphereOut );
        std::list<octomath::Vector3> resulting_path = lazyThetaStar_(input, statistical_data, sidelength_lookup_table, rviz_interface::PublishingInput( marker_pub), 5);


        if(resulting_path.size() == 0)
        {
            ROS_ERROR_STREAM("No solution found from  " << disc_initial << " to  " << disc_final);
        }
        else if(resulting_path.size() == 2)
        {
            // ASSERT_EQ(resulting_path.size(), 2) << "I was looking for path between " << disc_initial << " and " << disc_final << ". Was able to analyze " << count << " lines";
            ASSERT_LT(   ( *(resulting_path.begin()) ).distance( disc_initial ),  octree.getResolution()   );
            ASSERT_LT(   resulting_path.back().distance( disc_final ),  octree.getResolution()   );
            ASSERT_LT(statistical_data.iterations_used, 10);
            count_under10++;
        }
        EXPECT_EQ( memory_leak_initial_count, ThetaStarNode::OustandingObjects()) << "From  " << disc_initial << " to  " << disc_final;


    	// ROS_WARN_STREAM("Overall has " << count_under10 << " under 10; " << count_80 << " around 80 and " << count_over80 << " over 85.");
    }

	TEST(LazyThetaStarTests, LazyThetaStar_offshoreOil_CoreDumped_Test)
	{
		// (-8.3 -8.3 0.5) to (-7.3 -8.3 0.5)
		octomap::OcTree octree ("data/offShoreOil_1m.bt");
		octomath::Vector3 disc_initial(-8.3, -8.3, 0.5);
		octomath::Vector3 disc_final  (-7.3, -8.3, 0.5);
		testStraightLinesForwardNoObstacles(octree, disc_initial, disc_final);
	}

	TEST(LazyThetaStarTests, LazyThetaStar_CoreDumped_Test)
	{
		int max_time_secs= 120;
		double safety_margin = 0.1;
		octomap::OcTree octree ("data/from_-0.49_0.089_1.9_to_-8.5_-12_4.5.bt");
		octomath::Vector3 disc_initial(-0.488766, 0.0890607, 1.91693);
		octomath::Vector3 disc_final  (-8.5, -12.5, 4.5);
		testSucessfullQuery(octree, disc_initial, disc_final, max_time_secs, safety_margin);
	}

	// This is a case where the gap between our precision and octomap precion makes the found cell the next one
	// (-6.099999905 -9.699999809 0.5).distance( (-5.900000095 -9.699999809 0.5) ) = 0.1999998093 > 0.1
	TEST(LazyThetaStarTests, LazyThetaStar_StraighLine_Test)
	{
		// (-6.9 -9.7 0.5) to (-5.9 -9.7 0.5)
		octomap::OcTree octree ("data/offShoreOil_1m.bt");
		octomath::Vector3 disc_initial(-6.9, -9.7, 0.5);
		octomath::Vector3 disc_final  (-5.9, -9.7, 0.5);
		testStraightLinesForwardNoObstacles(octree, disc_initial, disc_final);
	}


	// Latest changes make it with 3 paths ...
	TEST(LazyThetaStarTests, LazyThetaStar_StraighLine_HeuristicValueNoDecimal_Test)
	{
		// (-0.69999998807907104 -11.699999809265137 0.5) and (0.30000001192092896 -11.699999809265137 0.5)
		octomap::OcTree octree ("data/offShoreOil_1m.bt");
		octomath::Vector3 disc_initial(-0.69999998807907104, -11.699999809265137, 0.5);
		octomath::Vector3 disc_final  ( 0.30000001192092896, -11.699999809265137, 0.5);
		testStraightLinesForwardNoObstacles(octree, disc_initial, disc_final);
	}

	// Latest changes make it with no solution ...
	TEST(LazyThetaStarTests, LazyThetaStar_StraighLine_Diagonal_Depth14_Test)
	{
		// (-6.5 -3.0999999046325684 0.5) and (-5.5 -3.0999999046325684 0.5)
		octomap::OcTree octree ("data/offShoreOil_1m.bt");
		octomath::Vector3 disc_initial(-6.5, -3.0999999046325684, 0.5);
		octomath::Vector3 disc_final  (-5.5, -3.0999999046325684, 0.5);
		testSucessfullQuery(octree, disc_initial, disc_final, 100);
	}

	// commenting out because checking voxel status by key lookup indentifies more unknown cells and this is not a straight path anymore
	TEST(LazyThetaStarTests, LazyThetaStar_Long_Test)
	{
		// (-6.900000095 -2.5 0.5) to (-5.900000095 -2.5 0.5)
		octomap::OcTree octree ("data/offShoreOil_1m.bt");
		octomath::Vector3 disc_initial(-6.900000095, -2.5, 0.5);
		octomath::Vector3 disc_final  (-5.900000095, -2.5, 0.5);
		testSucessfullQuery(octree, disc_initial, disc_final);
	}
	
	TEST(LazyThetaStarTests, LazyThetaStar_avoidWall)
	{
		ros::Publisher marker_pub;
		// (0.420435 0.313896 1.92169) to (-2.5 -10.5 3.5)
		octomap::OcTree octree ("data/(-11.2177; -18.2778; 2.39616)_(-8.5; 6.5; 3.5)_throughWall.bt");
		double sidelength_lookup_table  [octree.getTreeDepth()];
	   	LazyThetaStarOctree::fillLookupTable(octree.getResolution(), octree.getTreeDepth(), sidelength_lookup_table); 
		lazy_theta_star_msgs::LTStarRequest request;
		request.header.seq = 2;
		request.request_id = 3;
		request.start.x = -11.2177;
		request.start.y = -18.2778;
		request.start.z = 2.39616;
		request.goal.x = -8.5;
		request.goal.y = 6.5;
		request.goal.z = 3.5;
		request.max_time_secs = 1000;
		request.safety_margin = 1;
		lazy_theta_star_msgs::LTStarReply reply;
		processLTStarRequest(octree, request, reply, sidelength_lookup_table, marker_pub);
		ASSERT_TRUE(reply.success);
		ASSERT_EQ(0, ThetaStarNode::OustandingObjects());
	}

	TEST(LazyThetaStarTests, LazyThetaStar_AddingBadNode)
	{
		ros::Publisher marker_pub;
		// (0.420435 0.313896 1.92169) to (-2.5 -10.5 3.5)
		octomap::OcTree octree ("data/(-10.3054; -18.2637; 2.34813)_(-8.5; 6.5; 3.5)_badNodeAdded.bt");
		double sidelength_lookup_table  [octree.getTreeDepth()];
	   	LazyThetaStarOctree::fillLookupTable(octree.getResolution(), octree.getTreeDepth(), sidelength_lookup_table); 
		lazy_theta_star_msgs::LTStarRequest request;
		request.header.seq = 2;
		request.request_id = 3;
		request.start.x = -10.3054;
		request.start.y = -18.2637;
		request.start.z = 2.34813;
		request.goal.x = -8.5;
		request.goal.y = 6.5;
		request.goal.z = 3.5;
		request.max_time_secs = 500;
		request.safety_margin = 0.5;
		lazy_theta_star_msgs::LTStarReply reply;
		generateOffsets(octree.getResolution(), request.safety_margin, dephtZero, semiSphereOut );
		processLTStarRequest(octree, request, reply, sidelength_lookup_table, marker_pub);
		ASSERT_TRUE(reply.success);
		ASSERT_GT(reply.waypoint_amount, 2);
		ASSERT_EQ(0, ThetaStarNode::OustandingObjects());
	}

	TEST(LazyThetaStarTests, LazyThetaStar_corridorFree_2)
	{
		ros::Publisher marker_pub;
		octomap::OcTree octree ("data/(-10.3054; -18.2637; 2.34813)_(-8.5; 6.5; 3.5)_badNodeAdded.bt");
		octomath::Vector3 start(-11, -15, 3);
		octomath::Vector3 end(-11.5, -13.5, 3.5);
		double safety_margin = 0;
		generateOffsets(octree.getResolution(), safety_margin, dephtZero, semiSphereOut );
		bool start_to_end = is_flight_corridor_free( InputData(octree, start, end, safety_margin), rviz_interface::PublishingInput(marker_pub) );
		bool end_to_start = is_flight_corridor_free( InputData(octree, end, start, safety_margin), rviz_interface::PublishingInput(marker_pub) );
		ASSERT_EQ(start_to_end, end_to_start);
	}

	TEST(LazyThetaStarTests, LazyThetaStar_corridorFree_3)
	{
		ros::Publisher marker_pub;
		octomap::OcTree octree ("data/(-10.3054; -18.2637; 2.34813)_(-8.5; 6.5; 3.5)_badNodeAdded.bt");
		octomath::Vector3 start(-10.5, -13.5, 3.5);
		octomath::Vector3 end(-11.5, -13.5, 3.5);
		double safety_margin = 0;
		generateOffsets(octree.getResolution(), safety_margin, dephtZero, semiSphereOut );
		bool start_to_end = is_flight_corridor_free( InputData(octree, start, end, safety_margin), rviz_interface::PublishingInput(marker_pub) );
		bool end_to_start = is_flight_corridor_free( InputData(octree, end, start, safety_margin), rviz_interface::PublishingInput(marker_pub) );
		ASSERT_EQ(start_to_end, end_to_start);
	}

	TEST(LazyThetaStarTests, LazyThetaStar_corridorFree_4)
	{
		ros::Publisher marker_pub;
		octomap::OcTree octree ("data/(-10.3054; -18.2637; 2.34813)_(-8.5; 6.5; 3.5)_badNodeAdded.bt");
		octomath::Vector3 start(-10.5, -13.5, 3.5);
		octomath::Vector3 end(-10.5, -12.5, 3.5);
		double safety_margin = 0;
		generateOffsets(octree.getResolution(), safety_margin, dephtZero, semiSphereOut );
		bool start_to_end = is_flight_corridor_free( InputData(octree, start, end, safety_margin), rviz_interface::PublishingInput(marker_pub) );
		bool end_to_start = is_flight_corridor_free( InputData(octree, end, start, safety_margin), rviz_interface::PublishingInput(marker_pub) );
		ASSERT_EQ(start_to_end, end_to_start);
	}

	TEST(LazyThetaStarTests, LazyThetaStar_corridorFree_merge_1)
	{
		ros::Publisher marker_pub;
		octomap::OcTree octree ("data/(-10.3054; -18.2637; 2.34813)_(-8.5; 6.5; 3.5)_badNodeAdded.bt");
		octomath::Vector3 start(-11, -15, 3);
		octomath::Vector3 end(-11.5, -13.5, 2.5);
		double safety_margin = 0;

		bool start_to_end = getLineStatusBoundingBox( InputData(octree, start, end, safety_margin ) ) == CellStatus::kFree;
		ROS_ERROR_STREAM("reverse");
		bool end_to_start = getLineStatusBoundingBox( InputData(octree, end, start, safety_margin) ) == CellStatus::kFree;

		ASSERT_EQ(start_to_end, end_to_start);
	}

	TEST(LazyThetaStarTests, LazyThetaStar_corridorFree_merge_2)
	{
		ros::Publisher marker_pub;
		octomap::OcTree octree ("data/(-10.3054; -18.2637; 2.34813)_(-8.5; 6.5; 3.5)_badNodeAdded.bt");
		octomath::Vector3 start(-11, -15, 3);
		octomath::Vector3 end(-11.5, -13.5, 3.5);
		double safety_margin = 0;
		generateOffsets(octree.getResolution(), safety_margin, dephtZero, semiSphereOut );
		bool start_to_end = is_flight_corridor_free( InputData(octree, start, end, safety_margin), rviz_interface::PublishingInput(marker_pub ));
		bool end_to_start = is_flight_corridor_free( InputData(octree, end, start, safety_margin), rviz_interface::PublishingInput(marker_pub ));
		ASSERT_EQ(start_to_end, end_to_start);
	}

	TEST(LazyThetaStarTests, LazyThetaStar_corridorFree_merge_3)
	{
		ros::Publisher marker_pub;
		octomap::OcTree octree ("data/(-10.3054; -18.2637; 2.34813)_(-8.5; 6.5; 3.5)_badNodeAdded.bt");
		octomath::Vector3 start(-10.5, -13.5, 3.5);
		octomath::Vector3 end(-11.5, -13.5, 3.5);
		double safety_margin = 0;
		generateOffsets(octree.getResolution(), safety_margin, dephtZero, semiSphereOut );
		bool start_to_end = is_flight_corridor_free( InputData(octree, start, end, safety_margin), rviz_interface::PublishingInput(marker_pub) );
		bool end_to_start = is_flight_corridor_free( InputData(octree, end, start, safety_margin), rviz_interface::PublishingInput(marker_pub) );
		ASSERT_EQ(start_to_end, end_to_start);
	}

	TEST(LazyThetaStarTests, LazyThetaStar_corridorFree_merge_4)
	{
		ros::Publisher marker_pub;
		octomap::OcTree octree ("data/(-10.3054; -18.2637; 2.34813)_(-8.5; 6.5; 3.5)_badNodeAdded.bt");
		octomath::Vector3 start(-10.5, -13.5, 3.5);
		octomath::Vector3 end(-10.5, -12.5, 3.5);
		double safety_margin = 0;
		generateOffsets(octree.getResolution(), safety_margin, dephtZero, semiSphereOut );
		bool start_to_end = is_flight_corridor_free( InputData(octree, start, end, safety_margin), rviz_interface::PublishingInput(marker_pub) );
		bool end_to_start = is_flight_corridor_free( InputData(octree, end, start, safety_margin), rviz_interface::PublishingInput(marker_pub) );
		ASSERT_EQ(start_to_end, end_to_start);
	}

}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}