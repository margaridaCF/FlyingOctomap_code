#include <ltStar_lib_ortho.h>
#include <gtest/gtest.h>
#include <queue>


namespace LazyThetaStarOctree{

	
	void testStraightLinesForwardWithObstacles(octomap::OcTree octree, octomath::Vector3 disc_initial, octomath::Vector3 disc_final,
		int const& max_time_secs = 55, double safety_margin = 2)
	{
		ros::Publisher marker_pub;
		ResultSet statistical_data;
		double sidelength_lookup_table  [octree.getTreeDepth()];
		PublishingInput publish_input( marker_pub, true);
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
		ASSERT_FALSE( is_flight_corridor_free	(input, publish_input, false) ) << "This is a test with obstacles but there are none.";
		
		std::list<octomath::Vector3> resulting_path = lazyThetaStar_(input, statistical_data, sidelength_lookup_table, publish_input, max_time_secs, true);

		path_planning_msgs::LTStarRequest request;
        request.request_id = 0;
        request.header.frame_id = "world";
        request.start.x = disc_initial.x();
        request.start.y = disc_initial.y();
        request.start.z = disc_initial.z();
        request.goal.x = disc_final.x();
        request.goal.y = disc_final.y();
        request.goal.z = disc_final.z();
        request.max_time_secs = max_time_secs;
        request.safety_margin = safety_margin;
        path_planning_msgs::LTStarReply reply;

		bool success = processLTStarRequest(octree, request, reply, sidelength_lookup_table, publish_input);
		// NO PATH
		EXPECT_TRUE(success);
		if(success)
		{
			// CANONICAL: straight line, no issues
			// 2 waypoints: The center of start voxel & The center of the goal voxel
			octomath::Vector3 result_start (reply.waypoints[0].position.x, reply.waypoints[0].position.y, reply.waypoints[0].position.z);
			int last_waypoint_index = reply.waypoint_amount - 1;
			octomath::Vector3 result_goal (reply.waypoints[last_waypoint_index].position.x, reply.waypoints[last_waypoint_index].position.y, reply.waypoints[last_waypoint_index].position.z);

			double cell_size_goal = -1;
			octomath::Vector3 cell_center_coordinates_goal = disc_final;
			updateToCellCenterAndFindSize( cell_center_coordinates_goal, octree, cell_size_goal, sidelength_lookup_table);
			ASSERT_LT(      result_goal.distance( cell_center_coordinates_goal ),  cell_size_goal   );
			double cell_size_start = -1;
			octomath::Vector3 cell_center_coordinates_start = disc_initial;
			updateToCellCenterAndFindSize( cell_center_coordinates_start, octree, cell_size_start, sidelength_lookup_table);
			ASSERT_LT(      result_start.distance( cell_center_coordinates_start ),  cell_size_start   );
		}
		ASSERT_EQ(0, ThetaStarNode::OustandingObjects());
	}

	TEST(LazyThetaStarMeasurements, AllCombinations)
	{
		
	  		std::array<octomath::Vector3, 51> points = {
		  		/*A = */octomath::Vector3 (0.28, 13.3, 90),
		  		/*B = */octomath::Vector3 (-21.6, 20.3, 63),
		  		// /*C = */octomath::Vector3 (-34.9, 20.2, 32.0),
		  		// /*D = */octomath::Vector3 (-20.4, 12.4, 11.5),
		  		// /*E = */octomath::Vector3 (41.4, 22.4, 32),
		  		// /*F = */octomath::Vector3 (21.6, 23.5, 12.5),
		  // 		octomath::Vector3 (0, 0, 50),
				// octomath::Vector3 (0, -20, 30),
				// octomath::Vector3 (-70, -20, 30),
				// octomath::Vector3 (-70, 10, 90),
				// octomath::Vector3 (50, 10, 90),
				// octomath::Vector3 (50, 10, 65),
				// octomath::Vector3 (-70, 10, 65),
				// octomath::Vector3 (-35, 10, 50),
				// octomath::Vector3 (-35, 10, 10),
				// octomath::Vector3 (-70, 10, 10),
				// octomath::Vector3 (-70, 20, 30),
				// octomath::Vector3 (20, 20, 30),
				// octomath::Vector3 (-20, 20, 50),
				// octomath::Vector3 (-20, 20, 65) ,
				// octomath::Vector3 (70, 20, 65),
				// octomath::Vector3 (30, 20, 65),
				// octomath::Vector3 (0, 13, 60)  ,
				// octomath::Vector3 (0, 30, 100),
				// octomath::Vector3 (0, 30, 80),
				// octomath::Vector3 (0, 30, 100),
				// octomath::Vector3 (0, 0, 100),
				// octomath::Vector3 (0, 0, 75),
				// octomath::Vector3 (10, 0, 75),
				// octomath::Vector3 (10, 15, 75),
				// octomath::Vector3 (14, 15, 75),
				// octomath::Vector3 (14, 15, 20),
				// octomath::Vector3 (1, 15, 20),
				// octomath::Vector3 (1, 15, 10),
				// octomath::Vector3 (19.6, 29.7, 10),
				// octomath::Vector3 (19.6, 29.7,100),
				// octomath::Vector3 (19.6, 29.7, 30),
				// octomath::Vector3 (24.4, 16.8, 32),
				// octomath::Vector3 (32.5, 18.5, 32),
				// octomath::Vector3 (51.5, 33.5, 32),
				// octomath::Vector3 (51.5, 33.5, 12),
				// octomath::Vector3 (40.2, 26.8, 12),
				// octomath::Vector3 (40.2, 50, 12),
				// octomath::Vector3 (-30, 50, 12),
				// octomath::Vector3 (0, 50, 12),
				// octomath::Vector3 (0, 30, 25),
				// octomath::Vector3 (0, -10, 25),
				// octomath::Vector3 (0, -10, 10),
				// octomath::Vector3 (0, -30, 10),
				// octomath::Vector3 (-20, -30, 10),
				// octomath::Vector3 (-20, 0, 10)
			};

	    octomap::OcTree octree ("data/3Dpuzzle_sparse_complete.bt");
	    std::string dataset_name = "3Dpuzzle";
	    double max_time_secs = 60;
	    double safety_margin = 3;

	    // for (int i = 0; i < points.size(); ++i)
	    // {
		   //  for (int j = 0; j < points.size(); ++j)
		   //  {
	    		testStraightLinesForwardWithObstacles(octree, points[1], points[0], max_time_secs, safety_margin);
		    	
		   //  }
	    // }

	}

}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
