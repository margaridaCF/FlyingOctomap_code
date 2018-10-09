#include <ltStar_lib_ortho.h>
#include <gtest/gtest.h>
#include <queue>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <visualization_msgs/Marker.h>


#include <chrono>

namespace LazyThetaStarOctree
{
	void testStraightLinesForwardWithObstacles(octomap::OcTree octree, octomath::Vector3 disc_initial, octomath::Vector3 disc_final,
		int const& max_time_secs = 55, double safety_margin = 2)
	{
		ros::Publisher marker_pub;
		ResultSet statistical_data;
		double sidelength_lookup_table  [octree.getTreeDepth()];
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
		ASSERT_FALSE( hasLineOfSight(input) ) << "This is a test with obstacles but there are none.";

		std::list<octomath::Vector3> resulting_path = lazyThetaStar_(input, statistical_data, sidelength_lookup_table, PublishingInput( marker_pub, true), max_time_secs, true);
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
				bool dimensions_y_or_z_change = (previous.y() != it->y()) || (previous.z() != it->z());
				EXPECT_TRUE(dimensions_y_or_z_change) << "(" << previous.y() << " != " << it->y() << ") || (" << previous.z() << " != " << it->z() << ")";
				EXPECT_TRUE(dimensions_y_or_z_change) << " this should be a straight line. Find out why parent node is being replaced.";
				previous = *it;
				++it;
			}
		}
		ASSERT_EQ(0, ThetaStarNode::OustandingObjects());
	}

	// TEST(LazyThetaStarTests, ObstaclePath_10m_Test_16)
	// {
		
	//     octomath::Vector3 disc_initial(0, 5, 1.5); 
	//     octomath::Vector3 disc_final  (2, -5, 1.5); 
	//     octomap::OcTree octree ("data/run_2.bt");
	//     std::string dataset_name = "run 2";
	//     testStraightLinesForwardWithObstacles(octree, disc_initial, disc_final, 1000, 1.6);
	// }

	TEST(LazyThetaStarTests, ObstaclePath_BadNode_2)
	{
		// s node (-16.5 27.5 30.5) has no line of sight with the current parent (-22 10 6)(path 2)  and node of the neighbors are visible either (path 1). 
		// (-17 27 29) path 1
	    octomath::Vector3 disc_initial(-21.7403, 8.64695, 4.5147); 
	    octomath::Vector3 disc_final  (-0.5, 13.5, 10.5); 
	    octomap::OcTree octree ("data/from_-22_8.6_4.5_to_-0.5_14_10_noPath1.bt");
	    std::string dataset_name = "run 2";
	    testStraightLinesForwardWithObstacles(octree, disc_initial, disc_final, 120, 3);
	}


	// TEST(LazyThetaStarTests, LazyThetaStar_AddingBadNode)
	// {
	// 	ros::Publisher marker_pub;
	// 	// (0.420435 0.313896 1.92169) to (-2.5 -10.5 3.5)
	// 	octomap::OcTree octree ("data/(-10.3054; -18.2637; 2.34813)_(-8.5; 6.5; 3.5)_badNodeAdded.bt");
	// 	double sidelength_lookup_table  [octree.getTreeDepth()];
	//    	LazyThetaStarOctree::fillLookupTable(octree.getResolution(), octree.getTreeDepth(), sidelength_lookup_table); 
	// 	path_planning_msgs::LTStarRequest request;
	// 	request.header.seq = 2;
	// 	request.request_id = 3;
	// 	request.start.x = -10.3054;
	// 	request.start.y = -18.2637;
	// 	request.start.z = 2.34813;
	// 	request.goal.x = -8.5;
	// 	request.goal.y = 6.5;
	// 	request.goal.z = 3.5;
	// 	request.max_time_secs = 500;
	// 	request.safety_margin = 0.5;
	// 	path_planning_msgs::LTStarReply reply;
	// 	generateOffsets(octree.getResolution(), request.safety_margin, dephtZero, semiSphereOut );
	// 	processLTStarRequest(octree, request, reply, sidelength_lookup_table, marker_pub);
	// 	ASSERT_TRUE(reply.success);
	// 	ASSERT_GT(reply.waypoint_amount, 2);
	// 	ASSERT_EQ(0, ThetaStarNode::OustandingObjects());
	// }
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}