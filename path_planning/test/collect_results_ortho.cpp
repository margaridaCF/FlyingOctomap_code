#include <gtest/gtest.h>
#include <ltStar_lib_ortho.h>

namespace LazyThetaStarOctree
{	
    bool equal (const geometry_msgs::Point & a, const geometry_msgs::Point & b, 
		const double theta = 0.00000000000000000001)
 	{
 
		bool is_x_equal = std::abs(a.x - b.x) < theta;
		bool is_y_equal = std::abs(a.y - b.y) < theta;
		bool is_z_equal = std::abs(a.z - b.z) < theta;
 
		return is_x_equal && is_y_equal && is_z_equal;
 	}

    void testResults(path_planning_msgs::LTStarRequest request, octomap::OcTree & octree, std::string dataset_name)
    {
    	ROS_WARN_STREAM("Here I am.");

		ros::Publisher marker_pub;
		path_planning_msgs::LTStarReply reply;
		double sidelength_lookup_table  [octree.getTreeDepth()];
	   	LazyThetaStarOctree::fillLookupTable(octree.getResolution(), octree.getTreeDepth(), sidelength_lookup_table); 

		LazyThetaStarOctree::generateOffsets(octree.getResolution(), request.safety_margin, dephtZero, semiSphereOut );

	   
		bool is_start_clear = LazyThetaStarOctree::is_flight_corridor_free(LazyThetaStarOctree::InputData(octree, octomath::Vector3(request.start.x, request.start.y, request.start.z), octomath::Vector3(request.start.x, request.start.y, request.start.z+0.6), request.safety_margin), PublishingInput(marker_pub, false));
	   	ASSERT_TRUE(is_start_clear);


		bool is_goal_clear = LazyThetaStarOctree::is_flight_corridor_free(LazyThetaStarOctree::InputData(octree, octomath::Vector3(request.goal.x, request.goal.y, request.goal.z), octomath::Vector3(request.goal.x, request.goal.y, request.goal.z+0.6), request.safety_margin), PublishingInput(marker_pub, false));
	   	ASSERT_TRUE(is_goal_clear);

		bool success = true;
		processLTStarRequest(octree, request, reply, sidelength_lookup_table, PublishingInput(marker_pub, true, dataset_name)  );
		int waypoint_amount = reply.waypoint_amount;
		auto waypoints = reply.waypoints;
		ASSERT_EQ(reply.success, success);
		for (int i = 0; i < 1; ++i)
		{
			processLTStarRequest(octree, request, reply, sidelength_lookup_table, PublishingInput(marker_pub, true, dataset_name)  );
			ASSERT_EQ(reply.success, success);
			EXPECT_EQ(reply.waypoint_amount, waypoint_amount);
			EXPECT_EQ(0, ThetaStarNode::OustandingObjects());
			for (int i = 0; i < waypoint_amount; ++i)
			{
				EXPECT_TRUE( equal(waypoints[i].position, reply.waypoints[i].position) );
			}
		}
    }

    TEST(LazyThetaStarTests, LazyThetaStar_20180821_1110_42712_raw_1)
	{
		octomap::OcTree octree ("data/20180821_1110_42712_raw.bt");
		path_planning_msgs::LTStarRequest request;
		request.header.seq = 2;
		request.request_id = 3;
		request.start.x = -3.9;
		request.start.y = -14.76;
		request.start.z = 7;
		request.goal.x = 7.49;
		request.goal.y = -6.98;
		request.goal.z = 7;
		request.max_search_iterations = 120;
		request.safety_margin = 5;
		testResults(request, octree, "20180821_1110_42712_raw_ortho" );
	}
    TEST(LazyThetaStarTests, LazyThetaStar_20180821_1110_42712_raw_2)
	{
		octomap::OcTree octree ("data/20180821_1110_42712_raw.bt");
		path_planning_msgs::LTStarRequest request;
		request.header.seq = 2;
		request.request_id = 3;
		request.start.x = 7.49;
		request.start.y = -6.98;
		request.start.z = 7;
		request.goal.x = -3.9;
		request.goal.y = -14.76;
		request.goal.z = 7;
		request.max_search_iterations = 120;
		request.safety_margin = 5;
		testResults(request, octree, "20180821_1110_42712_raw_ortho" );
	}

    TEST(LazyThetaStarTests, LazyThetaStar_20180821_1110_42712_raw_3)
	{
		octomap::OcTree octree ("data/20180821_1110_42712_raw.bt");
		path_planning_msgs::LTStarRequest request;
		request.header.seq = 2;
		request.request_id = 3;
		request.start.x = -3.9;
		request.start.y = -14.76;
		request.start.z = 7;
		request.goal.x = 7.49;
		request.goal.y = -6.98;
		request.goal.z = 3;
		request.max_search_iterations = 120;
		request.safety_margin = 5;
		testResults(request, octree, "20180821_1110_42712_raw_ortho" );
	}

    TEST(LazyThetaStarTests, LazyThetaStar_20180821_1110_42712_raw_4)
	{
		octomap::OcTree octree ("data/20180821_1110_42712_raw.bt");
		path_planning_msgs::LTStarRequest request;
		request.header.seq = 2;
		request.request_id = 3;
		request.start.x = 7.49;
		request.start.y = -6.98;
		request.start.z = 3;
		request.goal.x = -3.9;
		request.goal.y = -14.76;
		request.goal.z = 7;
		request.max_search_iterations = 120;
		request.safety_margin = 5;
		testResults(request, octree, "20180821_1110_42712_raw_ortho" );
	}


    TEST(LazyThetaStarTests, LazyThetaStar_20180821_1110_42712_raw_far)
	{
		octomap::OcTree octree ("data/20180821_1110_42712_raw.bt");
		path_planning_msgs::LTStarRequest request;
		request.header.seq = 2;
		request.request_id = 3;
		request.start.x =  -15.7;
		request.start.y = -5.08;
		request.start.z = 5;
		request.goal.x = 13.9;
		request.goal.y = -18.2;
		request.goal.z = 3;
		request.max_search_iterations = 120;
		request.safety_margin = 5;
		testResults(request, octree, "20180821_1110_42712_raw_far" );
	}

    TEST(LazyThetaStarTests, LazyThetaStar_20180821_1110_42712_raw_far_back)
	{
		octomap::OcTree octree ("data/20180821_1110_42712_raw.bt");
		path_planning_msgs::LTStarRequest request;
		request.header.seq = 2;
		request.request_id = 3;
		request.goal.x =  -15.7;
		request.goal.y = -5.08;
		request.goal.z = 5;
		request.start.x = 13.9;
		request.start.y = -18.2;
		request.start.z = 3;
		request.max_search_iterations = 120;
		request.safety_margin = 5;
		testResults(request, octree, "20180821_1110_42712_raw_far_back_ortho" );
	}


    TEST(LazyThetaStarTests, LazyThetaStar_20180821_1110_42712_raw_far_straight)
	{
		octomap::OcTree octree ("data/20180821_1110_42712_raw.bt");
		path_planning_msgs::LTStarRequest request;
		request.header.seq = 2;
		request.request_id = 3;
		request.start.x =  16.8;
		request.start.y = -4.22;
		request.start.z = 5;
		request.goal.x = -23.5;
		request.goal.y = -28.4;
		request.goal.z = 3;
		request.max_search_iterations = 120;
		request.safety_margin = 5;
		testResults(request, octree, "20180821_1110_42712_raw_straight_far_ortho" );
	}

    TEST(LazyThetaStarTests, LazyThetaStar_20180821_1110_42712_raw_far_straight_back)
	{
		octomap::OcTree octree ("data/20180821_1110_42712_raw.bt");
		path_planning_msgs::LTStarRequest request;
		request.header.seq = 2;
		request.request_id = 3;
		request.goal.x =  16.8;
		request.goal.y = -4.22;
		request.goal.z = 5;
		request.start.x = -23.5;
		request.start.y = -28.4;
		request.start.z = 3;
		request.max_search_iterations = 120;
		request.safety_margin = 5;
		testResults(request, octree, "20180821_1110_42712_raw_far_straight_back_ortho" );
	}

    TEST(LazyThetaStarTests, LazyThetaStar_20180821_1110_42936_raw_1)
	{
		octomap::OcTree octree ("data/20180821_1110_42936_raw.bt");
		path_planning_msgs::LTStarRequest request;
		request.header.seq = 2;
		request.request_id = 3;
		request.start.x = -3.9;
		request.start.y = -14.76;
		request.start.z = 7;
		request.goal.x = 7.49;
		request.goal.y = -6.98;
		request.goal.z = 7;
		request.max_search_iterations = 120;
		request.safety_margin = 5;
		testResults(request, octree, "20180821_1110_42936_raw_ortho" );
	}
    TEST(LazyThetaStarTests, LazyThetaStar_20180821_1110_42936_raw_2)
	{
		octomap::OcTree octree ("data/20180821_1110_42936_raw.bt");
		path_planning_msgs::LTStarRequest request;
		request.header.seq = 2;
		request.request_id = 3;
		request.start.x = 7.49;
		request.start.y = -6.98;
		request.start.z = 7;
		request.goal.x = -3.9;
		request.goal.y = -14.76;
		request.goal.z = 7;
		request.max_search_iterations = 120;
		request.safety_margin = 5;
		testResults(request, octree, "20180821_1110_42936_raw_ortho" );
	}



    TEST(LazyThetaStarTests, LazyThetaStar_20180821_1110_43042_raw_1)
	{
		octomap::OcTree octree ("data/20180821_1110_43042_raw.bt");
		path_planning_msgs::LTStarRequest request;
		request.header.seq = 2;
		request.request_id = 3;
		request.start.x = -3.9;
		request.start.y = -14.76;
		request.start.z = 7;
		request.goal.x = 7.49;
		request.goal.y = -6.98;
		request.goal.z = 7;
		request.max_search_iterations = 120;
		request.safety_margin = 5;
		testResults(request, octree, "20180821_1110_43042_raw_ortho" );
	}
    TEST(LazyThetaStarTests, LazyThetaStar_20180821_1110_43042_raw_2)
	{
		octomap::OcTree octree ("data/20180821_1110_43042_raw.bt");
		path_planning_msgs::LTStarRequest request;
		request.header.seq = 2;
		request.request_id = 3;
		request.start.x = 7.49;
		request.start.y = -6.98;
		request.start.z = 7;
		request.goal.x = -3.9;
		request.goal.y = -14.76;
		request.goal.z = 7;
		request.max_search_iterations = 120;
		request.safety_margin = 5;
		testResults(request, octree, "20180821_1110_43042_raw_ortho" );
	}


    TEST(LazyThetaStarTests, LazyThetaStar_20180821_1207_5647_filtered)
	{
		octomap::OcTree octree ("data/20180821_1207_5647_filtered.bt");
		path_planning_msgs::LTStarRequest request;
		request.header.seq = 2;
		request.request_id = 3;
		request.start.x = -5.35;
		request.start.y = -14;
		request.start.z = 7;
		request.goal.x = 4.12;
		request.goal.y = -2.21;
		request.goal.z = 7;
		request.max_search_iterations = 120;
		request.safety_margin = 5;
		testResults(request, octree, "20180821_1207_5647_filtered_approxGoal_ortho" );
	}

    TEST(LazyThetaStarTests, LazyThetaStar_20180821_1207_5647_filtered_back)
	{
		octomap::OcTree octree ("data/20180821_1207_5647_filtered.bt");
		path_planning_msgs::LTStarRequest request;
		request.header.seq = 2;
		request.request_id = 3;
		request.goal.x = -5.35;
		request.goal.y = -14;
		request.goal.z = 7;
		request.start.x = 4.12;
		request.start.y = -2.21;
		request.start.z = 7;
		request.max_search_iterations = 120;
		request.safety_margin = 5;
		testResults(request, octree, "20180821_1207_5647_filtered_approxGoal_back_ortho" );
	}
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}