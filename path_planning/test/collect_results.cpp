#include <gtest/gtest.h>
#include <ltStar_temp.h>

namespace LazyThetaStarOctree
{
    void testResults(path_planning_msgs::LTStarRequest request, octomap::OcTree & octree, std::string dataset_name)
    {
		ros::Publisher marker_pub;
		path_planning_msgs::LTStarReply reply;
		double sidelength_lookup_table  [octree.getTreeDepth()];
	   	LazyThetaStarOctree::fillLookupTable(octree.getResolution(), octree.getTreeDepth(), sidelength_lookup_table); 

		bool success = true;
		processLTStarRequest(octree, request, reply, sidelength_lookup_table, PublishingInput(marker_pub, true, dataset_name)  );
		int waypoint_amount = reply.waypoint_amount;
		ASSERT_EQ(reply.success, success);
		for (int i = 0; i < 1; ++i)
		{
			processLTStarRequest(octree, request, reply, sidelength_lookup_table, PublishingInput(marker_pub, true, dataset_name)  );
			ASSERT_EQ(reply.success, success);
			EXPECT_EQ(reply.waypoint_amount, waypoint_amount);
			EXPECT_EQ(0, ThetaStarNode::OustandingObjects());
		}
    }

 //    TEST(LazyThetaStarTests, LazyThetaStar_20180821_1110_42712_raw_1)
	// {
	// 	octomap::OcTree octree ("data/20180821_1110_42712_raw.bt");
	// 	path_planning_msgs::LTStarRequest request;
	// 	request.header.seq = 2;
	// 	request.request_id = 3;
	// 	request.start.x = -3.9;
	// 	request.start.y = -14.76;
	// 	request.start.z = 7;
	// 	request.goal.x = 7.49;
	// 	request.goal.y = -6.98;
	// 	request.goal.z = 7;
	// 	request.max_search_iterations = 120;
	// 	request.safety_margin = 5;
	// 	testResults(request, octree, "20180821_1110_42712_raw" );
	// }
 //    TEST(LazyThetaStarTests, LazyThetaStar_20180821_1110_42712_raw_2)
	// {
	// 	octomap::OcTree octree ("data/20180821_1110_42712_raw.bt");
	// 	path_planning_msgs::LTStarRequest request;
	// 	request.header.seq = 2;
	// 	request.request_id = 3;
	// 	request.start.x = 7.49;
	// 	request.start.y = -6.98;
	// 	request.start.z = 7;
	// 	request.goal.x = -3.9;
	// 	request.goal.y = -14.76;
	// 	request.goal.z = 7;
	// 	request.max_search_iterations = 120;
	// 	request.safety_margin = 5;
	// 	testResults(request, octree, "20180821_1110_42712_raw" );
	// }

    TEST(LazyThetaStarTests, LazyThetaStar_20180821_1110_42712_filtered_1)
	{
		octomap::OcTree octree ("data/20180821_1110_42712_filtered.bt");
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
		testResults(request, octree, "20180821_1110_42712_filtered" );
	}
    TEST(LazyThetaStarTests, LazyThetaStar_20180821_1110_42712_filtered_2)
	{
		octomap::OcTree octree ("data/20180821_1110_42712_filtered.bt");
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
		testResults(request, octree, "20180821_1110_42712_filtered" );
	}


 //    TEST(LazyThetaStarTests, LazyThetaStar_20180821_1110_42936_raw_1)
	// {
	// 	octomap::OcTree octree ("data/20180821_1110_42936_raw.bt");
	// 	path_planning_msgs::LTStarRequest request;
	// 	request.header.seq = 2;
	// 	request.request_id = 3;
	// 	request.start.x = -3.9;
	// 	request.start.y = -14.76;
	// 	request.start.z = 7;
	// 	request.goal.x = 7.49;
	// 	request.goal.y = -6.98;
	// 	request.goal.z = 7;
	// 	request.max_search_iterations = 120;
	// 	request.safety_margin = 5;
	// 	testResults(request, octree, "20180821_1110_42936_raw" );
	// }
 //    TEST(LazyThetaStarTests, LazyThetaStar_20180821_1110_42936_raw_2)
	// {
	// 	octomap::OcTree octree ("data/20180821_1110_42936_raw.bt");
	// 	path_planning_msgs::LTStarRequest request;
	// 	request.header.seq = 2;
	// 	request.request_id = 3;
	// 	request.start.x = 7.49;
	// 	request.start.y = -6.98;
	// 	request.start.z = 7;
	// 	request.goal.x = -3.9;
	// 	request.goal.y = -14.76;
	// 	request.goal.z = 7;
	// 	request.max_search_iterations = 120;
	// 	request.safety_margin = 5;
	// 	testResults(request, octree, "20180821_1110_42936_raw" );
	// }

 //    TEST(LazyThetaStarTests, LazyThetaStar_20180821_1110_42936_filtered_1)
	// {
	// 	octomap::OcTree octree ("data/20180821_1110_42936_filtered.bt");
	// 	path_planning_msgs::LTStarRequest request;
	// 	request.header.seq = 2;
	// 	request.request_id = 3;
	// 	request.start.x = -3.9;
	// 	request.start.y = -14.76;
	// 	request.start.z = 7;
	// 	request.goal.x = 7.49;
	// 	request.goal.y = -6.98;
	// 	request.goal.z = 7;
	// 	request.max_search_iterations = 120;
	// 	request.safety_margin = 5;
	// 	testResults(request, octree, "20180821_1110_42936_filtered" );
	// }
 //    TEST(LazyThetaStarTests, LazyThetaStar_20180821_1110_42936_filtered_2)
	// {
	// 	octomap::OcTree octree ("data/20180821_1110_42936_filtered.bt");
	// 	path_planning_msgs::LTStarRequest request;
	// 	request.header.seq = 2;
	// 	request.request_id = 3;
	// 	request.start.x = 7.49;
	// 	request.start.y = -6.98;
	// 	request.start.z = 7;
	// 	request.goal.x = -3.9;
	// 	request.goal.y = -14.76;
	// 	request.goal.z = 7;
	// 	request.max_search_iterations = 120;
	// 	request.safety_margin = 5;
	// 	testResults(request, octree, "20180821_1110_42936_filtered" );
	// }




 //    TEST(LazyThetaStarTests, LazyThetaStar_20180821_1110_43042_raw_1)
	// {
	// 	octomap::OcTree octree ("data/20180821_1110_43042_raw.bt");
	// 	path_planning_msgs::LTStarRequest request;
	// 	request.header.seq = 2;
	// 	request.request_id = 3;
	// 	request.start.x = -3.9;
	// 	request.start.y = -14.76;
	// 	request.start.z = 7;
	// 	request.goal.x = 7.49;
	// 	request.goal.y = -6.98;
	// 	request.goal.z = 7;
	// 	request.max_search_iterations = 120;
	// 	request.safety_margin = 5;
	// 	testResults(request, octree, "20180821_1110_43042_raw" );
	// }
 //    TEST(LazyThetaStarTests, LazyThetaStar_20180821_1110_43042_raw_2)
	// {
	// 	octomap::OcTree octree ("data/20180821_1110_43042_raw.bt");
	// 	path_planning_msgs::LTStarRequest request;
	// 	request.header.seq = 2;
	// 	request.request_id = 3;
	// 	request.start.x = 7.49;
	// 	request.start.y = -6.98;
	// 	request.start.z = 7;
	// 	request.goal.x = -3.9;
	// 	request.goal.y = -14.76;
	// 	request.goal.z = 7;
	// 	request.max_search_iterations = 120;
	// 	request.safety_margin = 5;
	// 	testResults(request, octree, "20180821_1110_43042_raw" );
	// }

 //    TEST(LazyThetaStarTests, LazyThetaStar_20180821_1110_43042_filtered_1)
	// {
	// 	octomap::OcTree octree ("data/20180821_1110_43042_filtered.bt");
	// 	path_planning_msgs::LTStarRequest request;
	// 	request.header.seq = 2;
	// 	request.request_id = 3;
	// 	request.start.x = -3.9;
	// 	request.start.y = -14.76;
	// 	request.start.z = 7;
	// 	request.goal.x = 7.49;
	// 	request.goal.y = -6.98;
	// 	request.goal.z = 7;
	// 	request.max_search_iterations = 120;
	// 	request.safety_margin = 5;
	// 	testResults(request, octree, "20180821_1110_43042_filtered");
	// }
 //    TEST(LazyThetaStarTests, LazyThetaStar_20180821_1110_43042_filtered_2)
	// {
	// 	octomap::OcTree octree ("data/20180821_1110_43042_filtered.bt");
	// 	path_planning_msgs::LTStarRequest request;
	// 	request.header.seq = 2;
	// 	request.request_id = 3;
	// 	request.start.x = 7.49;
	// 	request.start.y = -6.98;
	// 	request.start.z = 7;
	// 	request.goal.x = -3.9;
	// 	request.goal.y = -14.76;
	// 	request.goal.z = 7;
	// 	request.max_search_iterations = 120;
	// 	request.safety_margin = 5;
	// 	testResults(request, octree, "20180821_1110_43042_filtered");
	// }

}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}