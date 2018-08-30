#include <gtest/gtest.h>
#include <ltStar_temp.h>

namespace LazyThetaStarOctree
{
	timespec diff(timespec start, timespec end)
    {
        timespec temp;
        if ((end.tv_nsec-start.tv_nsec)<0) {
            temp.tv_sec = end.tv_sec-start.tv_sec-1;
            // temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
        } else {
            temp.tv_sec = end.tv_sec-start.tv_sec;
            temp.tv_nsec = end.tv_nsec-start.tv_nsec;
        }
        return temp;
    }

    TEST(LazyThetaStarTests, LazyThetaStar_20180823_1110)
	{
		ros::Publisher marker_pub;
		// (0.420435 0.313896 1.92169) to (-2.5 -10.5 3.5)
		octomap::OcTree octree ("data/20180823_1110_manyRuns_karting.bt");
		double sidelength_lookup_table  [octree.getTreeDepth()];
	   	LazyThetaStarOctree::fillLookupTable(octree.getResolution(), octree.getTreeDepth(), sidelength_lookup_table); 
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
		path_planning_msgs::LTStarReply reply;

		for (int i = 0; i < 10; ++i)
		{
			processLTStarRequest(octree, request, reply, sidelength_lookup_table, PublishingInput(marker_pub, true, "20180823_1110_manyRuns_karting")  );
			ASSERT_TRUE(reply.success);
			ASSERT_EQ(reply.waypoint_amount, 5);
			ASSERT_EQ(0, ThetaStarNode::OustandingObjects());
		}
	}

    TEST(LazyThetaStarTests, LazyThetaStar_20180823_1110_filtered)
	{
		ros::Publisher marker_pub;
		// (0.420435 0.313896 1.92169) to (-2.5 -10.5 3.5)
		octomap::OcTree octree ("data/20180823_1110_manyRuns_karting_filterIntensityRange.bt");
		double sidelength_lookup_table  [octree.getTreeDepth()];
	   	LazyThetaStarOctree::fillLookupTable(octree.getResolution(), octree.getTreeDepth(), sidelength_lookup_table); 
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
		path_planning_msgs::LTStarReply reply;

		for (int i = 0; i < 10; ++i)
		{
			processLTStarRequest(octree, request, reply, sidelength_lookup_table, PublishingInput(marker_pub, true, "20180823_1110_manyRuns_karting_filterIntensityRange")  );
			ASSERT_TRUE(reply.success);
			ASSERT_EQ(reply.waypoint_amount, 6);
			ASSERT_EQ(0, ThetaStarNode::OustandingObjects());
		}
	}
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}