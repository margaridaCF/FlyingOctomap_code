#include <ltStar_temp.h>
#include <gtest/gtest.h>
#include <queue>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <visualization_msgs/Marker.h>


namespace LazyThetaStarOctree{

	TEST(LazyThetaStarTests, LazyThetaStar_throughWall)
	{
		ros::Publisher marker_pub;
		// (0.420435 0.313896 1.92169) to (-2.5 -10.5 3.5)
		octomap::OcTree octree ("data/(-11.2177; -18.2778; 2.39616)_(-8.5; 6.5; 3.5)_throughWall.bt");
		path_planning_msgs::LTStarRequest request;
		request.header.seq = 2;
		request.request_id = 3;
		request.start.x = -11.2177;
		request.start.y = -18.2778;
		request.start.z = 2.39616;
		request.goal.x = -8.5;
		request.goal.y = 6.5;
		request.goal.z = 3.5;
		request.max_search_iterations = 1000;
		request.safety_margin = 1;
		path_planning_msgs::LTStarReply reply;
		processLTStarRequest(octree, request, reply, marker_pub);
		ASSERT_FALSE(reply.success);
		ASSERT_EQ(0, ThetaStarNode::OustandingObjects());
	}

	TEST(LazyThetaStarTests, LazyThetaStar_AddingBadNode)
	{
		ros::Publisher marker_pub;
		// (0.420435 0.313896 1.92169) to (-2.5 -10.5 3.5)
		octomap::OcTree octree ("data/(-10.3054; -18.2637; 2.34813)_(-8.5; 6.5; 3.5)_badNodeAdded.bt");
		path_planning_msgs::LTStarRequest request;
		request.header.seq = 2;
		request.request_id = 3;
		request.start.x = -10.3054;
		request.start.y = -18.2637;
		request.start.z = 2.34813;
		request.goal.x = -8.5;
		request.goal.y = 6.5;
		request.goal.z = 3.5;
		request.max_search_iterations = 500;
		request.safety_margin = 0.5;
		path_planning_msgs::LTStarReply reply;
		processLTStarRequest(octree, request, reply, marker_pub);
		ASSERT_TRUE(reply.success);
		ASSERT_GT(reply.waypoint_amount, 2);
		ASSERT_EQ(0, ThetaStarNode::OustandingObjects());
	}


	// TEST(LazyThetaStarTests, LazyThetaStar_corridorFree_1)
	// {
	// 	ros::Publisher marker_pub;
	// 	octomap::OcTree octree ("data/(-10.3054; -18.2637; 2.34813)_(-8.5; 6.5; 3.5)_badNodeAdded.bt");
	// 	octomath::Vector3 start(-11, -15, 3);
	// 	octomath::Vector3 end(-11.5, -13.5, 2.5);
	// 	double safety_margin = 0;

	// 	octomath::Vector3 bounding_box_size(safety_margin, safety_margin, safety_margin);
	// 	bool start_to_end = getLineStatusBoundingBox(octree, start, end, bounding_box_size) == CellStatus::kFree;
	// 	ROS_ERROR_STREAM("reverse");
	// 	bool end_to_start = getLineStatusBoundingBox(octree, end, start, bounding_box_size) == CellStatus::kFree;

	// 	ASSERT_EQ(start_to_end, end_to_start);
	// }

	// TEST(LazyThetaStarTests, LazyThetaStar_getLineStatus)
	// {
	// 	ros::Publisher marker_pub;
	// 	octomap::OcTree octree ("data/(-10.3054; -18.2637; 2.34813)_(-8.5; 6.5; 3.5)_badNodeAdded.bt");
	// 	octomath::Vector3 start(-11, -15, 3);
	// 	octomath::Vector3 end(-11.5, -13.5, 2.5);
	// 	double safety_margin = 0;

	// 	octomath::Vector3 bounding_box_size(safety_margin, safety_margin, safety_margin);
	// 	bool start_to_end = getLineStatus(octree, start, end) == CellStatus::kFree;
	// 	ROS_ERROR_STREAM("reverse");
	// 	bool end_to_start = getLineStatus(octree, end, start) == CellStatus::kFree;

	// 	ASSERT_EQ(start_to_end, end_to_start);
	// }


	TEST(LazyThetaStarTests, LazyThetaStar_corridorFree_2)
	{
		ros::Publisher marker_pub;
		octomap::OcTree octree ("data/(-10.3054; -18.2637; 2.34813)_(-8.5; 6.5; 3.5)_badNodeAdded.bt");
		octomath::Vector3 start(-11, -15, 3);
		octomath::Vector3 end(-11.5, -13.5, 3.5);
		double safety_margin = 0;
		bool start_to_end = is_flight_corridor_free(octree, start, end, safety_margin, marker_pub);
		bool end_to_start = is_flight_corridor_free(octree, end, start, safety_margin, marker_pub);
		ASSERT_EQ(start_to_end, end_to_start);
	}

	TEST(LazyThetaStarTests, LazyThetaStar_corridorFree_3)
	{
		ros::Publisher marker_pub;
		octomap::OcTree octree ("data/(-10.3054; -18.2637; 2.34813)_(-8.5; 6.5; 3.5)_badNodeAdded.bt");
		octomath::Vector3 start(-10.5, -13.5, 3.5);
		octomath::Vector3 end(-11.5, -13.5, 3.5);
		double safety_margin = 0;
		bool start_to_end = is_flight_corridor_free(octree, start, end, safety_margin, marker_pub);
		bool end_to_start = is_flight_corridor_free(octree, end, start, safety_margin, marker_pub);
		ASSERT_EQ(start_to_end, end_to_start);
	}

	TEST(LazyThetaStarTests, LazyThetaStar_corridorFree_4)
	{
		ros::Publisher marker_pub;
		octomap::OcTree octree ("data/(-10.3054; -18.2637; 2.34813)_(-8.5; 6.5; 3.5)_badNodeAdded.bt");
		octomath::Vector3 start(-10.5, -13.5, 3.5);
		octomath::Vector3 end(-10.5, -12.5, 3.5);
		double safety_margin = 0;
		bool start_to_end = is_flight_corridor_free(octree, start, end, safety_margin, marker_pub);
		bool end_to_start = is_flight_corridor_free(octree, end, start, safety_margin, marker_pub);
		ASSERT_EQ(start_to_end, end_to_start);
	}

	TEST(LazyThetaStarTests, LazyThetaStar_corridorFree_merge_1)
	{
		ros::Publisher marker_pub;
		octomap::OcTree octree ("data/(-10.3054; -18.2637; 2.34813)_(-8.5; 6.5; 3.5)_badNodeAdded.bt");
		octomath::Vector3 start(-11, -15, 3);
		octomath::Vector3 end(-11.5, -13.5, 2.5);
		double safety_margin = 0;

		octomath::Vector3 bounding_box_size(safety_margin, safety_margin, safety_margin);
		bool start_to_end = getLineStatusBoundingBox(octree, start, end, bounding_box_size) == CellStatus::kFree;
		ROS_ERROR_STREAM("reverse");
		bool end_to_start = getLineStatusBoundingBox(octree, end, start, bounding_box_size) == CellStatus::kFree;

		ASSERT_EQ(start_to_end, end_to_start);
	}

	TEST(LazyThetaStarTests, LazyThetaStar_corridorFree_merge_2)
	{
		ros::Publisher marker_pub;
		octomap::OcTree octree ("data/(-10.3054; -18.2637; 2.34813)_(-8.5; 6.5; 3.5)_badNodeAdded.bt");
		octomath::Vector3 start(-11, -15, 3);
		octomath::Vector3 end(-11.5, -13.5, 3.5);
		double safety_margin = 0;
		bool start_to_end = is_flight_corridor_free(octree, start, end, safety_margin, marker_pub);
		bool end_to_start = is_flight_corridor_free(octree, end, start, safety_margin, marker_pub);
		ASSERT_EQ(start_to_end, end_to_start);
	}

	TEST(LazyThetaStarTests, LazyThetaStar_corridorFree_merge_3)
	{
		ros::Publisher marker_pub;
		octomap::OcTree octree ("data/(-10.3054; -18.2637; 2.34813)_(-8.5; 6.5; 3.5)_badNodeAdded.bt");
		octomath::Vector3 start(-10.5, -13.5, 3.5);
		octomath::Vector3 end(-11.5, -13.5, 3.5);
		double safety_margin = 0;
		bool start_to_end = is_flight_corridor_free(octree, start, end, safety_margin, marker_pub);
		bool end_to_start = is_flight_corridor_free(octree, end, start, safety_margin, marker_pub);
		ASSERT_EQ(start_to_end, end_to_start);
	}

	TEST(LazyThetaStarTests, LazyThetaStar_corridorFree_merge_4)
	{
		ros::Publisher marker_pub;
		octomap::OcTree octree ("data/(-10.3054; -18.2637; 2.34813)_(-8.5; 6.5; 3.5)_badNodeAdded.bt");
		octomath::Vector3 start(-10.5, -13.5, 3.5);
		octomath::Vector3 end(-10.5, -12.5, 3.5);
		double safety_margin = 0;
		bool start_to_end = is_flight_corridor_free(octree, start, end, safety_margin, marker_pub);
		bool end_to_start = is_flight_corridor_free(octree, end, start, safety_margin, marker_pub);
		ASSERT_EQ(start_to_end, end_to_start);
	}
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}