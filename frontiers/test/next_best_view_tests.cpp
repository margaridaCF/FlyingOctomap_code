#include <next_best_view.h>
#include <gtest/gtest.h>

namespace NextBestView{


	bool equal (const octomath::Vector3 & a, const octomath::Vector3 & b, 
		const double theta = 0.00000000000000000001) 
	{

		bool is_x_equal = abs(a.x() - b.x()) < theta;
		bool is_y_equal = abs(a.y() - b.y()) < theta;
		bool is_z_equal = abs(a.z() - b.z()) < theta;

		return is_x_equal && is_y_equal && is_z_equal;
	}

	
	TEST(NextBestViewTests, WrongRequestNumber)
	{
		// ARRANGE
		double distance_inFront = 1;
		double distance_behind = 1;
		int circle_divisions = 1;
		double frontier_safety_margin = 3;
		ros::Publisher marker_pub;
		bool publish = false;
    	NextBestViewSM nbv_state_machine( distance_inFront,  distance_behind, circle_divisions, frontier_safety_margin, marker_pub, publish);	
		octomap::OcTree octree ("data/circle_1m.bt");
		std::vector<observation_lib::OPPair> oppairs;
		geometry_msgs::Point min, max;
		min.x = 0;
		min.y = 0;
		min.z = 0;
		max.x = 6;
		max.y = 2;
		max.z = 2;	
		frontiers_msgs::FrontierRequest request;
		request.frontier_amount = 10;
		request.request_number = 0;

		// ACT
    	nbv_state_machine.NewRequest(&octree, request.request_number, request.amount, max, min);
		request.request_number++;
		bool success = nbv_state_machine.FindNext(request, oppairs);

    	// ASSERT
		ASSERT_FALSE(success);
	}
	
	TEST(NextBestViewTests, CorrectRequestNumber)
	{
		// ARRANGE
		double distance_inFront = 1;
		double distance_behind = 1;
		int circle_divisions = 1;
		double frontier_safety_margin = 3;
		bool publish = false;
    	NextBestViewSM nbv_state_machine( distance_inFront,  distance_behind, circle_divisions, frontier_safety_margin, marker_pub, publish);	
		octomap::OcTree octree ("data/circle_1m.bt");	
		frontiers_msgs::FrontierRequest request;
		request.frontier_amount = 10;
		request.request_number = 0;
		std::vector<observation_lib::OPPair> oppairs;
		geometry_msgs::Point min, max;
		min.x = 0;
		min.y = 0;
		min.z = 0;
		max.x = 6;
		max.y = 2;
		max.z = 2;

		// ACT
    	nbv_state_machine.NewRequest(&octree, request.request_number, request.amount, max, min);
		bool success = nbv_state_machine.FindNext(request, oppairs);

    	// ASSERT
		ASSERT_TRUE(success);
	}
	TEST(NextBestViewTests, NextGoal)
	{
		// New REquest

		// Ask for goals must be successfull. Number of goals variable
	}
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}