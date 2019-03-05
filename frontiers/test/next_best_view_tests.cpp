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
    	NextBestViewSM nbv_state_machine( distance_inFront,  distance_behind, circle_divisions,  frontier_safety_margin);	
		octomap::OcTree octree ("data/circle_1m.bt");
		int request_number = 0;
		int amount = 10;
		std::vector<observation_lib::OPPair> oppairs;

		// ACT
    	nbv_state_machine.NewRequest(&octree, request_number, amount);
		bool success = nbv_state_machine.FindNextOPPairs(amount, oppairs, request_number+1);

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
    	NextBestViewSM nbv_state_machine( distance_inFront,  distance_behind, circle_divisions,  frontier_safety_margin);		
		octomap::OcTree octree ("data/circle_1m.bt");
		int request_number = 0;
		int amount = 10;
		std::vector<observation_lib::OPPair> oppairs;

		// ACT
    	nbv_state_machine.NewRequest(&octree, request_number, amount);
		bool success = nbv_state_machine.FindNextOPPairs(amount, oppairs, request_number);

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