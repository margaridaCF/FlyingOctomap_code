#include <gtest/gtest.h>
#include <frontiers.h>
#include <neighbors.h>

namespace Frontiers_test
{
	
	void constraintToMinimumSizeAndBoundaries(double & candidate_lower_bound, double & candidate_upper_bound, double lower_bound, double upper_bound, double min_length)
	{
		if(candidate_lower_bound < lower_bound)
		{
			candidate_lower_bound = lower_bound;
			candidate_upper_bound = std::min(lower_bound + min_length, upper_bound);
		}
		else if (candidate_upper_bound > upper_bound)
		{
			candidate_upper_bound = upper_bound;
			candidate_lower_bound = std::max(upper_bound - min_length, lower_bound);
		}
	}

	TEST(CirculatorTest, constraintToMinimumSizeAndBoundaries)
	{
		double candidate_lower_bound_solution = 3;
		double candidate_upper_bound_solution = 10;
		double candidate_lower_bound = 2;
		double candidate_upper_bound = 9;
		double lower_bound = 3;
		double upper_bound = 10;  
		double min_length = 7; 
		constraintToMinimumSizeAndBoundaries(candidate_lower_bound, candidate_upper_bound, lower_bound, upper_bound, min_length);
		ROS_INFO_STREAM("Result " << candidate_lower_bound << " to " << candidate_upper_bound);
		ASSERT_EQ(candidate_lower_bound_solution, candidate_lower_bound);
		ASSERT_EQ(candidate_upper_bound_solution, candidate_upper_bound);
	}

	TEST(CirculatorTest, constraintToMinimumSizeAndBoundaries_insideGeofence)
	{
		double candidate_lower_bound_solution = 3;
		double candidate_upper_bound_solution = 10;
		double candidate_lower_bound = 2;
		double candidate_upper_bound = 9;
		double lower_bound = 3;
		double upper_bound = 10;  
		double min_length = 10; 
		constraintToMinimumSizeAndBoundaries(candidate_lower_bound, candidate_upper_bound, lower_bound, upper_bound, min_length);
		ROS_INFO_STREAM("Result " << candidate_lower_bound << " to " << candidate_upper_bound);
		ASSERT_EQ(candidate_lower_bound_solution, candidate_lower_bound);
		ASSERT_EQ(candidate_upper_bound_solution, candidate_upper_bound);
	}



	TEST(CirculatorTest, constraintToMinimumSizeAndBoundaries_upper)
	{
		double candidate_lower_bound_solution = 3;
		double candidate_lower_bound = 5;
		double candidate_upper_bound = 30;
		double lower_bound = 3;
		double upper_bound = 10;  
		double min_length = 7; 
		constraintToMinimumSizeAndBoundaries(candidate_lower_bound, candidate_upper_bound, lower_bound, upper_bound, min_length);
		ROS_INFO_STREAM("Result " << candidate_lower_bound << " to " << candidate_upper_bound);
		ASSERT_EQ(candidate_lower_bound_solution, candidate_lower_bound);
		ASSERT_EQ(upper_bound, candidate_upper_bound);
	}

	TEST(CirculatorTest, constraintToMinimumSizeAndBoundaries_MONKEY)
	{
		double candidate_lower_bound_solution = 3;
		double candidate_lower_bound = -2;
		double candidate_upper_bound = 12;
		double lower_bound = 4;
		double upper_bound = 35;  
		double min_length = 7; 
		constraintToMinimumSizeAndBoundaries(candidate_lower_bound, candidate_upper_bound, lower_bound, upper_bound, min_length);
		ROS_INFO_STREAM("Result " << candidate_lower_bound << " to " << candidate_upper_bound);
		ASSERT_EQ(candidate_lower_bound_solution, candidate_lower_bound);
		ASSERT_EQ(upper_bound, candidate_upper_bound);
	}

}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}