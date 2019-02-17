#include <gtest/gtest.h>
#include <architecture_math.h>


namespace architecture_math
{
	TEST(LazyThetaStarMeasurements, AllCombinations)
	{
		

        Eigen::Vector3d start (-0.0375595, 0.0711085, 1.88483);
        Eigen::Vector3d end   (2.21429e-17, -4.45732e-33, 10);

        double yaw = calculateOrientation(start, end);

        ASSERT_EQ(yaw, 0);
	}

}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}