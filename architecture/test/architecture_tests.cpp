#include <gtest/gtest.h>	
#include <architecture_math.h>


namespace architecture_math
{
	TEST(LazyThetaStarMeasurements, CalculateOrientation)
	{
        Eigen::Vector2d start (-0.0375595, 0.0711085);
        Eigen::Vector2d end   (2.21429e-17, -4.45732e-33);

        double yaw = calculateOrientation(start, end);

        ASSERT_NEAR(yaw, 5.198341144833269, 0.01);
	}

	TEST(LazyThetaStarMeasurements, CalculateOrientation_2)
	{
        Eigen::Vector2d start (0, 0);
        Eigen::Vector2d end   (0, 0);

        double yaw = calculateOrientation(start, end);

        ASSERT_NEAR(yaw, 0, 0.01);
	}


	TEST(LazyThetaStarMeasurements, CalculateOrientation_back)
	{
        Eigen::Vector2d start (0, 0);
        Eigen::Vector2d end   (0, -1);

        double yaw = calculateOrientation(start, end);

        ASSERT_NEAR(yaw, 3*M_PI/2, 0.01);
	}

	TEST(LazyThetaStarMeasurements, CalculateOrientation_front)
	{
        Eigen::Vector2d start (0, 0);
        Eigen::Vector2d end   (0, 1);

        double yaw = calculateOrientation(start, end);

        ASSERT_NEAR(yaw, 1.57, 0.01);
	}

	TEST(LazyThetaStarMeasurements, CalculateOrientation_right)
	{
        Eigen::Vector2d start (0, 0);
        Eigen::Vector2d end   (1, 0);

        double yaw = calculateOrientation(start, end);

        ASSERT_NEAR(yaw, 0, 0.01);
	}

	TEST(LazyThetaStarMeasurements, CalculateOrientation_left)
	{
        Eigen::Vector2d start (0, 0);
        Eigen::Vector2d end   (-1, 0);

        double yaw = calculateOrientation(start, end);

        ASSERT_NEAR(yaw, 3.14, 0.01);
	}

	TEST(LazyThetaStarMeasurements, CalculateOrientation_q1)
	{
        Eigen::Vector2d start (0, 0);
        Eigen::Vector2d end   (0.5, 0.9);

        double yaw = calculateOrientation(start, end);

        ASSERT_NEAR(yaw, 1.06, 0.01);
	}

	TEST(LazyThetaStarMeasurements, CalculateOrientation_q2)
	{
        Eigen::Vector2d start (0, 0);
        Eigen::Vector2d end   (-0.5, 0.9);

        double yaw = calculateOrientation(start, end);

        ASSERT_NEAR(yaw, 2.08, 0.01);
	}

	TEST(LazyThetaStarMeasurements, CalculateOrientation_q3)
	{
        Eigen::Vector2d start (0, 0);
        Eigen::Vector2d end   (-0.5, -0.9);

        double yaw = calculateOrientation(start, end);

        ASSERT_NEAR(yaw, 4.205290475992352, 0.01);
	}

	TEST(LazyThetaStarMeasurements, CalculateOrientation_q4)
	{
        Eigen::Vector2d start (0, 0);
        Eigen::Vector2d end   (0.5, -0.9);

        double yaw = calculateOrientation(start, end);

        ASSERT_NEAR(yaw, 5.219487484777026, 0.01);
	}
    TEST(LazyThetaStarMeasurements, CalculateOrientation_q1_diag)
    {
        Eigen::Vector2d start (0, 0);
        Eigen::Vector2d end   (0.5, 0.5);

        double yaw = calculateOrientation(start, end);

        ASSERT_NEAR(yaw, M_PI/4, 0.01);
    }

    TEST(LazyThetaStarMeasurements, CalculateOrientation_q2_diag)
    {
        Eigen::Vector2d start (0, 0);
        Eigen::Vector2d end   (-0.5, 0.5);

        double yaw = calculateOrientation(start, end);

        ASSERT_NEAR(yaw, (3*M_PI/4) , 0.01);
    }

    TEST(LazyThetaStarMeasurements, CalculateOrientation_q3_diag)
    {
        Eigen::Vector2d start (0, 0);
        Eigen::Vector2d end   (-0.5, -0.5);

        double yaw = calculateOrientation(start, end);

        ASSERT_NEAR(yaw, (5*M_PI/4), 0.01);
    }

    TEST(LazyThetaStarMeasurements, CalculateOrientation_q4_diag)
    {
        Eigen::Vector2d start (0, 0);
        Eigen::Vector2d end   (0.5, -0.5);

        double yaw = calculateOrientation(start, end);

        ASSERT_NEAR(yaw, (7*M_PI/4), 0.01);
    }
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}