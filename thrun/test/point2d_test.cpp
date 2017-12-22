#define TEST_DIR "@TEST_WITH_DATA_TEST_DIR@"

#include <gtest/gtest.h>

#include <point2d_lib.h>

#include <iostream>
#include <fstream>

namespace mapper {
	class Point2DTest : public ::testing::Test {
	protected:
	  	Point2DTest() 
	  	{
			
	  	}

		virtual ~Point2DTest() {
		// You can do clean-up work that doesn't throw exceptions here.
		}
	};

	TEST(Point2DTest, DivisionByConstant) {
		Point2d tens (10,10);
		Point2d result = tens/10.f;
		EXPECT_EQ(result.x, 1);
		EXPECT_EQ(result.y, 1);
	}

	TEST(Point2DTest, DivisionByZero) {
		Point2d tens (10,10);
		bool throwException = false;
		try {
			Point2d result = tens/0.f;
	    } catch (std::overflow_error e) {
	        throwException = true;
	    }
		EXPECT_TRUE(throwException);
	}

	TEST(Point2DTest, SubtractTwoOperands) {
		Point2d first (10,10);
		Point2d second (2.f, 15.3f);
		Point2d result = first - second;
		EXPECT_EQ(result.x, 8);
		EXPECT_EQ(result.y, -5.3f);
	}
	TEST(Point2DTest, SubtractByItself) {
		Point2d itself (10,10);
		Point2d amount (2.f, 15.3f);
		itself -= amount;
		EXPECT_EQ(itself.x, 8);
		EXPECT_EQ(itself.y, -5.3f);
	}
	TEST(Point2DTest, SubtractTwoOperands_constant) {
		Point2d point (10,10);
		float constant =15.3f;
		Point2d result = point - constant;
		EXPECT_EQ(result.x, -5.3f);
		EXPECT_EQ(result.y, -5.3f);
	}
	TEST(Point2DTest, SubtractByItself_constant) {
		Point2d point (10,10);
		float constant = 5.3f;
		point -= constant;
		EXPECT_EQ(4.7f, point.x);
		EXPECT_EQ(4.7f, point.y);
	}

	TEST(Point2DTest, AddTwoOperands_constant) {
		Point2d point (10,10);
		float constant = 15.3f;
		Point2d result = point + constant;
		EXPECT_EQ(result.x, 25.3f);
		EXPECT_EQ(result.y, 25.3f);
	}
	TEST(Point2DTest, AddByItself_constant) {
		Point2d point (10,10);
		float constant = 5.3f;
		point += constant;
		EXPECT_EQ(15.3f, point.x);
		EXPECT_EQ(15.3f, point.y);
	}

	TEST(Point2DTest, MultiplyTwoOperands_constant) {
		Point2d point (10,10);
		float constant = 15.3f;
		Point2d result = point * constant;
		EXPECT_EQ(result.x, 153.f);
		EXPECT_EQ(result.y, 153.f);
	}
	TEST(Point2DTest, MultiplyByItself_constant) {
		Point2d point (10,10);
		float constant = 5.3f;
		point *= constant;
		EXPECT_EQ(53.f, point.x);
		EXPECT_EQ(53.f, point.y);
	}

	TEST(Point2DTest, EqualOperand_True) {
		Point2d first (10,10);
		bool result = (first == first);
		EXPECT_TRUE(result);
	}
	TEST(Point2DTest, EqualOperand_False) {
		Point2d first (10,10);
		Point2d second (2.f, 15.3f);
		bool result = (first == second);
		EXPECT_FALSE(result);
	}

	/// -- MATH FUNCTIONS --
	TEST(Point2DTest, DistanceBetweenPoints){
		Point2d p1(-4.f, 7.f);
		Point2d p2 (1.8f, -10.f);
		float result = p1.distance(p2);
		EXPECT_LT (std::fabs(result - 17.9622), 0.1f);
		p1 = Point2d (0.4f, 0.4f);
		p2 = Point2d (2.45f, 0.45f);
		result = p1.distance(p2);
		EXPECT_LT (std::fabs(result - 2.0506), 0.1f);
		//EXPECT_EQ (17.962182495454165, result);
	}
	TEST(Point2DTest, DistanceBetweenSamePoint){
		Point2d p1(-4.f, 7.f);
		EXPECT_EQ (0.f, p1.distance(p1));
	}



// More tests 
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
