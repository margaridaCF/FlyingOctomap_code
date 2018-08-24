#include <orthogonal_planes.h>
#include <gtest/gtest.h>

namespace LazyThetaStarOctree{
	void testCoordinateFrame(octomath::Vector3 start, octomath::Vector3 goal)
	{
		CoordinateFrame coordinate_frame = generateCoordinateFrame(start, goal);

		// All vectors are orthogonal
		ASSERT_NEAR(coordinate_frame.orthogonalA.dot(coordinate_frame.orthogonalB), 0, 0.0000001)  << "All unitary vector must be orthogonal. (dot product equals 0)";
		ASSERT_NEAR(coordinate_frame.orthogonalA.dot(coordinate_frame.direction), 0, 0.0000001) << "All unitary vector must be orthogonal. (dot product equals 0)";
		ASSERT_NEAR(coordinate_frame.orthogonalB.dot(coordinate_frame.direction), 0, 0.0000001) << "All unitary vector must be orthogonal. (dot product equals 0)";
		
		// All vectors are unitary
		ASSERT_NEAR(coordinate_frame.direction.norm(),   1, 0.0001) << "Unitary vector should have size 1";
		ASSERT_NEAR(coordinate_frame.orthogonalA.norm(), 1, 0.0001) << "Unitary vector should have size 1";
		ASSERT_NEAR(coordinate_frame.orthogonalB.norm(), 1, 0.0001) << "Unitary vector should have size 1";
	}


	TEST(OrthogonalPlanesTest, generateCoordinateFrame_originToYaxis)
	{
		octomath::Vector3 start (0, 0, 0);
		octomath::Vector3 goal  (0, 1, 0);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_originToNegatativeYaxis)
	{
		octomath::Vector3 start (0, 0, 0);
		octomath::Vector3 goal  (0, -1, 0);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_originToXaxis)
	{
		octomath::Vector3 start (0, 0, 0);
		octomath::Vector3 goal  (1, 0, 0);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_originToNegatativeXaxis)
	{
		octomath::Vector3 start (0, 0, 0);
		octomath::Vector3 goal  (-1, 0, 0);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_originToZaxis)
	{
		octomath::Vector3 start (0, 0, 0);
		octomath::Vector3 goal  (0, 0, 1);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_closeToOriginToZaxis)
	{
		octomath::Vector3 start (0, 0, 0);
		octomath::Vector3 goal  (0.1, 0, 1);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_originToNegatativeZaxis)
	{
		octomath::Vector3 start (0, 0, 0);
		octomath::Vector3 goal  (0, 0, -1);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_originToXaxisZero)
	{
		octomath::Vector3 start (0, 0, 0);
		octomath::Vector3 goal  (0, 1, 1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(0, -1, 1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(0, -1, -1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(0, 1, -1);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_originToYaxisZero)
	{
		octomath::Vector3 start (0, 0, 0);
		octomath::Vector3 goal  (1, 0, 1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(-1, 0, 1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(-1, 0, -1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(1, 0, -1);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_originToAllDiagonals)
	{
		octomath::Vector3 start (0, 0, 0);
		octomath::Vector3 goal  (1, 1, 1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(-1, 1, 1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(-1, 1, -1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(1, 1, -1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(1, -1, 1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(-1, -1, 1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(-1, -1, -1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(1, -1, -1);
		testCoordinateFrame(start, goal);
	}

	TEST(OrthogonalPlanesTest, generateCoordinateFrame_offset_originToYaxis)
	{
		octomath::Vector3 start (2, 5, -634);
		octomath::Vector3 goal  (0, 1, 0);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_offset_originToNegatativeYaxis)
	{
		octomath::Vector3 start (2, 5, -634);
		octomath::Vector3 goal  (0, -1, 0);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_offset_originToXaxis)
	{
		octomath::Vector3 start (2, 5, -634);
		octomath::Vector3 goal  (1, 0, 0);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_offset_originToNegatativeXaxis)
	{
		octomath::Vector3 start (2, 5, -634);
		octomath::Vector3 goal  (-1, 0, 0);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_offset_originToZaxis)
	{
		octomath::Vector3 start (2, 5, -634);
		octomath::Vector3 goal  (0, 0, 1);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_offset_closeToOriginToZaxis)
	{
		octomath::Vector3 start (2, 5, -634);
		octomath::Vector3 goal  (0.1, 0, 1);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_offset_originToNegatativeZaxis)
	{
		octomath::Vector3 start (2, 5, -634);
		octomath::Vector3 goal  (0, 0, -1);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_offset_originToXaxisZero)
	{
		octomath::Vector3 start (2, 5, -634);
		octomath::Vector3 goal  (0, 1, 1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(0, -1, 1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(0, -1, -1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(0, 1, -1);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_offset_originToYaxisZero)
	{
		octomath::Vector3 start (2, 5, -634);
		octomath::Vector3 goal  (1, 0, 1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(-1, 0, 1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(-1, 0, -1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(1, 0, -1);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_offset_originToAllDiagonals)
	{
		octomath::Vector3 start (2, 5, -634);
		octomath::Vector3 goal  (1, 1, 1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(-1, 1, 1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(-1, 1, -1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(1, 1, -1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(1, -1, 1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(-1, -1, 1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(-1, -1, -1);
		testCoordinateFrame(start, goal);
		goal = octomath::Vector3(1, -1, -1);
		testCoordinateFrame(start, goal);
	}



	TEST(OrthogonalPlanesTest, generateCoordinateFrame_monkeyTest)
	{
		octomath::Vector3 start (2, 5, -634);
		octomath::Vector3 goal  (20, -31, 40);
		testCoordinateFrame(start, goal);
	}

	TEST(OrthogonalPlanesTest, generateCoordinateFrame_monkeyTest1)
	{
		octomath::Vector3 start (2, 5, -634);
		octomath::Vector3 goal  (20, -31, 40);
		testCoordinateFrame(start, goal);
	}

	TEST(OrthogonalPlanesTest, generateCoordinateFrame_monkeyTest2)
	{
		octomath::Vector3 start (-2, 785, -634);
		octomath::Vector3 goal  (20, -31, 40);
		testCoordinateFrame(start, goal);
	}

	TEST(OrthogonalPlanesTest, generateCoordinateFrame_monkeyTest3)
	{
		octomath::Vector3 start (-2, 5, -634);
		octomath::Vector3 goal  (20, -31, -40);
		testCoordinateFrame(start, goal);
	}
	TEST(OrthogonalPlanesTest, generateCoordinateFrame_monkeyTest4)
	{
		octomath::Vector3 start (2, 25, -634);
		octomath::Vector3 goal  (20, -31, 40);
		testCoordinateFrame(start, goal);
	}

	TEST(OrthogonalPlanesTest, generateCoordinateFrame_monkeyTest5)
	{
		octomath::Vector3 start (2, -5, -634);
		octomath::Vector3 goal  (20, -31, 40);
		testCoordinateFrame(start, goal);
	}

	TEST(OrthogonalPlanesTest, generateCoordinateFrame_monkeyTest6)
	{
		octomath::Vector3 start (-2, 785, -634);
		octomath::Vector3 goal  (-20, -31, 40);
		testCoordinateFrame(start, goal);
	}

	TEST(OrthogonalPlanesTest, generateCoordinateFrame_monkeyTest7)
	{
		octomath::Vector3 start (-2, -5, -634);
		octomath::Vector3 goal  (-20, -31, -40);
		testCoordinateFrame(start, goal);
	}

}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}