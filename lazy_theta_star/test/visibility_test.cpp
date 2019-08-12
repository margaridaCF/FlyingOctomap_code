#include <ltStar_lib_ortho.h>
#include <gtest/gtest.h>
#include <queue>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <visualization_msgs/Marker.h>


#include <chrono>

namespace LazyThetaStarOctree
{
	TEST(LazyThetaStarMeasurements, Visibility_StartUnknown_Visible)
	{
	    octomap::OcTree octree ("data/not_visible.bt");
		ros::Publisher marker_pub;
		octomath::Vector3 start(-7.25, -12.75, 4.75);
		octomath::Vector3 end  ( -13.25, -15.75, 4.75);
		InputData input (octree, start, end, 0);
		bool has_visibility = hasLineOfSight_UnknownAsFree(input, rviz_interface::PublishingInput( marker_pub, false));
		ASSERT_TRUE(has_visibility);
	}
	
	TEST(LazyThetaStarMeasurements, Visibility_AccrossWall)
	{

	    octomap::OcTree octree ("data/-8.1422_-19.2743_9.27004_-13.75_-19.75_10.25__noPath.bt");
		ros::Publisher marker_pub;


		octomath::Vector3 start(-18, -9, 4);
		octomath::Vector3 end  (-11, -9, 4);
		InputData input (octree, start, end, 0);
		bool has_visibility = hasLineOfSight_UnknownAsFree(input, rviz_interface::PublishingInput( marker_pub, false));
		ASSERT_FALSE(has_visibility);
	    

	}

	TEST(LazyThetaStarMeasurements, Visibility_AccrossUnknown)
	{

	    octomap::OcTree octree ("data/-8.1422_-19.2743_9.27004_-13.75_-19.75_10.25__noPath.bt");
		ros::Publisher marker_pub;


		octomath::Vector3 start(-18, 0, 4);
		octomath::Vector3 end  (-11, -9, 4);
		InputData input (octree, start, end, 0);
		bool has_visibility = hasLineOfSight_UnknownAsFree(input, rviz_interface::PublishingInput( marker_pub, false));
		ASSERT_TRUE(has_visibility);
	}

	TEST(LazyThetaStarMeasurements, Visibility_AccrossFree)
	{

	    octomap::OcTree octree ("data/-8.1422_-19.2743_9.27004_-13.75_-19.75_10.25__noPath.bt");
		ros::Publisher marker_pub;


		octomath::Vector3 start(-6, -15, 4);
		octomath::Vector3 end  (-7.5, -15, 4);
		InputData input (octree, start, end, 0);
		bool has_visibility = hasLineOfSight_UnknownAsFree(input, rviz_interface::PublishingInput( marker_pub, false));
		ASSERT_TRUE(has_visibility);
	}

	TEST(LazyThetaStarMeasurements, Visibility_AccrossFree_EndOccupied)
	{

	    octomap::OcTree octree ("data/-8.1422_-19.2743_9.27004_-13.75_-19.75_10.25__noPath.bt");
		ros::Publisher marker_pub;


		octomath::Vector3 start(-6, -15, 4);
		octomath::Vector3 end  (-7.5, -15, -0.5);
		InputData input (octree, start, end, 0);
		bool has_visibility = hasLineOfSight_UnknownAsFree(input, rviz_interface::PublishingInput( marker_pub, false));
		ASSERT_FALSE(has_visibility);
	}
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}