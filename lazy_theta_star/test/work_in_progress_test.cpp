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
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}