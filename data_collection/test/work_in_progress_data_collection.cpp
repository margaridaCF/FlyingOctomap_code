#include <gtest/gtest.h>
#include <collect_data.h>

namespace Frontiers_test
{
	

	TEST(DataCollection, FrontiersPerFlyby)
	{
		ros::Publisher marker_pub;
		octomap::OcTree octree ("data/experimentalDataset.bt");
		frontiers_msgs::FindFrontiers find_frontiers_msg;
		find_frontiers_msg.request.min.x = 0;
		find_frontiers_msg.request.min.y = 0;
		find_frontiers_msg.request.min.z = 0;
		find_frontiers_msg.request.max.x = 6;
		find_frontiers_msg.request.max.y = 2;
		find_frontiers_msg.request.max.z = 2;
		find_frontiers_msg.request.frontier_amount = 1;
		Frontiers::processFrontiersRequest(octree, find_frontiers_msg.request, find_frontiers_msg.response, marker_pub, false);
		// checkFrontiers(octree, find_frontiers_msg.request, find_frontiers_msg.response);
		
	}

}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}