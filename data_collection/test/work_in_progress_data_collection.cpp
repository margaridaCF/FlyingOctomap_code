#include <collect_data.h>

namespace collect_data
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
		find_frontiers_msg.request.new_request = true;
		find_frontiers_msg.request.current_position.x = 0;
		find_frontiers_msg.request.current_position.y = 0;
		find_frontiers_msg.request.current_position.z = 1;
		std::srand(std::time(0));
		std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
		Frontiers::processFrontiersRequest(octree, find_frontiers_msg.request, find_frontiers_msg.response, marker_pub, false);
		std::chrono::system_clock::time_point end = std::chrono::system_clock::now();

		checkFrontiers(octree, find_frontiers_msg.request, find_frontiers_msg.response);

		#ifdef SAVE_CSV
		
#endif
	}

}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}