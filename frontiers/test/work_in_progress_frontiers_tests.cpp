#include <gtest/gtest.h>
#include <next_best_view.h>
#include <frontiers.h>
#include <neighbors.h>

namespace Frontiers
{
	void printForMatlab(std::unordered_set<std::shared_ptr<octomath::Vector3>> neighbors)
	{
		std::cout << " x_values = [ ];\n";
		std::cout << " y_values = [ ];\n";
		std::cout << " z_values = [ ];\n";
		for (std::shared_ptr<octomath::Vector3> n : neighbors)
		{
			std::cout << " x_values = [x_values, " << n->x() << "];\n ";
			std::cout << " y_values = [y_values, " << n->y() << "];\n ";
			std::cout << " z_values = [z_values, " << n->z() << "];\n ";
		}
	}


	TEST(OctreeNeighborTest, NeighborTest_generateFromRealData_MaxRes_2D)
	{
		// ARRANGE
		octomap::OcTree octree ("data/circle_1m.bt");
		double distance_inFront = 1;
		double distance_behind = 1;
		double circle_divisions = 12;
		double frontier_safety_margin = 4;
		ros::Publisher marker_pub;
    	Frontiers::NextBestViewSM nbv_state_machine( distance_inFront, distance_behind, circle_divisions, frontier_safety_margin, marker_pub, true);

    	frontiers_msgs::FrontierRequest request;
		request.header.seq = 1;
		request.header.frame_id = "request_frame";
		request.min.x = 0;
		request.min.y = 0;
		request.min.z = 2;
		request.max.x = 10;
		request.max.y = 10;
		request.max.z = 10;
		request.current_position.x = 0;
		request.current_position.y = 0;
		request.current_position.z = 2;
		request.frontier_amount = 10;
		request.min_distance = 4.0;
		request.safety_margin = 4.0;
		request.sensing_distance = 5.0;
		request.sensor_angle = 0.0;
		request.new_request = true ;

		frontiers_msgs::FrontierReply reply;
		
		// ACT
		nbv_state_machine.NewRequest(&octree, request);
		std::vector<observation_lib::OPPair> oppairs;
		nbv_state_machine.FindNext(request, reply, oppairs);

		// ASSERT
		ASSERT_EQ(reply.frontiers_found, request.frontier_amount);
		LazyThetaStarOctree::unordered_set_pointers result;
		for (int i = 0; i < request.frontier_amount; ++i)
		{
			ROS_INFO_STREAM("(" << reply.frontiers[i].xyz_m.x << ", " << reply.frontiers[i].xyz_m.y << ", " << reply.frontiers[i].xyz_m.x << ")");
	        std::shared_ptr<octomath::Vector3> toInsert_ptr = std::make_shared<octomath::Vector3> (reply.frontiers[i].xyz_m.x,  reply.frontiers[i].xyz_m.y, reply.frontiers[i].xyz_m.x);
			result.insert(toInsert_ptr);
		}
		ASSERT_EQ(result.size(), request.frontier_amount);
	}
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}