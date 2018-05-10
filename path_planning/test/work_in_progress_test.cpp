#include <ltStar_temp.h>
#include <gtest/gtest.h>
#include <queue>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <visualization_msgs/Marker.h>


namespace LazyThetaStarOctree{

	// TEST(LazyThetaStarTests, LazyThetaStar_NoSolutionFound)
	// {
	// 	ros::Publisher marker_pub;
	// 	// (0.420435 0.313896 1.92169) to (-2.5 -10.5 3.5)
	// 	octomap::OcTree octree ("data/(10.9653; -14.8729; 3.00539)_(-8.5; 6.5; 2.5)_SolutionNotFound.bt");
	// 	path_planning_msgs::LTStarRequest request;
	// 	request.header.seq = 6;
	// 	request.request_id = 7;
	// 	request.start.x = 10.9653;
	// 	request.start.y = -14.8729;
	// 	request.start.z = 3.00539;
	// 	request.goal.x = -8.5;
	// 	request.goal.y = 6.5;
	// 	request.goal.z = 2.5;
	// 	request.max_search_iterations = 5000;
	// 	request.safety_margin = 1;
	// 	path_planning_msgs::LTStarReply reply;
	// 	processLTStarRequest(octree, request, reply, marker_pub);
	// 	ASSERT_TRUE(reply.success);
	// 	ASSERT_GT(reply.waypoint_amount, 2);
	// 	ASSERT_EQ(0, ThetaStarNode::OustandingObjects());
	// }

	bool isOccupied(octomath::Vector3 const& grid_coordinates_toTest, octomap::OcTree const& octree)
    {
        octomap::OcTreeNode* result = octree.search(grid_coordinates_toTest.x(), grid_coordinates_toTest.y(), grid_coordinates_toTest.z());
        if(result == NULL){
            return false;
        }
        else
        {
            return octree.isNodeOccupied(result);
        }
    }
    
    bool isExplored(octomath::Vector3 const& grid_coordinates_toTest, octomap::OcTree const& octree)
    {
        octomap::OcTreeNode* result = octree.search(grid_coordinates_toTest.x(), grid_coordinates_toTest.y(), grid_coordinates_toTest.z());
        if(result == NULL){
            return false;
        }
        else
        {
            return true;
        }
    }

	// TEST(LazyThetaStarTests, LazyThetaStar_visibility)
	// {
	// 	ros::Publisher marker_pub;
	// 	// (0.420435 0.313896 1.92169) to (-2.5 -10.5 3.5)
	// 	octomap::OcTree octree ("data/d.bt");
	// 	std::unordered_set<std::shared_ptr<octomath::Vector3>> neighbors;

	// 	octomath::Vector3 point_coordinates (10.5, -5.5, 2.5);
	// 	octomap::OcTreeKey point_key = octree.coordToKey(point_coordinates);
	// 	int depth = getNodeDepth_Octomap(point_key, octree);
	// 	double node_size = octree.getNodeSize(depth); // in meters
	// 	double resolution = octree.getResolution();
	// 	// ACT
	// 	octomath::Vector3 cell_center_coordinates = getCellCenter(point_coordinates, octree);
	// 	ROS_INFO_STREAM("Goal " << point_coordinates << " cell center " << cell_center_coordinates << " size " << node_size);
	// 	generateNeighbors_pointers(neighbors, point_coordinates, node_size, resolution);
	// 	for (std::unordered_set<std::shared_ptr<octomath::Vector3>>::iterator it = neighbors.begin(); it != neighbors.end(); ++it)
	// 	{
	// 		if(isExplored)
	// 		{
	// 			if(isOccupied(**it, octree))
	// 			{
	// 				ROS_INFO_STREAM(**it << " is occupied.");
	// 			}
	// 			else
	// 			{
	// 				ROS_INFO_STREAM(**it << " is free.");
	// 			}
	// 		}
	// 		else
	// 		{
	// 			ROS_INFO_STREAM(**it << " is unknown.");
	// 		}
	// 	}

	// }

	

	TEST(LazyThetaStarTests, ValidPathCloseToTheGoal)
	{
		int has_line_of_sight_count = 0;

		std::ostream &  log_file = std::cout;
		ros::Publisher marker_pub;
		octomap::OcTree octree ("data/d.bt");
		double resolution = octree.getResolution();
		double safety_margin = 1.1;
		// Closer than 2m
		// (9.500000; -4.500000; 1.500000 ) @ 1.000000; g = 9.359971; to final = 1.732051; parent is 0xdd3930
		// (9.000000; -5.000000; 3.000000 ) @ 2.000000; g = 8.944272; to final = 1.658312; parent is 0xda2090

		std::shared_ptr<ThetaStarNode> s = std::make_shared<ThetaStarNode>(std::make_shared<octomath::Vector3>(9.5, -4.5, 1.5), 1);

		log_file << "[START] s is " << s << std::endl;
		
		std::unordered_set<std::shared_ptr<octomath::Vector3>> neighbors;
		generateNeighbors_pointers(neighbors, *(s->coordinates), s->cell_size, resolution);

		// TODO check code repetition to go over the neighbors of s
		double cell_size = 0;
		// ln 12 foreach s' € nghbr_vis(s) do
		for(std::shared_ptr<octomath::Vector3> n_coordinates : neighbors)
		{
			// ROS_WARN_STREAM("@"<< used_search_iterations << "  Analyzing neighbor " << *n_coordinates);
			// ROS_WARN_STREAM("Existing Node objects " << ThetaStarNode::OustandingObjects());
			// Find minimum value for those with visibility and that it is in closed
			// TODO for neighbor pointd that belong to the same cell, the  calculations are duplicated it shouldn't be too hard to optimize this to skip all subsequent calculations (with a list of something)
			if(!normalizeToVisibleEndCenter(octree, s->coordinates, n_coordinates, cell_size, safety_margin, marker_pub, false))
			{
				// auto res_node = octree.search(*n_coordinates);
				// if(res_node == NULL)
				// {
 				// 	throw std::out_of_range("Skipping cases where unknown neighbors are found.");
				// }
				log_file << "[N] no line of sight " << *(s->coordinates) << " to " << *n_coordinates << ". Distance to goal " << weightedDistance(*(s->coordinates), *n_coordinates) << std::endl;
				continue;
			}
			else 
			{
				has_line_of_sight_count++;
				log_file << "[N] visible neighbor " << *n_coordinates << ". Distance to goal " << weightedDistance(*(s->coordinates), *n_coordinates) << std::endl;
			}
		}
		ASSERT_GT(has_line_of_sight_count, 0);
	}
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}