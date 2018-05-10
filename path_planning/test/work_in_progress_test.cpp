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

	TEST(LazyThetaStarTests, LazyThetaStar_visibility)
	{
		ros::Publisher marker_pub;
		// (0.420435 0.313896 1.92169) to (-2.5 -10.5 3.5)
		octomap::OcTree octree ("data/d.bt");
		std::unordered_set<std::shared_ptr<octomath::Vector3>> neighbors;

		octomath::Vector3 point_coordinates (-8.5,6.5,2.5);
		octomap::OcTreeKey point_key = octree.coordToKey(point_coordinates);
		int depth = getNodeDepth_Octomap(point_key, octree);
		double node_size = octree.getNodeSize(depth); // in meters
		double resolution = octree.getResolution();
		// ACT
		octomath::Vector3 cell_center_coordinates = getCellCenter(point_coordinates, octree);
		ROS_INFO_STREAM("Goal " << point_coordinates << " cell center " << cell_center_coordinates << " size " << node_size);
		generateNeighbors_pointers(neighbors, point_coordinates, node_size, resolution);
		for (std::unordered_set<std::shared_ptr<octomath::Vector3>>::iterator it = neighbors.begin(); it != neighbors.end(); ++it)
		{
			if(isExplored)
			{
				if(isOccupied(**it, octree))
				{
					ROS_INFO_STREAM(**it << " is occupied.");
				}
				else
				{
					ROS_INFO_STREAM(**it << " is free.");
				}
			}
			else
			{
				ROS_INFO_STREAM(**it << " is unknown.");
			}
		}

	}
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}