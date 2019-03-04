#include <ltStar_lib_ortho.h>
#include <voxel.h>
#include <octomap/OcTreeNode.h>
#include <gtest/gtest.h>

namespace LazyThetaStarOctree{

	int search (octomap::OcTree const& octree, const octomap::OcTreeKey& key)  {
    	// int depth = octree.getTreeDepth () ;
		// generate appropriate key_at_depth for queried depth
		octomap::OcTreeKey key_at_depth = key;
		octomap::OcTreeNode* curNode (octree.getRoot ());
		// int diff = tree_depth - depth;

		int tree_depth = octree.getTreeDepth () ;
		// follow nodes down to requested level (for 0 it's the last level)
		int i=(tree_depth-1);
		for (i; i>=0; --i) 
		{
			unsigned int pos = computeChildIdx(key_at_depth, i);
			if (octree.nodeChildExists(curNode, pos)) 
			{
				// cast needed: (nodes need to ensure it's the right pointer)
				curNode = octree.getNodeChild(curNode, pos);
			} 
			else 
			{
				// we expected a child but did not get it
				// is the current node a leaf already?
				if (! octree.nodeHasChildren(curNode) ) 
				{ // TODO similar check to nodeChildExists?
					return octree.getTreeDepth () - (i+1);
				} 
				else 
				{
					// it is not, search failed
					// This child is unknown
					std::ostringstream oss;
                    oss << "Failed to find depth, stopped at " << octree.getTreeDepth () - (i+1) << ", tree depth is " << octree.getTreeDepth() << ". @getNodeDepth_Octomap";  
					return -1;
				}
			}
		} // end for
		return octree.getTreeDepth () - (i+1);
	}

    bool findCellSize(octomap::OcTree const& octree, octomath::Vector3 const& target, double theoretical_size)
    {
        octomap::OcTree::leaf_iterator it = octree.begin_leafs();
        for (; it != octree.end_leafs(); ++it)
        {
            if(it.getCoordinate() == target)
            {
                ROS_WARN_STREAM("The center coordinates for " << target << " are " << it.getCoordinate() << ". It's depth is " << it.getDepth() << ", it's size is " << it.getSize()  );
                return true;
            }
            if( it.getCoordinate().distance(target) <= it.getSize())
            {
                ROS_WARN_STREAM(std::setprecision(10) << "The center coordinates for " << target << " are " << it.getCoordinate() << ". It's depth is " << it.getDepth() << ", it's size is " << it.getSize() );
                return true;
            }
        }
        return false;
    }

    // Other way to find the depth based on the search code of the octree
	int getNodeDepth_Octomap_EUROC (const octomap::OcTreeKey& key, 
	    octomap::OcTree const& octree)  
	{
	    int tree_depth = octree.getTreeDepth();
	    octomap::OcTreeNode* root = octree.getRoot();


	    if (root == NULL)
	        return -1;
	    int depth = tree_depth;
	    int depth_tracking = depth;

	    // generate appropriate key_at_depth for queried depth
	     octomap::OcTreeKey key_at_depth = key;
	    if (depth != tree_depth)
	        key_at_depth = octree.adjustKeyAtDepth(key, depth);
	    // ROS_WARN_STREAM("Depth "<<depth_tracking);

	    octomap::OcTreeNode* curNode (root);

	    unsigned int diff = tree_depth - depth;

	    // follow nodes down to requested level (for diff = 0 it's the last level)
	    for (unsigned i=(tree_depth-1); i>=diff; --i) {
	        unsigned int pos = computeChildIdx(key_at_depth, i);
	        if (octree.nodeChildExists(curNode, pos)) 
	        {
	            // cast needed: (nodes need to ensure it's the right pointer)
	            curNode = static_cast<octomap::OcTreeNode*>( octree.getNodeChild(curNode, pos) );
	            depth_tracking--;
	            // ROS_WARN_STREAM("/ Depth "<<depth_tracking);
	        } 
	        else 
	        {
	            // we expected a child but did not get it
	            // is the current node a leaf already?
	            if (!octree.nodeHasChildren(curNode)) 
	            {
	                // ROS_WARN_STREAM("Final depth "<<depth_tracking);
	                return 16-depth_tracking;
	            } 
	            else 
	            {
	                // it is not, search failed
	                // if(16-depth_tracking > 4)
	                //   ROS_ERROR_STREAM("Hit an unknown part of the map at depth " << 16-depth_tracking);
	                return -2;
	            }
	        }
	    } // end for
	          //ROS_WARN_STREAM("Final depth 2 "<<depth_tracking);
	    return 16-depth_tracking;
	}

	std::list<octomath::Vector3> lookForPointCenters(octomap::OcTree const& octree, octomath::Vector3 const& target)
	{
		std::list<octomath::Vector3> result;
		octomap::OcTree::leaf_iterator it = octree.begin_leafs();
        for (; it != octree.end_leafs(); ++it)
        {
        	
        	if(it.getCoordinate().distance(target) <= it.getSize())
        	{
        		// ROS_WARN_STREAM(it.getCoordinate() << " depth: " << it.getDepth() << ", size: " << it.getSize());
        		result.insert(result.begin(), it.getCoordinate());
        	}
        }
        return result;
	}

	std::list<Voxel> lookForPointCentersInVoxels(octomap::OcTree const& octree, octomath::Vector3 const& target)
	{
		std::list<Voxel> result;
		octomap::OcTree::leaf_iterator it = octree.begin_leafs();
        for (; it != octree.end_leafs(); ++it)
        {
        	
        	if(it.getCoordinate().distance(target) <= it.getSize())
        	{
        		// ROS_WARN_STREAM(it.getCoordinate() << " depth: " << it.getDepth() << ", size: " << it.getSize());
        		result.insert(   result.begin(), 
        			Voxel( it.getCoordinate(), 
        				it.getSize() )   );
        	}
        }
        return result;
	}
	void assess_depthGenerationAccuracy(octomap::OcTree const& octree)
	{
		int total = 0;
		int depth_fail = 0;
		int depth_fail_Euroc = 0;
		int depth_fail_Rainville = 0;
		int depth_fail_search = 0;
		octomap::OcTree::leaf_iterator it = octree.begin_leafs();
        for (; it != octree.end_leafs(); ++it)
        {
        	// int found_depth = getNodeDepth_Octomap(it.getKey(), octree);
        	// if(found_depth != it.getDepth())
        	// {
        	// 	depth_fail++;
        	// }
        	// int found_depth_EUROC = getNodeDepth_Octomap_EUROC(it.getKey(), octree);
        	// if(found_depth_EUROC != it.getDepth())
        	// {
        	// 	depth_fail_Euroc++;
        	// }
        	// double found_depth_Rainville = getNodeDepth_Rainville(&octree, it.getKey());
        	// if(found_depth_Rainville != it.getDepth())
        	// {
        	// 	depth_fail_Rainville++;
        	// }
        	
        	double found_depth_search = search(octree, it.getKey());
        	ASSERT_EQ(found_depth_search, it.getDepth()) << it.getCoordinate();
        	if(found_depth_search != it.getDepth())
        	{
        		depth_fail_search++;
        	}
        	total++;
        }
        // ROS_WARN_STREAM("Total leafs: "<< total);
        // ROS_WARN_STREAM("Euroc fail: "<< depth_fail);
        // ROS_WARN_STREAM("In use fail: "<< depth_fail_Euroc);
        // ROS_WARN_STREAM("Rainville fail: "<< depth_fail_Rainville);
        // ROS_WARN_STREAM("Search fail: "<< depth_fail_search);
        ASSERT_EQ(depth_fail, 0);
        ASSERT_EQ(depth_fail_Euroc, 0);
        ASSERT_EQ(depth_fail_Rainville, 0);
        ASSERT_EQ(depth_fail_search, 0);
	}

	TEST(DepthSizeTest, DepthAssessmentTest)
	{
		octomap::OcTree octree ("data/offShoreOil_1m.bt");
		assess_depthGenerationAccuracy(octree);
	}
	
	// TEST(DepthSizeTest, FindPointInfoTest)
	// {
	// 	octomap::OcTree octree ("data/offShoreOil_1m.bt");
	// 	octomath::Vector3 target (-5.400001, -2.600000, 0.600000);
	// 	lookForPointCenters(octree, target);
	// }

	TEST(DepthSizeTest, DepthSizeTest)
	{
		octomap::OcTree octree ("data/offShoreOil_1m.bt");
		double sidelength_lookup_table  [octree.getTreeDepth()];
	   	LazyThetaStarOctree::fillLookupTable(octree.getResolution(), octree.getTreeDepth(), sidelength_lookup_table); 
        octomath::Vector3 point_in_voxel (-5.400001, -2.600000, 0.600000);
        octomath::Vector3 correct_voxel_center (-5.2, -2.8, 0.4);
        double correct_size = 0.8;
        int correct_depth = 14;

        // As in lazy theta star
        // convert to key
        // octomap::OcTreeKey key = octree.coordToKey(point_in_voxel);
        octomap::OcTreeKey key;
        ASSERT_TRUE(octree.coordToKeyChecked(point_in_voxel, key)) << "Error in search: "<< point_in_voxel << " is out of OcTree bounds!";	    
        // find depth of cell
        // int found_depth = getNodeDepth_Octomap(key, octree);
        // EXPECT_EQ(found_depth, correct_depth);

        // int found_depth_EUROC = getNodeDepth_Octomap_EUROC(key, octree);
        // EXPECT_EQ(found_depth_EUROC, correct_depth);

        // double found_depth_Rainville = getNodeDepth_Rainville(&octree, key);
        // EXPECT_EQ(found_depth_Rainville, correct_depth);

        int found_depth_search = search(octree, key);
        EXPECT_EQ(found_depth_search, correct_depth);

        double found_length = findSideLenght(octree.getTreeDepth(), found_depth_search, sidelength_lookup_table); 
        EXPECT_EQ(found_length, correct_size);
        // get center coord of cell center at depth
        octomath::Vector3 cell_center = octree.keyToCoord(key, found_depth_search);
        ASSERT_EQ(cell_center, correct_voxel_center);
	}

	TEST(DepthSizeTest, DepthTest)
	{
		// Point found as error case during assessment of find depth for voxel centers
		octomap::OcTree octree ("data/offShoreOil_1m.bt");
        octomath::Vector3 voxel_center(-0.10000000149011612, -64.5, -0.89999997615814209);
        int correct_depth = 16;
        octomap::OcTreeKey key;
        ASSERT_TRUE(octree.coordToKeyChecked(voxel_center, key)) << "Error in search: "<< voxel_center << " is out of OcTree bounds!";	 
        double found_depth_search = search(octree, key);
        ASSERT_EQ(correct_depth, found_depth_search);
	}


	TEST(DepthSizeTest, hasLineOfSightTest)
	{
		// ARRANGE
		octomap::OcTree octree ("data/offShoreOil_1m.bt");
		octomath::Vector3 start = octomath::Vector3(-5.500001, -3.1, 0.5);
		octomath::Vector3 end = octomath::Vector3(-5.4, -2.6, 0.6);
		bool is_obstacle = false;
		// ACT
		// ROS_WARN_STREAM("The conversion being tested:");
		bool has_line_of_sight = hasLineOfSight(InputData( octree, start, end, 0) );
		// ASSERT
		// castRay & search
		octomath::Vector3 return_value;
		// ROS_WARN_STREAM("Figuring out the expected value castRay & search ");
		octomath::Vector3 direction = end - start;
		bool occupied_cell_was_hit = octree.castRay(start, direction, return_value, false, direction.norm());
		if(occupied_cell_was_hit)
		{
			octomap::OcTreeNode* search_result = octree.search(return_value);
			if (search_result == NULL)
			{
				// ROS_WARN_STREAM("This is unknown space");
			}
			else
			{
				// ROS_WARN_STREAM("This is known obstacle space. ");
				is_obstacle = true;
				// EXPECT_FALSE(has_line_of_sight);
			}
		}
		else
		{
			// ROS_WARN_STREAM("This is free space");
		}
		ASSERT_FALSE(is_obstacle);
		ASSERT_TRUE(has_line_of_sight);
	}


	TEST(DepthSizeTest, KeyTest)
	{
		octomap::OcTree octree ("data/offShoreOil_1m.bt");
		double sidelength_lookup_table  [octree.getTreeDepth()];
	   	LazyThetaStarOctree::fillLookupTable(octree.getResolution(), octree.getTreeDepth(), sidelength_lookup_table); 
		octomath::Vector3 origin (-5.4, -2.6, 0.6);
		std::shared_ptr<octomath::Vector3> a = std::make_shared<octomath::Vector3>(origin);
		std::shared_ptr<octomath::Vector3> b = std::make_shared<octomath::Vector3>(origin);
		
		octomap::OcTreeKey key = octree.coordToKey(*a);
		octomath::Vector3 cell_center = octree.keyToCoord(key, 14);

		double side_length_m = -1;
		octomap::OcTreeKey key_m = updatePointerToCellCenterAndFindSize(b, octree, side_length_m, sidelength_lookup_table);

        ASSERT_EQ(key, key_m);
        ASSERT_EQ(cell_center, *b);
        // ROS_WARN_STREAM("Center of " << origin << " is " << *b); //Center of (-5.4 -2.6 0.6) is (-5.2 -2.8 0.4)

	}

	TEST(DepthSizeTest, ReverseNormalizedLineOfSight)
	{
		ros::Publisher marker_pub;
		double cell_size = 0;
		double safety_margin = 0;
	    octomath::Vector3 p1(0.7, 3.1, 1.3); 
	    octomath::Vector3 p2(0.7, 2.9, 1.3); 
	    std::shared_ptr<octomath::Vector3> p1_ptr = std::make_shared<octomath::Vector3>(p1);
	    std::shared_ptr<octomath::Vector3> p2_ptr = std::make_shared<octomath::Vector3>(p2);
		octomap::OcTree octree ("data/run_2.bt");
		double sidelength_lookup_table  [octree.getTreeDepth()];
	   	LazyThetaStarOctree::fillLookupTable(octree.getResolution(), octree.getTreeDepth(), sidelength_lookup_table); 
		bool line_of_sight_A = normalizeToVisibleEndCenter(octree, p1_ptr, p2_ptr, cell_size, safety_margin, marker_pub, sidelength_lookup_table);
		bool line_of_sight_B = normalizeToVisibleEndCenter(octree, p2_ptr, p1_ptr, cell_size, safety_margin, marker_pub, sidelength_lookup_table);
		ASSERT_EQ(line_of_sight_B, line_of_sight_A);
	}

	
	TEST(WorkInProgressTest, ReverseLineOfSight)
	{
	    octomath::Vector3 p1(0.7, 3.1, 1.3); 
	    octomath::Vector3 p2(0.7, 2.9, 1.3); 
		octomap::OcTree octree ("data/run_2.bt");

		auto res_node = octree.search(p1);
		// if(res_node == NULL)
		// {
		// 	ROS_WARN_STREAM("[1] The coordinates " << p1 << " do not correspond to a node in this octree  ==> this neighbor is unknown");
		// }
		// else
		// {
			ASSERT_FALSE(octree.isNodeOccupied(res_node));
		// }
		res_node = octree.search(p2);
		// if(res_node == NULL)
		// {
		// 	ROS_WARN_STREAM("[1] The coordinates " << p2 << " do not correspond to a node in this octree  ==> this neighbor is unknown");
		// }
		// else
		// {
			ASSERT_TRUE(octree.isNodeOccupied(res_node));
		// }


		bool line_of_sight_A = hasLineOfSight( InputData( octree, p1, p2, 0) );
		bool line_of_sight_B = hasLineOfSight( InputData( octree, p2, p1, 0) );
		ASSERT_FALSE(line_of_sight_B);
		ASSERT_FALSE(line_of_sight_A);
		ASSERT_EQ(line_of_sight_B, line_of_sight_A);
	}
   
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}