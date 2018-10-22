// #include <neighbors_temp.h>
#include <gtest/gtest.h>
#include <octomap/OcTreeNode.h>
#include <ltStar_temp.h>

namespace LazyThetaStarOctree{

	// TEST(OctreeNeighborTest, GenerateData_FindCells)
	// {
	// 	// ARRANGE
	// 	ros::Time::init();
	// 	octomap::OcTree octree ("data/circle_1m.bt");
 //        findDifferentSizeCells_ptr_3D(octree);

 //        // findDifferentSizeCells_ptr_3D(octree, false);


	// 	octomap::OcTree octree_2 ("data/double_circle_1m.bt");
 //        findDifferentSizeCells_ptr_3D(octree_2);


	// 	octomap::OcTree octree_3 ("data/s_1m.bt");
 //        findDifferentSizeCells_ptr_3D(octree_3);

	// 	octomap::OcTree octree_4 ("data/offShoreOil_1m.bt");
 //        findDifferentSizeCells_ptr_3D(octree_4);


	// 	octomap::OcTree octree_5 ("data/offShoreOil_4obstacles_1m.bt");
 //        findDifferentSizeCells_ptr_3D(octree_5);
	// }

	// TEST(OctreeNeighborTest, GenerateData_CalculateVolume)
	// {
	// 	ros::Time::init();
	// 	octomap::OcTree octree ("data/euroc_demo_run_1.bt");
 //        double knownVolume = calculateCellSpace(octree);
 //        ROS_WARN_STREAM("The volume of known voxels is " << knownVolume);
	// }

	// TEST(OctreeNeighborTest, GenerateData_FindFreeLine_Test)
	// {
	// 	ros::Time::init();
	// 	octomap::OcTree octree ("data/circle_1m.bt");
	// 	octomath::Vector3 origin (0, 0, 0.6);
	// 	octomath::Vector3 direction (1, 0, 0);
	// 	octomath::Vector3 end;

	// 	octomap::OcTreeNode* originNode = octree.search(origin);
	// 	ASSERT_TRUE(originNode);
	// 	ASSERT_FALSE(octree.isNodeOccupied(originNode));

	// 	bool isOccupied = octree.castRay(origin, direction, end);
 //        ASSERT_TRUE(isOccupied);  // true if an occupied cell was hit
	// 	ROS_WARN_STREAM("Occupied point " << end);
 //        isOccupied = octree.castRay(origin, direction, end, false, 1);
 //        ASSERT_FALSE(isOccupied); // false if the maximum range or octree bounds are reached, or if an unknown node was hit.
	// }
	
    void writeToFileCellDistribution(std::string const& dataName, ResultSet const& statistical_data)
    {
        std::ofstream size_vs_cellCount;
        size_vs_cellCount.open ("/home/mfaria/Margarida/20170802_lazyThetaStar/experimental data/size_vs_cellCount" + dataName + ".csv", std::ofstream::out | std::ofstream::app);
        for(std::map<double, int>::const_iterator it = statistical_data.cellVoxelDistribution.cbegin();
            it != statistical_data.cellVoxelDistribution.end();
            ++it)
        {
            size_vs_cellCount << it->first << ", " << it->second << std::endl;
        }
        size_vs_cellCount.close();
    }

    void writeToFileCellDistribution_OneLine(std::string const& dataName, ResultSet const& statistical_data)
    {
        std::ofstream iterations_cellDistribution;
        iterations_cellDistribution.open ("/home/mfaria/Margarida/20170802_lazyThetaStar/experimental data/iterations_cellDistribution" + dataName + ".csv", std::ofstream::out | std::ofstream::app);
        iterations_cellDistribution << statistical_data.iterations_used;
        for(std::map<double, int>::const_iterator it = statistical_data.cellVoxelDistribution.cbegin();
            it != statistical_data.cellVoxelDistribution.end();
            ++it)
        {
            iterations_cellDistribution << ", " << it->first << ", " << it->second;
        }
        iterations_cellDistribution << std::endl;
        iterations_cellDistribution.close();
    }

    bool findInitialAndFinalPoints(octomap::OcTree const& octree, octomath::Vector3 const& disc_initial, octomath::Vector3 const& disc_final)
    {
        if( disc_initial.z() < 0.4  || disc_initial.z() > 0.6 ) return false;
        // Initial node is not occupied
        octomap::OcTreeNode* originNode = octree.search(disc_initial);
        // ASSERT_TRUE(originNode);
        if(!originNode) return false;
        if(octree.isNodeOccupied(originNode)) return false;
        // ASSERT_FALSE(octree.isNodeOccupied(originNode));
        // Final node is not occupied
        octomap::OcTreeNode* finalNode = octree.search(disc_final);
        // ASSERT_TRUE(finalNode);
        if(!finalNode) return false;
        // ASSERT_FALSE(octree.isNodeOccupied(finalNode));
        if(octree.isNodeOccupied(finalNode)) return false;
        // The path is clear from start to finish
        octomath::Vector3 direction = (disc_final - disc_initial).normalized();
        octomath::Vector3 end;
        bool isOccupied = octree.castRay(disc_initial, direction, end, false, weightedDistance(disc_initial, disc_final));
        // ASSERT_FALSE(isOccupied); // false if the maximum range or octree bounds are reached, or if an unknown node was hit.
        if(isOccupied) return false;
        return true;
    }




    void findObstaclePath(octomap::OcTree const& octree, std::string dataName, double line_length, int reasonable_iterations_limit, int max_leafs = 10000)
    {
        ASSERT_EQ( 0, ThetaStarNode::OustandingObjects());
        int count = 0;
        int count_NoSolution_A = 0;
        int count_correct = 0;
        int count_reasonable = 0;
        int reasonable_steps_limit = 50;
        bool found = false;
        octomap::OcTree::leaf_iterator it = octree.begin_leafs();
        for (; it != octree.end_leafs(); ++it)
        {
            int memory_leak_initial_count = ThetaStarNode::OustandingObjects();
            octomath::Vector3 disc_initial = it.getCoordinate();
            octomath::Vector3 disc_final  ( disc_initial.x()+line_length, disc_initial.y(), disc_initial.z() );
            if(disc_initial.z() < 0.4 || disc_initial.z() > 0.6) continue;
            // Initial node is not occupied
            octomap::OcTreeNode* originNode = octree.search(disc_initial);
            if(!originNode) continue;
            if(octree.isNodeOccupied(originNode)) continue;
            // Final node is not occupied
            octomap::OcTreeNode* finalNode = octree.search(disc_final);
            if(!finalNode) continue;
            if(octree.isNodeOccupied(finalNode)) continue;
            // The path is not clear from start to finish
            octomath::Vector3 direction (1, 0, 0);
            octomath::Vector3 end;
            bool isOccupied = octree.castRay(disc_initial, direction, end, false, weightedDistance(disc_initial, disc_final));
            // false if the maximum range or octree bounds are reached, or if an unknown node was hit.
            if(!isOccupied) continue;

            // std::cout << "Starting lazy theta from " << disc_initial << " to " << disc_final << std::endl;
            ResultSet statistical_data;
            std::list<octomath::Vector3> resulting_path = lazyThetaStar_(octree, disc_initial, disc_final, statistical_data, reasonable_iterations_limit);

            count++;

            if(resulting_path.size() == 0)
            {
                // LIMITATIONS A -> to graph --> No solution found in 200 steps
                // ROS_ERROR_STREAM("No solution found from  " << disc_initial << " to  " << disc_final);
                count_NoSolution_A++;
            }
            else 
            {
                ASSERT_LT(   ( *(resulting_path.begin()) ).distance( disc_initial ),  octree.getResolution()   );
                ASSERT_LT(   resulting_path.back().distance( disc_final ),  octree.getResolution()   );
                found = true;
                
            }
            EXPECT_EQ( memory_leak_initial_count, ThetaStarNode::OustandingObjects()) << "From  " << disc_initial << " to  " << disc_final;


            if(found)
            {
                ROS_WARN_STREAM("Found a test case from " << disc_initial << " to " << disc_final << " after going through " << count << " paths." );
                writeToFileCellDistribution_OneLine(dataName+"_obstacles", statistical_data);

            }
            if( count >= max_leafs)
            {
                ROS_WARN_STREAM("Reached " << count << " analyzed, breaking out.");
                break;
            }
        }
        // ROS_WARN_STREAM("Overall has " << count_under10 << " under reasonable_iterations_limit; " << count_80 << " around 80 and " << count_over80 << " over reasonable_iterations_limit.");
        // ROS_WARN_STREAM("From " << max_leafs);
        // ROS_WARN_STREAM(" :) " << count_correct + count_reasonable << " of which " << count_correct << " correct and " << count_reasonable << " reasonable.");
        ROS_WARN_STREAM(" :( " << count_NoSolution_A );
    }

	void gatherStats_StraightLinesForwardNoObstacles(octomap::OcTree const& octree, std::string dataName, 
        double line_length, 
        int reasonable_iterations_limit, 
        int correct_iterations_limit,
        int max_leafs = 100000, 
        int maximum_iterations_limit=100 )
    {
        ASSERT_EQ( 0, ThetaStarNode::OustandingObjects());
        int count = 0;
        int count_NoSolution_A = 0;
        int count_TooManyIterations_B = 0;
        int count_JaggedPath_C = 0;
        int count_LongPath_D = 0;
        int count_correct = 0;
        int count_correct_path_size = 0;
        int count_invalid_path = 0;
        int count_wtf = 0;
        int count_incorrect = 0;
        int count_reasonable = 0;
        int reasonable_steps_limit = 10;
        int correct_steps_limit = 2;
        int count_unknown = 0;

        ResultSet statistical_data;
        bool jagged, tooManyIterations, longPath, noSolution, is_valid_waypoints;
        jagged = tooManyIterations = longPath = noSolution = false;
        int l = 0;
        octomap::OcTree::leaf_iterator it = octree.begin_leafs();
        for (; it != octree.end_leafs() && l < max_leafs; ++it)
        {
            is_valid_waypoints = true;
            int memory_leak_initial_count = ThetaStarNode::OustandingObjects();
            octomath::Vector3 disc_initial = it.getCoordinate();
            octomath::Vector3 disc_final  ( disc_initial.x()+line_length, disc_initial.y(), disc_initial.z() );
            if(findInitialAndFinalPoints(octree, disc_initial, disc_final) && hasLineOfSight(octree, disc_initial, disc_final))
            {
                // std::cout << "Starting lazy theta from " << disc_initial << " to " << disc_final << std::endl;
                try
                {

                    std::list<octomath::Vector3> resulting_path = lazyThetaStar_(octree, disc_initial, disc_final, statistical_data, maximum_iterations_limit);
                    l++;
                    count++;
                    tooManyIterations = statistical_data.iterations_used > correct_iterations_limit;
                    if(tooManyIterations)
                    {
                        count_TooManyIterations_B++;                   
                    }
                    if(resulting_path.size() == 0)                      
                    {
                        ROS_WARN_STREAM("No solution From  " << disc_initial << " to  " << disc_final);
                        noSolution = true;
                        count_NoSolution_A++;
                    }
                    else if(resulting_path.size() >= correct_steps_limit)                 
                    {
                        double cell_size_goal = -1;
                        octomath::Vector3 cell_center_coordinates_goal = disc_final;
                        updateToCellCenterAndFindSize( cell_center_coordinates_goal, octree, cell_size_goal);
                        if(     resulting_path.back().distance( cell_center_coordinates_goal ) >  cell_size_goal   )
                        {
                            ROS_WARN_STREAM("Distance from goal point to goal voxel center is too big");
                            ROS_WARN_STREAM(resulting_path.back() << ".distance(" << cell_center_coordinates_goal << " ) <=  " << cell_size_goal);
                            ROS_WARN_STREAM(resulting_path.back().distance(cell_center_coordinates_goal ) << " <=  " << cell_size_goal);
                            count_invalid_path++;
                            continue;
                        }
                        double cell_size_start = -1;
                        octomath::Vector3 cell_center_coordinates_start = disc_initial;
                        updateToCellCenterAndFindSize( cell_center_coordinates_start, octree, cell_size_start);
                        if(      resulting_path.begin()->distance( cell_center_coordinates_start ) >  cell_size_start   )
                        {
                            ROS_WARN_STREAM("Distance from goal point to start voxel center is too big");
                            ROS_WARN_STREAM(*(resulting_path.begin()) << ".distance(" << cell_center_coordinates_start << " ) <=  " << cell_size_start);
                            ROS_WARN_STREAM(resulting_path.begin()->distance(cell_center_coordinates_start ) << " <=  " << cell_size_start);
                            count_invalid_path++;
                            continue;
                        }
                        if( resulting_path.size() == correct_steps_limit )
                        {
                            count_correct_path_size++;
                        }
                        else
                        {
                            // LIMITATIONS C -> to graph --> The path goes up/down/righ/left when it could go straight
                            longPath = true;
                            count_LongPath_D++;
                            std::list<octomath::Vector3>::iterator it = resulting_path.begin();
                            octomath::Vector3 previous = *it;
                            ++it;
                            // Check that there are no redundant parts in the path
                            while(it != resulting_path.end())
                            {
                                bool dimensions_y_or_z_change = (previous.y() != it->y()) || (previous.z() != it->z());
                                if(!dimensions_y_or_z_change)
                                {
                                    ROS_WARN_STREAM(disc_initial << " to " << disc_final);
                                    ROS_WARN_STREAM("Dimensions Y or Z aren't changing => redundant waypoints");
                                    ROS_WARN_STREAM(   "("<<previous.y() << " != " << it->y() << ") || (" << previous.z() << " != " << it->z() << ")"   );
                                    count_invalid_path++;
                                    is_valid_waypoints = false;
                                }
                                // Find cell size
                                octomath::Vector3 cell_center = *it;
                                double cell_size = -1;
                                updateToCellCenterAndFindSize(cell_center, octree, cell_size);
                                // // X
                                double x_deviation = std::abs(   it->y() - disc_final.y()   );
                                // EXPECT_LT( x_deviation, cell_size );  
                                // // Y
                                double y_deviation = std::abs(   it->z() - disc_final.z()   );
                                // EXPECT_LT( y_deviation, cell_size );

                                if(x_deviation >= cell_size || y_deviation >= cell_size)
                                {
                                    jagged =true;
                                    count_JaggedPath_C++;
                                    break;
                                }
                                previous = *it;
                                ++it;
                            }
                        }
                    }   
                    else                                       
                    {
                        ROS_ERROR_STREAM(" what is this? path size is " << resulting_path.size());
                        count_wtf++;
                    }             
                    if(resulting_path.size() == correct_steps_limit && is_valid_waypoints && !tooManyIterations)   
                    {
                        count_correct++;
                    }

                    if(resulting_path.size() > correct_steps_limit && resulting_path.size() < reasonable_steps_limit && statistical_data.iterations_used < reasonable_iterations_limit)
                    {
                        count_reasonable++;
                    }
                    // TODO!!! EXPECT_EQ( memory_leak_initial_count, ThetaStarNode::OustandingObjects()) << "From  " << disc_initial << " to  " << disc_final;
                    // ROS_WARN_STREAM("Writing to file from  " << disc_initial << " to  " << disc_final);
                    writeToFileCellDistribution_OneLine(dataName, statistical_data);
                }
                catch(std::out_of_range e)
                {
                    count_unknown++;
                }

            }

        }
        int count_incorrect_solutions = count - count_correct;
        ROS_WARN_STREAM("count_NoSolution_A " << count_NoSolution_A);
        ROS_WARN_STREAM("count_correct_path_size " << count_correct_path_size);
        ROS_WARN_STREAM("count_LongPath_D " << count_LongPath_D);
        ROS_WARN_STREAM("count_wtf " << count_wtf);
        ROS_WARN_STREAM("count_TooManyIterations_B " << count_TooManyIterations_B);
        ROS_WARN_STREAM("count_invalid_path " << count_invalid_path);
        ROS_WARN_STREAM("count_unknown " << count_unknown);
        ROS_WARN_STREAM("From " << count);
        ROS_WARN_STREAM(" :) " << count_correct);
        ROS_WARN_STREAM(" :( " << count_incorrect_solutions << " ===> no solution: " << count_NoSolution_A << "; TooManyIterations_B: " << count_TooManyIterations_B << "; JaggedPath_C: " << count_JaggedPath_C << "; LongPath_D:" << count_LongPath_D << "; Invalid path: " << count_invalid_path);
        ROS_WARN_STREAM(" :| " << count_reasonable << " ===> reasonable solutions among the " << count_incorrect_solutions << " incorrect ones");

        EXPECT_EQ(count_invalid_path, 0);
        EXPECT_GE(count_correct_path_size, count_correct);  
        EXPECT_GE(count_NoSolution_A + count_TooManyIterations_B + count_LongPath_D, count_incorrect_solutions);
    }

    /*TEST(OctreeNeighborTest, GenerateData_FindFreeStraightLine_1m)
    {
        ros::Time::init();
        int line_length_m = 1;
        int correct_iterations_limit = 7;         
        int reasonable_iterations_limit = 10;
        int maximum_iterations_limit = 500;
        int maximum_voxels_analyzed = 100000;
        octomap::OcTree octree_5 ("data/offShoreOil_1m.bt");
        gatherStats_StraightLinesForwardNoObstacles(octree_5, "offShoreOil_2Obst_", 
            line_length_m, 
            reasonable_iterations_limit, 
            correct_iterations_limit,
            maximum_voxels_analyzed, 
            maximum_iterations_limit);
    }*/

    TEST(OctreeNeighborTest, GenerateData_FindFreeStraightLine_10m)
    {
        ros::Time::init();
        octomap::OcTree octree_5 ("data/offShoreOil_4obstacles_1m.bt");
        int line_length_m = 10;
        int regularGrid_iterations = line_length_m/octree_5.getResolution(); 
        int maximum_iterations_limit = 500;
        int maximum_voxels_analyzed = 100000;
        gatherStats_StraightLinesForwardNoObstacles(octree_5, "offShoreOil_4obstacles_10m_", 
            line_length_m, 
            regularGrid_iterations*2, 
            regularGrid_iterations,
            maximum_voxels_analyzed, 
            maximum_iterations_limit);
    }

    /*TEST(OctreeNeighborTest, GenerateData_FindFreeStraightLine_10m)
    {
        ros::Time::init();
        int line_length_m = 10;
        int correct_iterations_limit = 51; 
        int reasonable_iterations_limit = 100;
        int maximum_iterations_limit = 500;
        int maximum_voxels_analyzed = 100000;
        octomap::OcTree octree_5 ("data/offShoreOil_1m.bt");
        gatherStats_StraightLinesForwardNoObstacles(octree_5, "offShoreOil_2Obst_", 
            line_length_m, 
            reasonable_iterations_limit, 
            correct_iterations_limit,
            maximum_voxels_analyzed, 
            maximum_iterations_limit);
    }*/

    /*TEST(LazyThetaStarTests, LazyThetaStar_GeneratePointFor3dVisualization_Neighbors_Test) // TODO move to analyze tests
    {
        // octomath::Vector3 center (-3.600000, -4.400000, 0.400000);
        std::ostringstream oss;
        std::ostringstream oss_x;
        std::ostringstream oss_y;
        std::ostringstream oss_z;
        oss_x << std::setprecision(10);
        oss_y << std::setprecision(10);
        oss_z << std::setprecision(10);
        std::ostringstream oss_x_A;
        std::ostringstream oss_y_A;
        std::ostringstream oss_z_A;
        oss_x_A << std::setprecision(10);
        oss_y_A << std::setprecision(10);
        oss_z_A << std::setprecision(10);
        std::ostringstream oss_x_UNKNOWN;
        std::ostringstream oss_y_UNKNOWN;
        std::ostringstream oss_z_UNKNOWN;
        oss_x_UNKNOWN << std::setprecision(10);
        oss_y_UNKNOWN << std::setprecision(10);
        oss_z_UNKNOWN << std::setprecision(10);
        octomath::Vector3 afterProcessing;
        std::shared_ptr<octomath::Vector3> afterProcessing_ptr = std::make_shared<octomath::Vector3>(afterProcessing);

        double cell_size = 0.4;
        octomap::OcTree octree ("data/offShoreOil_1m.bt");
        octomath::Vector3 s37  (-3.900000, -4.900000,  0.500000);
        octomath::Vector3 s223 (-3.600000, -4.400000,  0.4000006);
        octomath::Vector3 s39  (-3.400000, -5.000000,  0.600000);
        octomath::Vector3 s40  (-3.000000, -5.000000, 0.60000);
        octomath::Vector3 s41  (-2.600000, -5.000000, 0.600000);
        octomath::Vector3 s220 (-2.000000, -5.200000, 0.400000);

        octomath::Vector3 center = s220;
        oss << "Center: " << center;
        std::shared_ptr<octomath::Vector3> center_ptr = std::make_shared<octomath::Vector3>(center);
        updatePointerToCellCenterAndFindSize(center_ptr, octree, cell_size);
        oss << " => " << *center_ptr << " for size " << cell_size << std::endl;
        ROS_WARN_STREAM(oss.str());

        std::unordered_set<std::shared_ptr<octomath::Vector3>> neighbors;
        generateNeighbors_pointers(neighbors, *center_ptr, cell_size, 0.2);
        ROS_WARN_STREAM("Amount of neighbors " << neighbors.size() << " for size " << cell_size);
        for(std::shared_ptr<octomath::Vector3> n_coordinates : neighbors)
        {
            oss_x << n_coordinates->x() << ", ";
            oss_y << n_coordinates->y() << ", ";
            oss_z << n_coordinates->z() << ", ";
            try
            {
                updatePointerToCellCenterAndFindSize(n_coordinates, octree, cell_size);
                // if(cell_size > 0.2)
                // {
                //  ROS_WARN_STREAM ("cell_size: " << cell_size);   
                // } 
                oss_x_A << n_coordinates->x() << ", ";
                oss_y_A << n_coordinates->y() << ", ";
                oss_z_A << n_coordinates->z() << ", ";
            }
            catch(const std::out_of_range& oor)
            {
                ROS_WARN_STREAM (*n_coordinates << " is unknown ");
                oss_x_UNKNOWN << n_coordinates->x() << ", ";
                oss_y_UNKNOWN << n_coordinates->y() << ", ";
                oss_z_UNKNOWN << n_coordinates->z() << ", ";
            }
        }
        ROS_WARN_STREAM("X = ( " << oss_x.str() << ")");
        ROS_WARN_STREAM("Y = ( " << oss_y.str() << ")");
        ROS_WARN_STREAM("Z = ( " << oss_z.str() << ")");
        ROS_WARN_STREAM("X_A = ( " << oss_x_A.str() << ")");
        ROS_WARN_STREAM("Y_A = ( " << oss_y_A.str() << ")");
        ROS_WARN_STREAM("Z_A = ( " << oss_z_A.str() << ")");
        ROS_WARN_STREAM("X_UNKNOWN = ( " << oss_x_UNKNOWN.str() << ")");
        ROS_WARN_STREAM("Y_UNKNOWN = ( " << oss_y_UNKNOWN.str() << ")");
        ROS_WARN_STREAM("Z_UNKNOWN = ( " << oss_z_UNKNOWN.str() << ")");

        octomath::Vector3 candidate (-1.5, -4.9, 0.5); // (-1.2 -5.2 0.4)
        oss << "Candidate: " << candidate;
        std::shared_ptr<octomath::Vector3> candidate_ptr = std::make_shared<octomath::Vector3>(candidate);
        updatePointerToCellCenterAndFindSize(candidate_ptr, octree, cell_size);
        oss << " => " << *candidate_ptr << " for size " << cell_size << std::endl;
        ROS_WARN_STREAM(oss.str());
    }*/

    /*TEST(OctreeNeighborTest, VisualizePath_1m)
    {
        ros::Time::init();
        // (-6.5 -3.0999999046325684 0.5) and (-5.5 -3.0999999046325684 0.5)
        octomap::OcTree octree ("data/offShoreOil_1m.bt");
        octomath::Vector3 disc_initial(-6.5, -3.0999999046325684, 0.5);
        octomath::Vector3 disc_final  (-5.5, -3.0999999046325684, 0.5);
        // testStraightLinesForwardNoObstacles(octree, disc_initial, disc_final);
        // Initial node is not occupied
        octomap::OcTreeNode* originNode = octree.search(disc_initial);
        ASSERT_TRUE(originNode);
        ASSERT_FALSE(octree.isNodeOccupied(originNode));
        // Final node is not occupied
        octomap::OcTreeNode* finalNode = octree.search(disc_final);
        ASSERT_TRUE(finalNode);
        ASSERT_FALSE(octree.isNodeOccupied(finalNode));
        // The path is clear from start to finish
        octomath::Vector3 direction (1, 0, 0);
        octomath::Vector3 end;
        bool isOccupied = octree.castRay(disc_initial, direction, end, false, weightedDistance(disc_initial, disc_final));
        ASSERT_FALSE(isOccupied); // false if the maximum range or octree bounds are reached, or if an unknown node was hit.

        ResultSet statistical_data;
        std::list<octomath::Vector3> resulting_path = lazyThetaStar_(octree, disc_initial, disc_final, statistical_data, 55, true);
        ASSERT_EQ(resulting_path.size(), 3);
        for(octomath::Vector3 w : resulting_path)
        {
            ROS_WARN_STREAM(w);
        }
        ASSERT_LT(   ( *(resulting_path.begin()) ).distance( disc_initial ),  octree.getResolution()   );
        ASSERT_LT(   resulting_path.back().distance( disc_final ),  octree.getResolution()   );
        ASSERT_EQ(0, ThetaStarNode::OustandingObjects());
        
    }*/


    // TEST(OctreeNeighborTest, GenerateData_FindObstaclePath_1m)
    // {
    //     ros::Time::init();

    //     octomap::OcTree octree_5 ("data/offShoreOil_1m.bt");
    //     findObstaclePath(octree_5, "offShoreOil_2Obst", 1, 100, 1000);
    // }
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}