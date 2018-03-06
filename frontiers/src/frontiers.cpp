#include <frontiers.h>

namespace Frontiers{

    bool processFrontiersRequest(octomap::OcTree& octree, frontiers_msgs::FrontierRequest& request, frontiers_msgs::FrontierReply& reply)
    {
    	reply.header.seq = request.header.seq + 1;
    	reply.request_id = request.header.seq;
    	reply.header.frame_id = request.header.frame_id;
    	octomath::Vector3  max = octomath::Vector3(request.max.x, request.max.y, request.max.z);
    	octomath::Vector3  min = octomath::Vector3(request.min.x, request.min.y, request.min.z);
        int frontiers_count = 0;
    // }

    // ResultSet GridBenchmark::findFrontierCells(octomath::Vector3  max, octomath::Vector3  min, Directions dimensions,
    //             AlgorithmObject & algorithmSpecification) 
    //     {
        // octomath::Vector3 size = max - min;
        // double area = (size.x()*size.y()*size.z());
        // std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

        // === Task ===
        const std::array<octomath::Vector3, 6> rayDirections ({
                octomath::Vector3(1, 0, 0), // FRONT
                octomath::Vector3(0, 1, 0), // LEFT
                octomath::Vector3(0, -1, 0), // RIGHT
                octomath::Vector3(-1, 0, 0), // BACKWARDS
                octomath::Vector3(0, 0, -1), // UP
                octomath::Vector3(0, 0, 1), // DOWN
            });
        // frontierCells.clear();
        // std::list<Voxel> frontierCells;
        // z_levels.clear();
        // float z_lvl;
        // float x_min = 0;
        // float x_max = 0;
        // float y_min = 0;
        // float y_max = 0;


        bool hasUnExploredNeighbors;
        // int gridIterations = 0;
        // float frontierArea = 0;
        // double total_area = 0;
        // double coverage = 0;
        octomath::Vector3 grid_coordinates_curr, grid_coordinates_toTest;
        int i;

        float z_max = max.z();
        float z_min = min.z();

        Voxel currentVoxel;
        // algorithmSpecification.iteratorInit(min, max);
        // setBoundingBox(min, max);
        octomap::OcTreeKey bbxMinKey, bbxMaxKey;
        if(!octree.coordToKeyChecked(min, bbxMinKey) || !octree.coordToKeyChecked(max, bbxMaxKey))
        {
            ROS_ERROR_STREAM("Problems with the octree");
            reply.success = false;
        return reply.success;
        }
        octomap::OcTree::leaf_bbx_iterator it = octree.begin_leafs_bbx(bbxMinKey,bbxMaxKey);

        //ROS_WARN_STREAM("Bounding box bbx [ " << min << " - " << max << "]" );


        octomath::Vector3 bbxMin (currentVoxel.x, currentVoxel.y, currentVoxel.z);
        octomath::Vector3 bbxMax (currentVoxel.x, currentVoxel.y, currentVoxel.z);

        // while(!algorithmSpecification.iteratorEndReached())
        while( !(it == octree.end_leafs_bbx()) && frontiers_count < request.frontier_amount)
        {
            // algorithmSpecification.getCurrentCell(currentVoxel);
            octomath::Vector3 coord = it.getCoordinate();
            Voxel cell (coord.x(), coord.y(), coord.z(), it.getSize());
            //ROS_WARN_STREAM("At cell " << currentVoxel);


            // coverage = algorithmSpecification.calculateCoverage(currentVoxel.size, dimensions);
            grid_coordinates_curr = octomath::Vector3(currentVoxel.x, currentVoxel.y, currentVoxel.z);
            //ROS_WARN_STREAM("For " << grid_coordinates_curr << " occupied? " << isOccupied(grid_coordinates_curr) 
            //    << " explored? " << isExplored(grid_coordinates_curr));
            if( isExplored(grid_coordinates_curr, octree)
                && !isOccupied(grid_coordinates_curr, octree) ) 
            {
                hasUnExploredNeighbors = false;
                //ROS_WARN_STREAM(grid_coordinates_curr << "Level 1");
                for(i = 0; i < d3; i++)
                { 
                    octomath::Vector3 direction = rayDirections[i];
                    // grid_coordinates_toTest = algorithmSpecification.calculateNeighbor(grid_coordinates_curr, direction, currentVoxel.size);
                    grid_coordinates_toTest = grid_coordinates_curr + (direction * currentVoxel.size);
                    if(!isOccupied(grid_coordinates_toTest, octree))
                    {
                        hasUnExploredNeighbors = !isExplored(grid_coordinates_toTest, octree) || hasUnExploredNeighbors;
                    }
                }
                if(hasUnExploredNeighbors)
                {
                    // frontierCells.emplace(frontierCells.begin(), currentVoxel.x, currentVoxel.y, currentVoxel.z, currentVoxel.size);
                    reply.frontiers[frontiers_count].xyz_m.x = currentVoxel.x;
                    reply.frontiers[frontiers_count].xyz_m.y = currentVoxel.y;
                    reply.frontiers[frontiers_count].xyz_m.z = currentVoxel.z;
                    reply.frontiers[frontiers_count].size = currentVoxel.size;
                    frontiers_count++;
                    // frontierArea += coverage;
                    // z_levels.emplace( grid_coordi    reply.frontiers = {};nates_curr.z());
                }
            }
            // gridIterations++;
            // total_area+=coverage;

            // x_max = std::max(grid_coordinates_curr.x(), x_max);
            // x_min = std::min(grid_coordinates_curr.x(), x_min);
            // y_max = std::max(grid_coordinates_curr.y(), y_max);
            // y_min = std::min(grid_coordinates_curr.y(), y_min);
            // max = octomath::Vector3 (x_max, y_max, 1);
            // min = octomath::Vector3 (x_min, y_min, 1);


            // bbxMax = octomath::Vector3 (x_max, y_max, 1);
            // bbxMin = octomath::Vector3 (x_min, y_min, 1);



            // algorithmSpecification.iteratorNext();
            it++;
        }

        // === TIME ===
        // std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

        // std::string algoName = ResultSecretary::translateAlgorithmName(algorithmSpecification.algorithmEnum);

        // ROS_WARN_STREAM("Algorithm "<<algoName);
        // ROS_WARN_STREAM("Explored boundaries "<<bbxMin<<" - "<<bbxMax);
        // ROS_WARN_STREAM("Iterated over "<<gridIterations<<" - "<< total_area <<"m found "<<frontierCells.size()
        //     <<" frontier cells that cover "<<frontierArea
        //     <<" m. With "<<z_levels.size()<<" z levels. Frontiers found in "<<time_span.count()<<" seconds.");

        

        //storage.storeAlgorithmResults(scenario, algorithmSpecification.algorithmEnum, result);
        // return ResultSet (gridIterations, total_area, frontierCells.size(), frontierArea, time_span.count(), 
        //     scenario, algorithmSpecification.algorithmEnum);
        reply.frontiers_found = frontiers_count;
        reply.success = true;
        return reply.success;
    }

    bool isOccupied(octomath::Vector3 const& grid_coordinates_toTest, octomap::OcTree& octree)
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

    bool isExplored(octomath::Vector3 const& grid_coordinates_toTest, octomap::OcTree& octree)
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
    }