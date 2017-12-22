#include <frontier_cells_sandbox.h>

namespace mapper {
	FrontierCellsSandbox::FrontierCellsSandbox(Scenario scenario, std::string scenarioName)
		: width(50), height(50), scenario(scenario), scenarioName(scenarioName), 
        rayDirections ({
            octomath::Vector3(1, 0, 0), // FRONT
            octomath::Vector3(0, 1, 0), // LEFT
            octomath::Vector3(0, -1, 0), // RIGHT
            octomath::Vector3(-1, 0, 0), // BACKWARDS
            octomath::Vector3(0, 0, -1), // UP
            octomath::Vector3(0, 0, 1), // DOWN
        }), z_levels{}
	{
	}

    bool FrontierCellsSandbox::findFrontierCells(octomath::Vector3 max, octomath::Vector3 min, Directions directions) 
    {
        octomath::Vector3 dimensions = max - min;
        double area = (dimensions.x()*dimensions.y()*dimensions.z());
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

        float covered_area = 0;
        double frontier_cells_coverage = 0;

        // === Task ===
        frontierCells.clear();
        z_levels.clear();
        bool hasUnExploredNeighbors;
        int count = 0;
        octomath::Vector3 grid_coordinates_curr, grid_coordinates_toTest;
        int i;

        float z_max = max.z();
        float z_min = min.z();
        if(directions == Directions::twoD)
        {
            z_max = z_min = 1;  // Earlier 2D tests were done with this z_level
            area = dimensions.x()*dimensions.y();
            ROS_WARN_STREAM("2D ");
        }

        for (float z = z_min; z <= z_max; z+= incrementIteration())
        {   
            for (float x = min.x(); x < max.x(); x+= incrementIteration()) 
            {
                for (float y = min.y(); y < max.y(); y+= incrementIteration())
                {
                    grid_coordinates_curr = octomath::Vector3(x, y, z);
                    if(isExplored(grid_coordinates_curr)
                        && !isOccupied(grid_coordinates_curr)) 
                    {
                        hasUnExploredNeighbors = false;

                        for(i = 0; i < directions; i++)
                        { 
                            octomath::Vector3 direction = rayDirections[i];
                            grid_coordinates_toTest = grid_coordinates_curr + (direction * incrementIteration());
                            if(!isOccupied(grid_coordinates_toTest))
                            {
                                hasUnExploredNeighbors = !isExplored(grid_coordinates_toTest) || hasUnExploredNeighbors;
                            }
                        }
                        if(hasUnExploredNeighbors)
                        {
                            frontierCells.emplace(frontierCells.begin(), x, y, z, incrementIteration());
                            frontier_cells_coverage += calculateCoverage(incrementIteration(), directions);
                            //ROS_WARN_STREAM ("Frontier cell "<< grid_coordinates_curr);
                        }
                    }
                    count++;
                    covered_area+=calculateCoverage(incrementIteration(), directions);
                }
            }
            //ROS_WARN_STREAM("New z level "<<z << ". Z max "<<z_max);
            z_levels.emplace(z);
        }

        // === TIME ===
        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        double frontier_cells_length = frontierCells.size()*incrementIteration();

        std::string algorithmName;
        Algorithm algorithmEnum;
        if(directions == Directions::threeD)
        {
            algorithmName = "[3D] Regular Grid";
            algorithmEnum = Algorithm::regularGrid_3d;
        }
        else if(directions == Directions::twoD)
        {
            algorithmName = "[2D] Regular Grid";
            algorithmEnum = Algorithm::regularGrid_2d;
        }
        else
        {
            ROS_ERROR_STREAM("Strange number of directions to find neighbors "<<directions<< " @ FrontierCellsSandbox::findFrontierCells()");
        }
        ResultSet result (count, area, frontierCells.size(), (frontierCells.size()*incrementIteration()), time_span.count(), scenarioName, algorithmName);
        storage.storeAlgorithmResults(scenario, algorithmEnum, result);

        ROS_WARN_STREAM("Regular grid iterated over "<<count<<" that covers "<< covered_area << "m of " << area  <<"m. Found "<<frontierCells.size()
            <<" frontier cells that cover "<<frontier_cells_coverage
            <<" m. With "<<z_levels.size()<<" z levels. Frontiers found in "<<time_span.count()<<" seconds.");
    }

    std::list<Voxel> const& FrontierCellsSandbox::getFrontierCells() const
    {
        return frontierCells;
    }

    void FrontierCellsSandbox::drawWorld(std::string const& fileName, octomath::Vector3 max, octomath::Vector3 real_min, float z_level, bool printFrontierCells) const
    {
        float diff_x, diff_y, halfSize;
        double bmp_iteration = 0.05;
        int bmp_width =  std::ceil(   std::abs(max.x()-real_min.x())  / bmp_iteration);
        int bmp_height = std::ceil(   std::abs(max.y()-real_min.y())  / bmp_iteration);

        //ROS_WARN_STREAM("std::ceil(   " << std::abs(max.y()-real_min.y()) << " / " << bmp_iteration <<")");
        //ROS_WARN_STREAM("Bmp Boundaries [" << bmp_width << ", " << bmp_height <<"]");

        BMP* bmp;
        bmp = BMP_Create( bmp_width+1, bmp_height+2, 24 );
        BMP_CHECK_ERROR( stderr);

        int bothRedAndGreen = 0;
        int justRed = 0;
        int justGreen = 0;
        int empty = 0;

        int bmp_y;
        int bmp_x = 0;
        for (double x = real_min.x(); x < max.x(); x+= bmp_iteration )
        {
            bmp_y = 0;
            for (double y = real_min.y(); y < max.y(); y+= bmp_iteration )
            {
                int r = 0;
                octomath::Vector3 target (x, y, z_level);
                if(isOccupied(target))
                {
                    r = 250;
                }
                int g = 0;
                if(isExplored(target))
                {
                    g = 250;
                }

                if(r == 250 && g == 250)
                {
                    bothRedAndGreen++;
                }
                else if(r == 250 )
                {
                    justRed++;
                }
                else if(g == 250)
                {
                    justGreen++;
                }
                else
                {
                    empty++;
                }
                /* Invert RGB values */
                BMP_SetPixelRGB( bmp, bmp_x, bmp_y, r, g, 0 );
                if(BMP_GetError() != BMP_OK)
                {
                    BMP_CHECK_ERROR( stderr);
                }
                bmp_y++;
            }
            bmp_x++;
        }
        if(printFrontierCells)
        {
            for (Voxel frontier : frontierCells)
            {
                if(!frontier.isInZlevel(z_level))
                {
                    continue;
                }
                bmp_x = std::ceil(  (frontier.x - real_min.x()) / bmp_iteration);
                bmp_y = std::ceil(  (frontier.y - real_min.y()) / bmp_iteration);
                

                BMP_SetPixelRGB( bmp, bmp_x, bmp_y, 250, 250, 250 );
                if(BMP_GetError() != BMP_OK)
                {
                    BMP_CHECK_ERROR( stderr);
                    //ROS_WARN_STREAM("("<<frontier.x<<" - "<<real_min.x<<") / "<<bmp_iteration<<" = "<<bmp_x);
                    //ROS_WARN_STREAM("("<<frontier.y<<" - "<<real_min.y<<") / "<<bmp_iteration<<" = "<<bmp_y);
                    if(bmp_x >= bmp_width)
                    {
                        diff_x = std::abs(max.x()-frontier.x);
                        halfSize = frontier.size/2  +  0.0001;  // Taking rounding errors into account
                        if(diff_x <= halfSize)
                        {
                            ROS_WARN_STREAM("On the edge - no worries. ");
                        }
                        else
                        {
                            ROS_ERROR_STREAM("RED ALERT! Y ("<<frontier.x<<" - "<<real_min.x()<<") / "<<bmp_iteration<<" = "<<bmp_x);
                        }
                    }
                    if(bmp_x < 0 )
                    {
                        diff_x = std::abs(real_min.x()-frontier.x);
                        halfSize = frontier.size/2  +  0.0001;  // Taking rounding errors into account
                        if(diff_x <= halfSize)
                        {
                            ROS_WARN_STREAM("On the edge - no worries. ");
                        }
                        else
                        {
                            ROS_ERROR_STREAM("X ("<<frontier.x<<" - "<<real_min.x()<<") / "<<bmp_iteration<<" = "<<bmp_x);
                        }
                    }
                    if(bmp_y < 0)
                    {
                        ROS_ERROR_STREAM("Y ("<<frontier.y<<" - "<<real_min.y()<<") / "<<bmp_iteration<<" = "<<bmp_y);
                    }
                    if(bmp_y >= bmp_height)
                    {
                        diff_y = std::abs(max.y()-frontier.y);
                        halfSize = frontier.size/2  +  0.0001;  // Taking rounding errors into account
                        if(diff_y <= halfSize)
                        {
                            ROS_WARN_STREAM("On the edge - no worries. ");
                        }
                        else
                        {
                            ROS_ERROR_STREAM("RED ALERT! Y ("<<frontier.y<<" - "<<real_min.y()<<") / "<<bmp_iteration<<" = "<<bmp_y);
                        }
                    }
                }
            }
        }

        // Label
        int offset = std::floor(bmp_width/5);
        for ( int x = 0 ; x < offset ; ++x )
        {
            // Unexplored & free
            BMP_SetPixelRGB( bmp, x, bmp_height-1,    0, 0, 0 );
            BMP_SetPixelRGB( bmp, x, bmp_height-2,  0, 0, 0 );
            // Explored & free
            BMP_SetPixelRGB( bmp, x+offset, bmp_height-1,    250, 0, 0 );
            BMP_SetPixelRGB( bmp, x+offset, bmp_height-2,  250, 0, 0 );
            // Unexplored & occupied  == obstacles
            BMP_SetPixelRGB( bmp, x+offset*2, bmp_height-1,    0, 250, 0 );
            BMP_SetPixelRGB( bmp, x+offset*2, bmp_height-2,  0, 250, 0 );
            // Explored & occupied == never happens
            BMP_SetPixelRGB( bmp, x+offset*3, bmp_height-1,    250, 250, 0 );
            BMP_SetPixelRGB( bmp, x+offset*3, bmp_height-2,  250, 250, 0 );
            // Frontier
            BMP_SetPixelRGB( bmp, x+offset*4, bmp_height-1,    250, 250, 250 );
            BMP_SetPixelRGB( bmp, x+offset*4, bmp_height-2,  250, 250, 250 );
        }

        /* Save output image */
        BMP_WriteFile( bmp, ("/home/mfaria/openTheHatch/"+fileName).c_str() );
        BMP_CHECK_ERROR( stderr);

        // /* Free output image memory */
         BMP_Free( bmp );
    }

    double FrontierCellsSandbox::calculateCoverage(double side, Directions dimensions)
    {
        double coverage = 0;
        if(dimensions == twoD)
        {
            coverage = side * side;
        }
        else
        {
            coverage = side * side * side;
        }

        return coverage;
    }

    OctomapFrontierCellsSandbox::OctomapFrontierCellsSandbox(Scenario scenario, std::string scenarioName, std::string data_file)
        : FrontierCellsSandbox(scenario, scenarioName), octree (data_file)
        //: octree("data/20161129_octomap_circle_res02.bt")
    {
    }

    float OctomapFrontierCellsSandbox::incrementIteration() const
    {
        return 0.2f;
    }

    bool OctomapFrontierCellsSandbox::isOccupied(octomath::Vector3 const& grid_coordinates_toTest) const
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

    bool OctomapFrontierCellsSandbox::isExplored(octomath::Vector3 const& grid_coordinates_toTest) const
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

    bool OctomapFrontierCellsSandbox::findFrontierCells_sparseIteration(octomath::Vector3 & bbxMax, octomath::Vector3 & bbxMin, Directions dimensions) 
    {
        frontierCells.clear();
        z_levels.clear();

        float x_min = 0;
        float x_max = 0;
        float y_min = 0;
        float y_max = 0;

        octomath::Vector3 dim = bbxMax - bbxMin;
        double area = (dim.x()*dim.y()*dim.z());
        if(dimensions == Directions::twoD)
        {
            area = dim.x()*dim.y();
        }

        int i;
        float z_lvl;
        bool hasUnExploredNeighbors;
        octomap::OcTreeKey bbxMinKey, bbxMaxKey;
        if(!octree.coordToKeyChecked(bbxMin, bbxMinKey) || !octree.coordToKeyChecked(bbxMax, bbxMaxKey))
        {
            ROS_ERROR_STREAM("Problems with the octree");
        }
        octomap::OcTree::leaf_bbx_iterator it = octree.begin_leafs_bbx(bbxMinKey,bbxMaxKey);
        int count = 0;
        double coverage = 0;
        double total_area = 0;
        double frontier_area = 0;
        octomath::Vector3 grid_coordinates_curr, grid_coordinates_toTest;
        for (; it != octree.end_leafs_bbx(); ++it)
        {
            coverage = calculateCoverage(it.getSize(), dimensions);
            grid_coordinates_curr  = it.getCoordinate();
            if(isExplored(grid_coordinates_curr)
                && !isOccupied(grid_coordinates_curr)) 
            {
                hasUnExploredNeighbors = false;
                double offset = (it.getSize() / 2) +0.001;
                for(i = 0; i < dimensions; i++)
                { 
                    octomath::Vector3 direction = rayDirections[i];
                    grid_coordinates_toTest = grid_coordinates_curr + (direction * offset);
                    if(!isOccupied(grid_coordinates_toTest))
                    {
                        hasUnExploredNeighbors = !isExplored(grid_coordinates_toTest) || hasUnExploredNeighbors;
                    }
                }
                if(hasUnExploredNeighbors)
                {
                    frontierCells.emplace(frontierCells.begin(), grid_coordinates_curr.x(), grid_coordinates_curr.y(), grid_coordinates_curr.z(), it.getSize());
                    //ROS_WARN_STREAM ("Frontier cell "<< grid_coordinates_curr << " - " << it.getSize());

                    frontier_area+=coverage;
                    z_levels.emplace( grid_coordinates_curr.z());
                }
            }
            x_max = std::max(grid_coordinates_curr.x(), x_max);
            x_min = std::min(grid_coordinates_curr.x(), x_min);
            y_max = std::max(grid_coordinates_curr.y(), y_max);
            y_min = std::min(grid_coordinates_curr.y(), y_min);
            bbxMax = octomath::Vector3 (x_max, y_max, 1);
            bbxMin = octomath::Vector3 (x_min, y_min, 1);

            count++;
            total_area+=coverage;
            //ROS_WARN_STREAM("Analysing cell "<< grid_coordinates_curr << " coverage "<< coverage << " Total area covered: " << total_area);
        }
        ROS_WARN_STREAM("Explored boundaries "<<bbxMin<<" - "<<bbxMax);
        //ROS_WARN_STREAM("Sparse  grid iterated over "<<count<<" - "<< total_area <<"m found "<<frontierCells.size()<<" frontier cells with "<<frontier_area);
        ROS_WARN_STREAM("Sparse  grid iterated over "<<count<<" that covers "<< total_area << "m of " << area  <<"m. Found "<<frontierCells.size()
            <<" frontier cells that cover "<<frontier_area <<" m. With "<<z_levels.size()<<" z levels. ");
    }

    void ResultSecretary::exportCsv() const
    {
        int algoCount = 0;
        int scenarioCount;
        std::ofstream myfile;
        myfile.open ("/home/mfaria/openTheHatch/frontierResults.csv");

        //  ==== Scenario iteration order ====
        std::list<Scenario> scenarioOrder = {Scenario::circle, Scenario::double_circle, Scenario::s, Scenario::oil, Scenario::oil_larger};
        myfile<<",Circle,,,,,Double circle,,,,,S,,,,,Large oil structure 2 obstacles,,,,,Large oil structure 3 obstacles,,,,\n";
        myfile<<",Total test area,,Frontier cells,,,Total test area,,Frontier cells,,,Total test area,,Frontier cells,,,Total test area,,Frontier cells,,,Total test area,,Frontier cells,\n";
        myfile<<",# cells,Area (m2 or m3),# found,Area covered (m2 or m3),Execution time (seconds),# cells,Area (m2 or m3),# found,Area covered (m2 or m3),Execution time (seconds),# cells,Area (m2 or m3),# found,Area covered (m2 or m3),Execution time (seconds),# cells,Area (m2 or m3),# found,Area covered (m2 or m3),Execution time (seconds),# cells,Area (m2 or m3),# found,Area covered (m2 or m3),Execution time (seconds)\n";

        for(auto const& algoIt : resultStorage)
        {
            scenarioCount = 0;
            AlgorithmResults const& algorithmResultsForEachScenario = algoIt.second;
            // Algo name to start the line
            myfile << algorithmResultsForEachScenario.begin()->second.algorithm << ",";
            for (Scenario scenario : scenarioOrder)
            {
                if(algorithmResultsForEachScenario.count(scenario) == 0)
                {
                    continue;
                }
                ResultSet const& resultSet = algorithmResultsForEachScenario.at(scenario);
                myfile << resultSet.cellItertions << "," << resultSet.scenarioArea << "," 
                    << resultSet.frontierCellsFound << "," << resultSet.frontierArea << "," << resultSet.executionTime << ",";
                scenarioCount++;
            }
            myfile << "\n";
            algoCount++;
        }
        myfile.close();

        ROS_WARN_STREAM("Exported "<< algoCount << " algorithms in "<<scenarioCount<<" scenarios.");
    }

    void FrontierCellsSandbox::exportCsv() const
    {
        storage.exportCsv();
    }

    void OctomapFrontierCellsSandbox::getBounds(std::tuple<double, double, double> & min, std::tuple<double, double, double> & max)
    {
        octree.getMetricMin(std::get<0>(min), std::get<1>(min), std::get<2>(min));
        octree.getMetricMax(std::get<0>(max), std::get<1>(max), std::get<2>(max));
    }

    /// Print a file for each level stored during frontier cell search
    /// return the number of levels
    int OctomapFrontierCellsSandbox::drawAllLevels(std::string dataset_name, octomath::Vector3 max, octomath::Vector3 real_min)
    {
        //ROS_WARN_STREAM("Boundaries [" << real_min << "  -  " << max << "]");
        std::string filename;
        for(float z_level = real_min.z(); z_level <= max.z(); z_level += incrementIteration())
        {
            filename = dataset_name+"_"+std::to_string(z_level)+".bmp";
            drawWorld(filename, max, real_min, z_level, true);

            //ROS_WARN_STREAM("Now level "<< z_level);
        }
        return z_levels.size();
    }

    void ResultSecretary::storeAlgorithmResults(Scenario scenario, Algorithm algorithm, ResultSet results)
    {
        //if(!resultStorage.count(scenario))
        //{
            //ScenarioResults toAdd;
            //toAdd.emplace(algorithm, results);
            //resultStorage.emplace(scenario, std::map <Algorithm, ScenarioResults> {{algorithm, results}});
        //}
        //else{
            resultStorage[algorithm].emplace(scenario, results);
        //}
    }
}