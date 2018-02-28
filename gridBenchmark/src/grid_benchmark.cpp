#include <grid_benchmark.h>

namespace mapper {

    GridBenchmark::GridBenchmark(Scenario scenario, std::string scenarioName, octomap::OcTree & octree)
        :scenario(scenario), scenarioName(scenarioName), octree (octree),
        rayDirections ({
            octomath::Vector3(1, 0, 0), // FRONT
            octomath::Vector3(0, 1, 0), // LEFT
            octomath::Vector3(0, -1, 0), // RIGHT
            octomath::Vector3(-1, 0, 0), // BACKWARDS
            octomath::Vector3(0, 0, -1), // UP
            octomath::Vector3(0, 0, 1), // DOWN
        }), z_levels{}
    {}

    int GridBenchmark::getFrontierNumber()
    {
        return frontierCells.size();
    }

    ResultSet GridBenchmark::findFrontierCells(octomath::Vector3  max, octomath::Vector3  min, Directions dimensions,
            AlgorithmObject & algorithmSpecification) 
    {
        octomath::Vector3 size = max - min;
        double area = (size.x()*size.y()*size.z());
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

        // === Task ===
        frontierCells.clear();
        z_levels.clear();
        float z_lvl;
        float x_min = 0;
        float x_max = 0;
        float y_min = 0;
        float y_max = 0;


        bool hasUnExploredNeighbors;
        int gridIterations = 0;
        float frontierArea = 0;
        double total_area = 0;
        double coverage = 0;
        octomath::Vector3 grid_coordinates_curr, grid_coordinates_toTest;
        int i;

        float z_max = max.z();
        float z_min = min.z();
        if(dimensions == Directions::twoD)
        {
            z_max = z_min = 1;  // Earlier 2D tests were done with this z_level
            area = size.x()*size.y();
            max = octomath::Vector3(max.x(), max.y(), z_max);
            min = octomath::Vector3(min.x(), min.y(), z_min);
        }

        Voxel currentVoxel;
        algorithmSpecification.iteratorInit(min, max);


        octomath::Vector3 bbxMin (currentVoxel.x, currentVoxel.y, currentVoxel.z);
        octomath::Vector3 bbxMax (currentVoxel.x, currentVoxel.y, currentVoxel.z);

        while(!algorithmSpecification.iteratorEndReached())
        {
            algorithmSpecification.getCurrentCell(currentVoxel);
            //ROS_WARN_STREAM("At cell " << currentVoxel);


            coverage = algorithmSpecification.calculateCoverage(currentVoxel.size, dimensions);
            grid_coordinates_curr = octomath::Vector3(currentVoxel.x, currentVoxel.y, currentVoxel.z);
            //ROS_WARN_STREAM("For " << grid_coordinates_curr << " occupied? " << isOccupied(grid_coordinates_curr) 
            //    << " explored? " << isExplored(grid_coordinates_curr));
            if(isExplored(grid_coordinates_curr)
                && !isOccupied(grid_coordinates_curr)) 
            {
                hasUnExploredNeighbors = false;
                //ROS_WARN_STREAM(grid_coordinates_curr << "Level 1");
                for(i = 0; i < dimensions; i++)
                { 
                    octomath::Vector3 direction = rayDirections[i];
                    grid_coordinates_toTest = algorithmSpecification.calculateNeighbor(grid_coordinates_curr, direction, currentVoxel.size);
                    if(!isOccupied(grid_coordinates_toTest))
                    {
                        hasUnExploredNeighbors = !isExplored(grid_coordinates_toTest) || hasUnExploredNeighbors;
                    }
                }
                if(hasUnExploredNeighbors)
                {
                    frontierCells.emplace(frontierCells.begin(), currentVoxel.x, currentVoxel.y, currentVoxel.z, currentVoxel.size);
                    frontierArea += coverage;
                    z_levels.emplace( grid_coordinates_curr.z());
                }
            }
            gridIterations++;
            total_area+=coverage;

            x_max = std::max(grid_coordinates_curr.x(), x_max);
            x_min = std::min(grid_coordinates_curr.x(), x_min);
            y_max = std::max(grid_coordinates_curr.y(), y_max);
            y_min = std::min(grid_coordinates_curr.y(), y_min);
            max = octomath::Vector3 (x_max, y_max, 1);
            min = octomath::Vector3 (x_min, y_min, 1);


            bbxMax = octomath::Vector3 (x_max, y_max, 1);
            bbxMin = octomath::Vector3 (x_min, y_min, 1);



            algorithmSpecification.iteratorNext();
        }

        // === TIME ===
        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

        std::string algoName = ResultSecretary::translateAlgorithmName(algorithmSpecification.algorithmEnum);

        // ROS_WARN_STREAM("Algorithm "<<algoName);
        // ROS_WARN_STREAM("Explored boundaries "<<bbxMin<<" - "<<bbxMax);
        // ROS_WARN_STREAM("Iterated over "<<gridIterations<<" - "<< total_area <<"m found "<<frontierCells.size()
        //     <<" frontier cells that cover "<<frontierArea
        //     <<" m. With "<<z_levels.size()<<" z levels. Frontiers found in "<<time_span.count()<<" seconds.");

        

        //storage.storeAlgorithmResults(scenario, algorithmSpecification.algorithmEnum, result);
        return ResultSet (gridIterations, total_area, frontierCells.size(), frontierArea, time_span.count(), 
            scenario, algorithmSpecification.algorithmEnum);
    }

    void GridBenchmark::drawWorld(std::string const& fileName, octomath::Vector3 max, octomath::Vector3 real_min, 
            float z_level, bool printFrontierCells) const
    {
        float diff_x, diff_y, halfSize;
        float grid_res = octree.getResolution();
        double bmp_iteration = (grid_res/2);
        float grid_factor = grid_res / (bmp_iteration);
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

        int bmp_y, grid_x, grid_y;
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
                grid_x = std::floor(  (frontier.x - real_min.x()) / grid_res);
                grid_y = std::floor(  (frontier.y - real_min.y()) / grid_res);
                int bmp_x_min = grid_x * grid_factor;
                int bmp_y_min = grid_y * grid_factor;
                for (int l = 0; l < grid_factor; l++)
                {
                    for (int r = 0; r < grid_factor; r++)
                    {
                        bmp_x = bmp_x_min+l;
                        bmp_y = bmp_y_min+r;
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
        BMP_WriteFile( bmp, ("/home/mfaria/Margarida/20161129_frontierCells/frontierCellsResults/"+fileName).c_str() );
        BMP_CHECK_ERROR( stderr);

        // /* Free output image memory */
         BMP_Free( bmp );
    }

    bool GridBenchmark::isOccupied(octomath::Vector3 const& grid_coordinates_toTest) const
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

    bool GridBenchmark::isExplored(octomath::Vector3 const& grid_coordinates_toTest) const
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

    void GridBenchmark::getBounds(std::tuple<double, double, double> & min, std::tuple<double, double, double> & max)
    {
        octree.getMetricMin(std::get<0>(min), std::get<1>(min), std::get<2>(min));
        octree.getMetricMax(std::get<0>(max), std::get<1>(max), std::get<2>(max));
    }

    std::list<Voxel> const& GridBenchmark::getFrontierCells() const
    {
        return frontierCells;
    }

    AlgorithmObject::AlgorithmObject(std::string algorithmName, Algorithm algorithmEnum, octomap::OcTree& octree, float grid_res)
        :algorithmName(algorithmName), algorithmEnum(algorithmEnum), octree(octree), grid_res(grid_res)
    { }

    void AlgorithmObject::setBoundingBox(octomath::Vector3 min, octomath::Vector3 max)
    {
        if(algorithmEnum == regularGrid_2d || algorithmEnum == sparseGrid_2d)
        {
            // Earlier 2D tests were done with this z_level
            bbx_min = octomath::Vector3(min.x(), min.y(), 1);
            bbx_max = octomath::Vector3(max.x(), max.y(), 1.1f);
            //ROS_WARN_STREAM("In 2D");
        }
        else
        {
            //ROS_WARN_STREAM("In 3D");
            bbx_min = min;
            bbx_max = max;
        }

        //ROS_WARN_STREAM("Bounding box [ " << bbx_min << " - " << bbx_max << "]");
    }

     octomath::Vector3 const& AlgorithmObject::getBoundingBoxMin() const
     {
        return bbx_min;
     }
     octomath::Vector3 const& AlgorithmObject::getBoundingBoxMax() const
     {
        return bbx_max;
     }

    double AlgorithmObject::calculateCoverage(double side, Directions dimensions)
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

///  ==================== REGULAR GRID ====================
    RegularGrid::RegularGrid(std::string algorithmName, Algorithm algorithmEnum, octomap::OcTree& octree, float grid_res)
        :AlgorithmObject(algorithmName, algorithmEnum, octree, grid_res)
    { }
    
    void RegularGrid::iteratorNext()
    {
        //ROS_WARN_STREAM(y<<" > "<<bbx_max.y() << " ----  diff "<< (bbx_max.y()-y) << " <= " << grid_res);
        float diff = (bbx_max.y()-y);
        //if(y > bbx_max.y())
        if(diff <= grid_res)
        {
            diff = bbx_max.x() - x;
            //ROS_WARN_STREAM(" ----  diff "<< diff << " <= " << grid_res);
            if(diff <= grid_res )
            {
                z+=grid_res;
                y=bbx_min.y();
                x=bbx_min.x();
                //ROS_WARN_STREAM("Incrementing z");
            }
            else
            {
                x+=grid_res;
                y=bbx_min.y();
                //ROS_WARN_STREAM("Incrementing x");
            }
        }
        else
        {
            y+=grid_res;
            //ROS_WARN_STREAM("Incrementing y");
        }

        //return Voxel(x, y, z, grid_res);
    }
    
    bool RegularGrid::iteratorInit(octomath::Vector3 min, octomath::Vector3 max)
    {
        setBoundingBox(min, max);
        x = bbx_min.x();
        y = bbx_min.y();
        z = bbx_min.z();
        return true;
        /*return Voxel(x, y, z, grid_res);*/
    }
    
    bool RegularGrid::iteratorEndReached() const
    {
        //ROS_WARN_STREAM("iteratorEnd?  " << z << " > " << bbx_max.z() << "  -->  " << (z > bbx_max.z()));
        return z > bbx_max.z();
    }
    
    octomath::Vector3 RegularGrid::calculateNeighbor(octomath::Vector3 grid_coordinates_curr, 
            octomath::Vector3 direction, double voxel_size) const
    {
        return grid_coordinates_curr + (direction * voxel_size);
    }

    void RegularGrid::getCurrentCell(Voxel& cell) const
    {
        cell = Voxel(x, y, z, grid_res);
    }

///  ==================== SPARSE GRID ====================
    SparseGrid::SparseGrid(std::string algorithmName, Algorithm algorithmEnum, octomap::OcTree& octree, float grid_res)
        :AlgorithmObject(algorithmName, algorithmEnum, octree, grid_res)
    { }
    
    void SparseGrid::iteratorNext()
    {
        it++;
    }
    
    bool SparseGrid::iteratorInit(octomath::Vector3 min, octomath::Vector3 max)
    {
        setBoundingBox(min, max);

        octomap::OcTreeKey bbxMinKey, bbxMaxKey;
        if(!octree.coordToKeyChecked(bbx_min, bbxMinKey) || !octree.coordToKeyChecked(bbx_max, bbxMaxKey))
        {
            ROS_ERROR_STREAM("Problems with the octree");
            return false;
        }
        it = octree.begin_leafs_bbx(bbxMinKey,bbxMaxKey);

        //ROS_WARN_STREAM("Bounding box bbx [ " << bbx_min << " - " << bbx_max << "]" );
        return true;
    }
    
    bool SparseGrid::iteratorEndReached() const
    {
        return it == octree.end_leafs_bbx();
    }
    
    octomath::Vector3 SparseGrid::calculateNeighbor(octomath::Vector3 grid_coordinates_curr, 
            octomath::Vector3 direction, double voxel_size) const
    {
        //double offset = (voxel_size / 2) ;
        return grid_coordinates_curr + (direction * grid_res) ;
    }

    void SparseGrid::getCurrentCell(Voxel& cell) const
    {
        octomath::Vector3 coord = it.getCoordinate();
        cell = Voxel(coord.x(), coord.y(), coord.z(), it.getSize());
    }

    /// Print a file for each level stored during frontier cell search
    /// return the number of levels
     int GridBenchmark::drawAllLevels(std::string dataset_name, octomath::Vector3 max, octomath::Vector3 real_min)
     {
        //ROS_WARN_STREAM("Boundaries [" << real_min << "  -  " << max << "]");
        std::string filename;
        float min_z = *z_levels.begin();
        float z_iteration = octree.getResolution();

        for(float z_level = std::max(real_min.z(), min_z); z_level <= max.z(); z_level += z_iteration)
        {
            filename = dataset_name+"_"+std::to_string(z_level)+".bmp";
            drawWorld(filename, max, real_min, z_level, true);
            ROS_WARN_STREAM("Now level "<< z_level);
        }


        return z_levels.size();
     }

    void ResultSecretary::exportForTables(std::ofstream & myfile) const
    {
        // 2D - Frontier Area vs Count
        myfile << ",,,,,,,,,,,,,,,,,,,,,,,,,\n";
        myfile << "2D - Frontier Area vs Count,,,,,,,,,,,,,,,,,,,,,,,,,\n";
        myfile << ",Room,Corridor,Z,2 pillars,4 pillars,,,,,,,,,,,,,,,,,,,,\n";
        myfile << "Regular Grid #," << getResult(circle, regularGrid_2d).frontierCellsFound << "," << getResult(double_circle, regularGrid_2d).frontierCellsFound 
            << "," << getResult(s, regularGrid_2d).frontierCellsFound << "," << getResult(oil, regularGrid_2d).frontierCellsFound 
            << "," << getResult(oil_larger, regularGrid_2d).frontierCellsFound << ",,,,,,,,,,,,,,,,,,,,\n";
        myfile << "Sparse Grid #," << getResult(circle, sparseGrid_2d).frontierCellsFound << "," << getResult(double_circle, sparseGrid_2d).frontierCellsFound 
            << "," << getResult(s, sparseGrid_2d).frontierCellsFound << "," << getResult(oil, sparseGrid_2d).frontierCellsFound 
            << "," << getResult(oil_larger, sparseGrid_2d).frontierCellsFound << ",,,,,,,,,,,,,,,,,,,,\n";
        myfile << "Regular Grid Area," << getResult(circle, regularGrid_2d).frontierArea << "," << getResult(double_circle, regularGrid_2d).frontierArea 
            << "," << getResult(s, regularGrid_2d).frontierArea << "," << getResult(oil, regularGrid_2d).frontierArea 
            << "," << getResult(oil_larger, regularGrid_2d).frontierArea << ",,,,,,,,,,,,,,,,,,,,\n";
        myfile << "Sparse Grid Area," << getResult(circle, sparseGrid_2d).frontierArea << "," << getResult(double_circle, sparseGrid_2d).frontierArea 
            << "," << getResult(s, sparseGrid_2d).frontierArea << "," << getResult(oil, sparseGrid_2d).frontierArea 
            << "," << getResult(oil_larger, sparseGrid_2d).frontierArea << ",,,,,,,,,,,,,,,,,,,,\n";

        // 3D - Frontier Volume vs Count
        myfile << ",,,,,,,,,,,,,,,,,,,,,,,,,\n";
        myfile << "3d - Frontier Area vs Count,,,,,,,,,,,,,,,,,,,,,,,,,\n";
        myfile << ",Room,Corridor,Z,2 pillars,4 pillars,,,,,,,,,,,,,,,,,,,,\n";
        myfile << "Regular Grid #," << getResult(circle, regularGrid_3d).frontierCellsFound << "," << getResult(double_circle, regularGrid_3d).frontierCellsFound 
            << "," << getResult(s, regularGrid_3d).frontierCellsFound << "," << getResult(oil, regularGrid_3d).frontierCellsFound 
            << "," << getResult(oil_larger, regularGrid_3d).frontierCellsFound << ",,,,,,,,,,,,,,,,,,,,\n";
        myfile << "Sparse Grid #," << getResult(circle, sparseGrid_3d).frontierCellsFound << "," << getResult(double_circle, sparseGrid_3d).frontierCellsFound 
            << "," << getResult(s, sparseGrid_3d).frontierCellsFound << "," << getResult(oil, sparseGrid_3d).frontierCellsFound 
            << "," << getResult(oil_larger, sparseGrid_3d).frontierCellsFound << ",,,,,,,,,,,,,,,,,,,,\n";
        myfile << "Regular Grid Area," << getResult(circle, regularGrid_3d).frontierArea << "," << getResult(double_circle, regularGrid_3d).frontierArea 
            << "," << getResult(s, regularGrid_3d).frontierArea << "," << getResult(oil, regularGrid_3d).frontierArea 
            << "," << getResult(oil_larger, regularGrid_3d).frontierArea << ",,,,,,,,,,,,,,,,,,,,\n";
        myfile << "Sparse Grid Area," << getResult(circle, sparseGrid_3d).frontierArea << "," << getResult(double_circle, sparseGrid_3d).frontierArea 
            << "," << getResult(s, sparseGrid_3d).frontierArea << "," << getResult(oil, sparseGrid_3d).frontierArea 
            << "," << getResult(oil_larger, sparseGrid_3d).frontierArea << ",,,,,,,,,,,,,,,,,,,,\n";

        // Execution time
        myfile << ",,,,,,,,,,,,,,,,,,,,,,,,,\n";
        myfile << "Execution time,,,,,,,,,,,,,,,,,,,,,,,,,\n";
        myfile << ",Room,Corridor,Z,2 pillars,4 pillars,,,,,,,,,,,,,,,,,,,,\n";
        myfile << "Regular Grid 2D," << getResult(circle, regularGrid_2d).executionTime << "," << getResult(double_circle, regularGrid_2d).executionTime 
            << "," << getResult(s, regularGrid_2d).executionTime << "," << getResult(oil, regularGrid_2d).executionTime 
            << "," << getResult(oil_larger, regularGrid_2d).executionTime << ",,,,,,,,,,,,,,,,,,,,\n";
        myfile << "Sparse Grid 2D," << getResult(circle, sparseGrid_2d).executionTime << "," << getResult(double_circle, sparseGrid_2d).executionTime 
            << "," << getResult(s, sparseGrid_2d).executionTime << "," << getResult(oil, sparseGrid_2d).executionTime 
            << "," << getResult(oil_larger, sparseGrid_2d).executionTime << ",,,,,,,,,,,,,,,,,,,,\n";      
        myfile << "Regular Grid 3D," << getResult(circle, regularGrid_3d).executionTime << "," << getResult(double_circle, regularGrid_3d).executionTime 
            << "," << getResult(s, regularGrid_3d).executionTime << "," << getResult(oil, regularGrid_3d).executionTime 
            << "," << getResult(oil_larger, regularGrid_3d).executionTime << ",,,,,,,,,,,,,,,,,,,,\n";
        myfile << "Sparse Grid 3D," << getResult(circle, sparseGrid_3d).executionTime << "," << getResult(double_circle, sparseGrid_3d).executionTime 
            << "," << getResult(s, sparseGrid_3d).executionTime << "," << getResult(oil, sparseGrid_3d).executionTime 
            << "," << getResult(oil_larger, sparseGrid_3d).executionTime << ",,,,,,,,,,,,,,,,,,,,\n";

        // Cell iteration relation
        myfile << ",,,,,,,,,,,,,,,,,,,,,,,,,\n";
        myfile << "2D Cell sparse cell iteration relation,,,,,,,,,,,,,,,,,,,,,,,,,\n";
        myfile << ",Unknown cells,Analysed cells,Frontier,Total possible it,,,,,,,,,,,,,,,,,,,,,\n";
        myfile << "Room,";
        insertIterationCalculations(myfile, getResult(circle, sparseGrid_2d).frontierCellsFound, 
            getResult(circle, regularGrid_2d).cellIterations, getResult(circle, sparseGrid_2d).cellIterations);
        myfile << "Corridor,";
        insertIterationCalculations(myfile, getResult(double_circle, sparseGrid_2d).frontierCellsFound, 
            getResult(double_circle, regularGrid_2d).cellIterations, getResult(double_circle, sparseGrid_2d).cellIterations);
        myfile << "Z,";
        insertIterationCalculations(myfile, getResult(s, sparseGrid_2d).frontierCellsFound, 
            getResult(s, regularGrid_2d).cellIterations, getResult(s, sparseGrid_2d).cellIterations);
        myfile << "2 pillars,";
        insertIterationCalculations(myfile, getResult(oil, sparseGrid_2d).frontierCellsFound, 
            getResult(oil, regularGrid_2d).cellIterations, getResult(oil, sparseGrid_2d).cellIterations);
        myfile << "4 pillars,";
        insertIterationCalculations(myfile, getResult(oil_larger, sparseGrid_2d).frontierCellsFound, 
            getResult(oil_larger, regularGrid_2d).cellIterations, getResult(oil_larger, sparseGrid_2d).cellIterations);
        myfile << ",,,,,,,,,,,,,,,,,,,,,,,,,\n";
        myfile << "3d Cell sparse cell iteration relation,,,,,,,,,,,,,,,,,,,,,,,,,\n";
        myfile << ",Unknown cells,Analysed cells,Frontier cells,Total possible it,,,,,,,,,,,,,,,,,,,,,\n";
        myfile << "Room,";
        insertIterationCalculations(myfile, getResult(circle, sparseGrid_3d).frontierCellsFound, 
            getResult(circle, regularGrid_3d).cellIterations, getResult(circle, sparseGrid_3d).cellIterations);
        myfile << "Corridor,";
        insertIterationCalculations(myfile, getResult(double_circle, sparseGrid_3d).frontierCellsFound, 
            getResult(double_circle, regularGrid_3d).cellIterations, getResult(double_circle, sparseGrid_3d).cellIterations);
        myfile << "Z,";
        insertIterationCalculations(myfile, getResult(s, sparseGrid_3d).frontierCellsFound, 
            getResult(s, regularGrid_3d).cellIterations, getResult(s, sparseGrid_3d).cellIterations);
        myfile << "2 pillars,";
        insertIterationCalculations(myfile, getResult(oil, sparseGrid_3d).frontierCellsFound, 
            getResult(oil, regularGrid_3d).cellIterations, getResult(oil, sparseGrid_3d).cellIterations);
        myfile << "4 pillars,";
        insertIterationCalculations(myfile, getResult(oil_larger, sparseGrid_3d).frontierCellsFound, 
            getResult(oil_larger, regularGrid_3d).cellIterations, getResult(oil_larger, sparseGrid_3d).cellIterations);
        
    }


    void ResultSecretary::exportCsv(std::list<Scenario> scenarioOrder, bool exportTables, std::string fileName) const
    {
        int algoCount = 0;
        int scenarioCount;
        std::ofstream myfile;
        myfile.open ("/home/mfaria/Margarida/20161129_frontierCells/frontierCellsResults/"+ fileName +".csv");

        //  ==== Scenario iteration order ====
        //std::list<Scenario> scenarioOrder = {Scenario::circle, Scenario::double_circle, Scenario::s, Scenario::oil, Scenario::oil_larger};
        myfile<<",Circle,,,,,Corridor,,,,,Z,,,,,2 pillars,,,,,3 pillars,,,,\n";
        myfile<<",Total Space,,Frontier Space,,,Total Space,,Frontier Space,,,Total Space,,Frontier Space,,,Total Space,,Frontier Space,,,Total Space,,Frontier Space,\n";
        myfile<<",Cells,Covered,Found,Covered,Time,Cells,Covered,Found,Covered,Time,Cells,Covered,Found,Covered,Time,Cells,Covered,Found,Covered,Time,Cells,Covered,Found,Covered,Time\n";

        for(auto const& algoIt : resultStorage)
        {
            scenarioCount = 0;
            AlgorithmResults const& algorithmResultsForEachScenario = algoIt.second;
            // Algo name to start the line
            myfile << translateAlgorithmName(algorithmResultsForEachScenario.begin()->second.algorithm) << ",";
            for (Scenario scenario : scenarioOrder)
            {
                if(algorithmResultsForEachScenario.count(scenario) == 0)
                {
                    continue;
                }
                ResultSet const& resultSet = algorithmResultsForEachScenario.at(scenario);
                myfile << resultSet.cellIterations << "," << resultSet.scenarioArea << "," 
                    << resultSet.frontierCellsFound << "," << resultSet.frontierArea << "," << resultSet.executionTime << ",";
                scenarioCount++;
            }
            myfile << "\n";
            algoCount++;
        }

        
        if(exportTables)
        {
            exportForTables(myfile);
        }




        myfile.close();

        ROS_WARN_STREAM("Exported "<< algoCount << " algorithms in "<<scenarioCount<<" scenarios.");
    }

    void ResultSecretary::insertIterationCalculations(std::ofstream & myfile , int frontierCells, int totalCells, int sparseCells) const
    {
        int unknowCells = totalCells - sparseCells;
        myfile << unknowCells <<"," << sparseCells <<"," << frontierCells <<"," << totalCells <<",,,,,,,,,,,,,,,,,,,,,\n";
    }

    std::string ResultSecretary::translateAlgorithmName(Algorithm algorithm) 
    {
        std::string algoName;
        switch(algorithm)
        {
            case regularGrid_2d: algoName = "regularGrid_2d"; break;
            case regularGrid_3d: algoName = "regularGrid_3d"; break;
            case sparseGrid_2d: algoName = "sparseGrid_2d"; break;
            case sparseGrid_3d: algoName = "sparseGrid_3d"; break;
        }
        return algoName;
    }

    void ResultSecretary::storeAlgorithmResults(ResultSet result) 
    {
        resultStorage[result.algorithm].emplace(result.scenario, result);
    }

    ResultSet const& ResultSecretary::getResult(Scenario scenario, Algorithm algorithm) const
    {
        return resultStorage.at(algorithm).at(scenario);
    }

    bool ResultSet::compareResults(ResultSet const& newResults, bool print) const
    {
        double diff;
        if(scenario != newResults.scenario)
        {
            ROS_WARN_STREAM("Different scenario.");
            return false;
        }
        if(algorithm != newResults.algorithm)
        {
            ROS_WARN_STREAM("Different algorithm.");
            return false;
        }
        // CELL ITERATION
        if(cellIterations != newResults.cellIterations)
        {
            if(print)   ROS_WARN_STREAM("Different cell iteration count " << newResults.cellIterations << " != " << cellIterations);
            return false;
        }
        // SCENARIO AREA
        diff = std::abs(scenarioArea - newResults.scenarioArea);
        if(diff > 0.01)
        {
            if(print)   ROS_WARN_STREAM("Different scenario area volume " << newResults.scenarioArea << " != " << scenarioArea << " [diff " << diff << "]");
            return false;
        }
        // FRONTIER CELL COUNT
        if(frontierCellsFound != newResults.frontierCellsFound)
        {
            if(print)   ROS_WARN_STREAM("Different frontier cells count " << newResults.frontierCellsFound << " != " << frontierCellsFound);
            return false;
        }
        // FRONTIER CELL COVERED
        diff = std::abs(frontierArea - newResults.frontierArea);
        if(diff > 0.1)
        {
            if(print)   ROS_WARN_STREAM("Differences in frontier area time are too big " << newResults.frontierArea << " != " << frontierArea << " [diff " << diff << "]");
            return false;
        }
        // EXECUTION TIME
        diff = std::abs(executionTime - newResults.executionTime);
        if(diff > 0.1)
        {
            if(print)   ROS_WARN_STREAM("Differences in execution time are too big " << newResults.executionTime << " != " << executionTime << " [diff " << diff << "]");
            return false;
        }

        return true;
    }
}