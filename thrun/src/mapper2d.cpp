#include <mapper2d.h>
#include <sstream> 

namespace mapper {

    Mapper2D::Mapper2D(Point2d world_robot_position, float z_plane)
        :
        world_robot_position(world_robot_position),
        z_plane(z_plane),
        rayDirections ({
            Point2d(1, 0), // FRONT
            Point2d(0, 1), // LEFT
            Point2d(0, -1), // RIGHT
            Point2d(-1, 0), // BACKWARDS
            Point2d(1,  1), // FRONT LEFT
            Point2d(1, -1), // FRONT RIGHT
            Point2d(-1, 1), // BACKWARDS LEFT
            Point2d(-1, -1), // BACKWARDS RIGHT
        })
    {  

        grid_offset=Point2d( world_robot_position.x-range, world_robot_position.y-range ),
        grid_robot_position = toGridCoordinates(world_robot_position);


        /// If you are a c++ wizard please correct this humble code
        /// The compilar can't find the static variables
        /// Undefined reference to ... 
        float safety = Mapper2D::world_min_safety_distance;
        float next_step = Mapper2D::world_next_step_distance;
        world_near_distance = std::max(safety, next_step);
        // Init value function matrix
        for (int x = 0; x < value_func_dim; ++x)
        {
            for (int y = 0; y < value_func_dim; ++y)
            {
                valueFunction [x][y] = COST_MAXIMUM;
                informationGain [x][y] = explored;
            }
        }
        valueFunction [grid_robot_position.x][grid_robot_position.y] = 0.f;
        informationGain [grid_robot_position.x][grid_robot_position.y] = unexplored;
    }

    float Mapper2D::valueFunctionByGridCoordinates(Point2d const& grid_coordinates) const
    {
        return valueFunction[grid_coordinates.x][grid_coordinates.y];
    }

    int Mapper2D::toGridDistance(float world_distance) const
    {
        float res = Mapper2D::resolution;
        return std::ceil(world_distance / res);
    }

    Point2d Mapper2D::toGridCoordinates(Point2d const& world_coordinates) const{
        float res = Mapper2D::resolution;
        Point2d grid_coordinates = ( (world_coordinates - grid_offset) / res).ceil();
        // Rounding down for max position
        if(grid_coordinates.x == value_func_dim ) grid_coordinates.x -=1; 
        if(grid_coordinates.y == value_func_dim ) grid_coordinates.y -=1;
        if(!validateGridCoordinates(grid_coordinates))
        {
            ROS_ERROR_STREAM("Grid index out of bounds"
                << ".\n ( (world_coordinates - grid_offset) / res).ceil()"
                << ".\n ( ("<<world_coordinates<<" - "<<grid_offset<<") / "<<res<<").ceil()"
                << ".\n --> Conversion "<<world_coordinates << " to " << grid_coordinates
                << ".\n Maximum index: "<<value_func_dim -1 
                << ".\n Offset: "<<grid_offset
                << ".\n Resolution: "<<res
                << ".\n Range: "<<range
                << ".\n world_robot_position: "<<world_robot_position);
        }    
        return grid_coordinates;
    }

    Point2d Mapper2D::toWorldCoordinates(Point2d const& grid_coordinates) const
    {
        float res = Mapper2D::resolution;
        return (grid_coordinates * res) + grid_offset;
    }

    Point2d Mapper2D::toGridCoordinatesDEBUG(Point2d const& world_coordinates) const{
        float res = Mapper2D::resolution;
        Point2d grid_coordinates = ( (world_coordinates - grid_offset) / res).ceil();
        // Rounding down for max position
        if(grid_coordinates.x == value_func_dim ) grid_coordinates.x -=1; 
        if(grid_coordinates.y == value_func_dim ) grid_coordinates.y -=1;
        if(!validateGridCoordinates(grid_coordinates))
        {
            ROS_ERROR_STREAM("Grid index out of bounds"
                << ".\n --> Conversion "<<world_coordinates << " to " << grid_coordinates
                << ".\n Maximum index: "<<value_func_dim -1 
                << ".\n Offset: "<<grid_offset
                << ".\n Resolution: "<<res
                << ".\n Range: "<<range
                << ".\n world_robot_position: "<<world_robot_position);
        }    
        else
        {
            ROS_WARN_STREAM("--> Conversion "<<world_coordinates << " to " << grid_coordinates
                << ".\n Maximum index: "<<value_func_dim -1 
                << ".\n Offset: "<<grid_offset
                << ".\n Resolution: "<<res
                << ".\n Range: "<<range
                << ".\n world_robot_position: "<<world_robot_position);
        }
        return grid_coordinates;
    }

    bool Mapper2D::validateGridCoordinates(Point2d grid_coordinates) const
    {
        if(grid_coordinates.x >= value_func_dim || grid_coordinates.x < 0 
            || grid_coordinates.y >= value_func_dim || grid_coordinates.y < 0)
        {
            return false;
        }
        return true;
    }

    float Mapper2D::calculateValueFunction(Point2d const& unitary_direction, const octomap::OcTree* octree) const
    {
        // If occupied set value as off limits
        octomath::Vector3 world_obstacle_coordinates;
        octomath::Vector3 unitary_direction_3D = octomath::Vector3(unitary_direction.x, unitary_direction.y, 0.f);
        if(Mapper2D::isOccupied(world_robot_position, z_plane, octree, world_obstacle_coordinates, unitary_direction_3D))
        {
            Point2d world_obstacle_coordinates2D = Point2d (world_obstacle_coordinates.x(), world_obstacle_coordinates.y());
            float world_distance_to_obstacle = world_robot_position.distance(world_obstacle_coordinates2D);
            if(world_distance_to_obstacle <= world_near_distance )
            {
                return Mapper2D::COST_OFF_LIMITS;   
            }
        }

        //ROS_INFO_STREAM("The path is free");
        Point2d world_analysis_position = world_robot_position+unitary_direction;
        Point2d grid_analysis_position = toGridCoordinates(world_analysis_position);
        if(informationGain[grid_analysis_position.x][grid_analysis_position.y] == explored)
        {
            return COST_MAXIMUM;
        }
        else
        {
            float prev_value = valueFunctionByGridCoordinates(grid_analysis_position);
            float cost = Mapper2D::displacementCostXY(world_robot_position, world_analysis_position);
            return cost + prev_value;
        }
    }

    float Mapper2D::calculateValueFunction_v2(Point2d const& world_target_point, const octomap::OcTree* octree) const
    {
        octomath::Vector3 world_target_point_3D (world_target_point.x, world_target_point.y, z_plane);
        octomath::Vector3 world_robot_3D (world_robot_position.x, world_robot_position.y, z_plane);
        octomath::Vector3 world_direction_vector = world_target_point_3D - world_robot_3D;

        octomath::Vector3 world_obstacle_coordinates;
        if(isOccupied(world_robot_position, z_plane, octree, world_obstacle_coordinates, world_direction_vector))
        {
            float world_distance_to_obstacle = world_obstacle_coordinates.distance(world_robot_3D);
            if(world_distance_to_obstacle <= world_near_distance )
            {
                return Mapper2D::COST_OFF_LIMITS;   
            }
        }
        //ROS_INFO_STREAM("The path is free");
        Point2d grid_target_point = toGridCoordinates(world_target_point);
        if(informationGain[grid_target_point.x][grid_target_point.y] == explored )
        {
            return COST_MAXIMUM;
        }
        else
        {
            float prev_value = valueFunction[grid_target_point.x][grid_target_point.y];
            float cost = Mapper2D::displacementCostXY(world_robot_position, world_target_point);
            return cost + prev_value;
        }
    }

    bool Mapper2D::isOccupied(const Point2d origin, const float z,
        const octomap::OcTree* octree, octomath::Vector3& end, 
        const octomath::Vector3 direction) 
    {
        //ROS_INFO_STREAM("Casting ray from ("<<origin.x<<", "<<origin.y<<", "<<z<<") in direction "<<direction<<" with range "<<range);
        return octree->castRay(octomath::Vector3(origin.x, origin.y, z),
                direction,
                end,
                false,
                range
                );
    }

    float Mapper2D::displacementCostXY(Point2d const& curr_position, Point2d const& destination) 
    {
        return std::min(std::sqrt(pow( curr_position.x- destination.x, 2)
            + pow(curr_position.y - destination.y, 2)), static_cast<double>(Mapper2D::COST_MAXIMUM) );
    }

    void Mapper2D::updateInformationGain(Point2d const& grid_coordinates, InformationGain value)
    {
        //ROS_INFO_STREAM(" ("<< grid_coordinates.x << ", "<< grid_coordinates.y <<") = "<<value);
        informationGain[grid_coordinates.x][grid_coordinates.y] = value;
    }

    void Mapper2D::updateValueFunction(Point2d const& grid_coordinates, float value)
    {
        //ROS_INFO_STREAM(" ("<< grid_coordinates.x << ", "<< grid_coordinates.y <<") = "<<value);
        valueFunction[grid_coordinates.x][grid_coordinates.y] = value;
    }

    bool Mapper2D::findFrontierCells(std::list<Point2d> & frontierCells) const
    {

        int size = frontierCells.size();
        bool hasExploredNeighbors, hasUnExploredNeighbors;
        for (int y = 0; y < Mapper2D::value_func_dim; ++y)
        {
            for (int x = 0; x < Mapper2D::value_func_dim; ++x)
            {
                hasExploredNeighbors = false;
                hasUnExploredNeighbors = false;


                Point2d grid_coordinates_curr (x, y);

                for(auto direction : rayDirections)
                { 
                    Point2d grid_coordinates_toTest = grid_coordinates_curr + direction;
                    if(validateGridCoordinates(grid_coordinates_toTest))
                    {
                        hasExploredNeighbors = informationGain[grid_coordinates_toTest.x][grid_coordinates_toTest.y] == explored || hasExploredNeighbors;
                        hasUnExploredNeighbors = informationGain[grid_coordinates_toTest.x][grid_coordinates_toTest.y] == unexplored || hasUnExploredNeighbors;
                    }

                }

                if(hasExploredNeighbors && hasUnExploredNeighbors)
                {
                    frontierCells.emplace(frontierCells.begin(), x, y);
                }
            }
        }
    }

    float Mapper2D::getValueFunction(Point2d const& grid_coordinates) const
    {
        return valueFunction[grid_coordinates.x][grid_coordinates.y];
    }

    void Mapper2D::printOccupancyMatrix(const geometry_msgs::Vector3 translation, const octomap::OcTree* octree) 
    {
        octomath::Vector3 robot_position(translation.x, translation.y, translation.z);
        ROS_INFO_STREAM("robot_position "<<robot_position);
        octomath::Vector3 occupied_point;
        std::stringstream debugStream_occupancyMatrix;
        std::stringstream debugStream_occupiedPoints;
        std::stringstream debugStream_obstacleDistance;
        std::stringstream debugStream_directions;

        bool isOccupiedFlag;
        // Front
        octomath::Vector3 direction(1, 0, 0);
        debugStream_directions << "Directions\n(1, ?, 0.85)   " << direction << " (1, ?, 0.85)\n";
        isOccupiedFlag = isOccupied(Point2d(translation.x, translation.y), translation.z, octree, occupied_point, direction);
        debugStream_occupancyMatrix << "Occupancy\n0 "<< isOccupiedFlag <<" 0\n";
        if(isOccupiedFlag)
        {
            debugStream_occupiedPoints << "Occupied Points\n       (?, ?)     " << occupied_point << "    (?, ?) \n";
        }
        else
        {
            debugStream_occupiedPoints << "Occupied Points\n       (?, ?)     " << "  free   " << "    (?, ?) \n";
        }
        debugStream_obstacleDistance << "Obstacle distance\n-   "<< std::setprecision(2) << robot_position.distance(occupied_point) <<"   -\n";
        // Left + ROBOT
        direction =  octomath::Vector3(0, 1, 0);
        debugStream_directions << direction << "         ROBOT  ";
        isOccupiedFlag = isOccupied(Point2d(translation.x, translation.y), translation.z, octree, occupied_point, direction);
        debugStream_occupancyMatrix << isOccupiedFlag << " X ";
        if(isOccupiedFlag)
        {
            debugStream_occupiedPoints << occupied_point << "       ROBOT          ";
        }
        else
        {
            debugStream_occupiedPoints << "       free   " << "       ROBOT          ";
        }
        debugStream_obstacleDistance << robot_position.distance(occupied_point) << "  X   ";
        // Right
        direction =  octomath::Vector3(0, -1, 0);
        debugStream_directions << direction << " \n";
        isOccupiedFlag = isOccupied(Point2d(translation.x, translation.y), translation.z, octree, occupied_point, direction);
        debugStream_occupancyMatrix << isOccupiedFlag << "\n";
        if(isOccupiedFlag)
        {
            debugStream_occupiedPoints << occupied_point << " \n ";
        }
        else
        {
            debugStream_occupiedPoints << "  free   " << " \n ";
        }
        debugStream_obstacleDistance << robot_position.distance(occupied_point) << "\n";
        // Back
        direction =  octomath::Vector3(-1, 0, 0);
        debugStream_directions << "(-1, ?, 0.85)  " << direction << " (-1, ?, 0.85)\n";
        isOccupiedFlag = isOccupied(Point2d(translation.x, translation.y), translation.z, octree, occupied_point, direction);
        debugStream_occupancyMatrix << "0 "<< isOccupiedFlag <<" 0";
        //debugStream_occupiedPoints << "     (?, ?)      " << isOccupiedFlag ? occupied_point : "  ---   " << "   (?, ?) \n";
        if(isOccupiedFlag)
        {
            debugStream_occupiedPoints << "     (?, ?)      " << occupied_point << "   (?, ?) \n";
        }
        else
        {
            debugStream_occupiedPoints << "     (?, ?)      " <<  "  free      (?, ?) \n";
        }
        debugStream_obstacleDistance << "-   "<< robot_position.distance(occupied_point) <<"   -";

        ROS_INFO_STREAM(debugStream_occupancyMatrix.str());
        ROS_INFO_STREAM(debugStream_occupiedPoints.str());
        ROS_INFO_STREAM(debugStream_obstacleDistance.str());
        ROS_INFO_STREAM(debugStream_directions.str());
    }

    void Mapper2D::iterateValueFunction(const octomap::OcTree* octree)
    {
        for (int y = 0; y < Mapper2D::value_func_dim; ++y)
        {
            for (int x = 0; x < Mapper2D::value_func_dim; ++x)
            {
                float value = calculateValueFunction_v2(toWorldCoordinates(Point2d(x, y)), octree);
                updateValueFunction(Point2d(x, y), value);
            }
        }
    }

    void Mapper2D::printInformationGainMatrix() const
    {
        std::stringstream debugStream_informationGain;
        debugStream_informationGain << "Information Gain\n";

        for (int y = 0; y < Mapper2D::value_func_dim; ++y)
        {
            for (int x = 0; x < Mapper2D::value_func_dim; ++x)
            {
                debugStream_informationGain << informationGain[x][y] << "  ";
            }
            debugStream_informationGain << "\n";
        }

        ROS_INFO_STREAM(debugStream_informationGain.str());
    }

    void Mapper2D::printValueFunctionMatrix() const
    {
        std::stringstream stream;
        stream << "Value Function\n";

        for (int y = 0; y < Mapper2D::value_func_dim; ++y)
        {
            for (int x = 0; x < Mapper2D::value_func_dim; ++x)
            {
                stream << valueFunction[x][y] << "  ";
            }
            stream << "\n";
        }

        ROS_INFO_STREAM(stream.str());
    }

    void Mapper2D::printOccupancyMatrixRealityCheck(const geometry_msgs::Vector3 translation, const octomap::OcTree* octree) 
    {
        //int z_plane = 1;
        octomap::point3d min (-range, -range, 0);
        octomap::point3d max (range, range, 2);
        for(octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(min,max), endLoop=octree->end_leafs_bbx(); 
            it!= endLoop; 
            ++it)
        {
            octomap::OcTreeNode * temp = octree->search(it.getKey());
            if (temp != NULL && octree->isNodeOccupied(temp))
            {   
                std::cout << it.getCoordinate() << "    ";
                octomath::Vector3 end;
                std::cout << " according to ray it is occupied? "<< isOccupied(Point2d(translation.x, translation.y), translation.z, octree, end) << "\n";
            }   
        }
    }

}