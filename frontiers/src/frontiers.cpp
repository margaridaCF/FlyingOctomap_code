#include <frontiers.h>
#include <neighbors.h>
#include <ordered_neighbors.h>

namespace Frontiers{

    std::ofstream log_file;

    enum State
    {
        free = 0, occupied = 1 , unknown = 2
    };

    void calculate_closer_position(octomath::Vector3 & sensing_position, octomath::Vector3 const& n_coordinates, double const safety_margin)
    {
        sensing_position = sensing_position - n_coordinates;
        sensing_position.normalize();
        sensing_position = sensing_position * safety_margin;
        sensing_position = n_coordinates + sensing_position;
    }

    bool isCenterGoodGoal(double voxel_side, double octree_resolution, double sensing_distance)
    {
        return voxel_side/2 + octree_resolution/2 < sensing_distance ;
    }

    State getState(octomath::Vector3 const& grid_coordinates_toTest, octomap::OcTree const& octree)
    {
        octomap::OcTreeNode* result = octree.search(grid_coordinates_toTest.x(), grid_coordinates_toTest.y(), grid_coordinates_toTest.z());
        if(result == NULL){
            return unknown;
        }
        else
        {
            if (octree.isNodeOccupied(result)) return occupied;
            else return free;
        }
    }


    void paintState(State state, octomath::Vector3 const& position, visualization_msgs::MarkerArray & marker_array, int id)
    {
        float red = 0.0f;
        float green = 0.0f;
        float blue = 0.0f;
        if(state == free)
        {
            green = 1.0f;
            red = 1.0f;
            return;
        }
        else if( state == occupied)
        {
            red = 1.0f;
        }
        else if(state == unknown)
        {
            blue = 1.0f;
        }
        visualization_msgs::Marker marker;
        rviz_interface::build_small_marker(position, marker, red, green, blue, "neighbor", id, 0.1);
        marker_array.markers.push_back(marker);
    }

    void searchFrontier(octomap::OcTree const& octree, octomap::OcTree::leaf_bbx_iterator & it, frontiers_msgs::FrontierRequest const& request, frontiers_msgs::FrontierReply & reply, ros::Publisher const& marker_pub, bool publish)
    {
        #ifdef SAVE_LOG
        log_file.open ("/ros_ws/src/data/current/frontiers.log", std::ofstream::app);
        #endif
        octomath::Vector3 current_position (request.current_position.x, request.current_position.y, request.current_position.z);
        
        #ifdef BASELINE 
        frontiers_msgs::VoxelMsg current_position_voxel_msg;
        current_position_voxel_msg.xyz_m.x = current_position.x();
        current_position_voxel_msg.xyz_m.y = current_position.y();
        current_position_voxel_msg.xyz_m.z = current_position.z();
        OrderedNeighbors allNeighbors (current_position_voxel_msg);
        #endif
        octomath::Vector3 grid_coordinates_curr, grid_coordinates_toTest;
        Voxel currentVoxel;
        // bool is_frontier;
        double resolution = octree.getResolution();
        int frontiers_count = 0;
        visualization_msgs::MarkerArray marker_array;
        LazyThetaStarOctree::unordered_set_pointers analyzed;
        int n_id = 0;
        while( !(it == octree.end_leafs_bbx()) && frontiers_count < request.frontier_amount)
        {
            bool use_center_as_goal = isCenterGoodGoal(it.getSize(), resolution, request.sensing_distance);
            octomath::Vector3 coord = it.getCoordinate();
            currentVoxel = Voxel (coord.x(), coord.y(), coord.z(), it.getSize());
            grid_coordinates_curr = octomath::Vector3(currentVoxel.x, currentVoxel.y, currentVoxel.z);


            State curr_state = getState(grid_coordinates_curr, octree);
            // paintState(curr_state, grid_coordinates_curr, marker_array, n_id);

            // float red = 0.0f;
            // float green = 0.0f;
            // float blue = 0.0f;
            // visualization_msgs::Marker marker;
            // rviz_interface::build_small_marker(grid_coordinates_curr, marker, red, green, blue, "candidate", 1000, 0.1);
            // marker_array.markers.push_back(marker);


            if( curr_state == free )
            // if( isExplored(grid_coordinates_curr, octree)
            //     && !isOccupied(grid_coordinates_curr)
                // && meetsOperationalRequirements(grid_coordinates_curr, request.min_distance, current_position, octree, request.safety_margin, request.min, request.max, marker_pub, publish)
                // ) 
            {
                // ROS_WARN_STREAM("[frontiers] Frontier candidate " << coord << " size " << it.getSize() );
                // is_frontier = false;
                // Generate neighbors
                // unordered_set_pointers neighbors;
                // LazyThetaStarOctree::generateNeighbors_frontiers_pointers(neighbors, grid_coordinates_curr, currentVoxel.size, resolution, request.sensor_angle);
                

                LazyThetaStarOctree::unordered_set_pointers neighbors;
                LazyThetaStarOctree::generateNeighbors_frontiers_pointers(neighbors, grid_coordinates_curr, currentVoxel.size, resolution, request.sensor_angle);
                

                for(std::shared_ptr<octomath::Vector3> n_coordinates : neighbors)
                {
                    auto out = analyzed.insert(std::make_shared<octomath::Vector3> (n_coordinates->x(), n_coordinates->y(), n_coordinates->z() )   );
                    if(!out.second)
                    {
                        // ROS_INFO_STREAM("Been there, done that.");
                        continue;
                    }
                    if(!meetsOperationalRequirements(*n_coordinates, request.min_distance, current_position, octree, request.safety_margin, request.min, request.max, marker_pub, publish))
                    {
                        // Octomap's bounding box is not accurate at all. The neighbors are outside the bounding box anyway, we just want to recover data from what is inside the bouding box. This is to enforce the geofence.
                        continue;
                    }   
                    // ROS_INFO_STREAM("[frontiers] Neighbor " << *n_coordinates);

                    State n_state = getState(*n_coordinates, octree);
                    paintState(n_state, *n_coordinates, marker_array, n_id);
                    n_id++;
                    if(n_state == unknown)
                    {
                        // ROS_INFO_STREAM("Unknown ");
                        ROS_INFO_STREAM("[Frontiers] Frontier candidate " << coord << " size " << it.getSize() << ". Unknown neighbor is " << *n_coordinates);
                        frontiers_msgs::VoxelMsg voxel_msg;
                        voxel_msg.size = currentVoxel.size;
                        #ifdef RUNNING_ROS
                        // ROS_INFO_STREAM("[Frontiers] Adding marker for (" << grid_coordinates_curr.x() << ", " 
                            // << grid_coordinates_curr.y() << ", " << grid_coordinates_curr.z() << ")");
                        // rviz_interface::publish_sensing_position(n_coordinates, frontiers_count, marker_array);
                        #endif
                        voxel_msg.xyz_m.x = n_coordinates->x();
                        voxel_msg.xyz_m.y = n_coordinates->y();
                        voxel_msg.xyz_m.z = n_coordinates->z();
                        #ifdef BASELINE 
                        allNeighbors.insert(voxel_msg);
                        #else
                        reply.frontiers.push_back(voxel_msg);
                        frontiers_count++;
                        if( frontiers_count == request.frontier_amount)
                        {
                            // ROS_INFO_STREAM("[Frontiers] Selecting as frontier " << grid_coordinates_curr << ". Distance to current position " << current_position << " is " << grid_coordinates_curr.distance(current_position));
                            break;
                        }
                        #endif
                    }
                }
            }
            it++;
        }

        #ifdef RUNNING_ROS
        marker_pub.publish(marker_array);
        #endif

        #ifdef BASELINE
        allNeighbors.buildMessageList(request.frontier_amount, reply);
        #else
        reply.frontiers_found = frontiers_count;
        #endif
        #ifdef SAVE_LOG
        log_file.close();
        #endif
    }

    bool processFrontiersRequest(octomap::OcTree const& octree, frontiers_msgs::FrontierRequest const& request, frontiers_msgs::FrontierReply & reply, ros::Publisher const& marker_pub, bool publish )
    {

        // std::ofstream log;
        // log.open ("/ros_ws/src/frontiers/processFrontiersRequest.log");
        double resolution = octree.getResolution();
        double unknown_neighbor_distance = request.safety_margin + (resolution/2);
        reply.header.seq = request.header.seq + 1;
        reply.request_id = request.header.seq;
        reply.header.frame_id = request.header.frame_id;
        octomath::Vector3  max = octomath::Vector3(request.max.x-resolution, request.max.y-resolution, request.max.z-resolution);
        octomath::Vector3  min = octomath::Vector3(request.min.x+resolution, request.min.y+resolution, request.min.z+resolution);
        octomath::Vector3 current_position (request.current_position.x, request.current_position.y, request.current_position.z);

        int i;

        float z_max = max.z();
        float z_min = min.z();

        octomap::OcTreeKey bbxMinKey, bbxMaxKey;
        if(!octree.coordToKeyChecked(min, bbxMinKey) || !octree.coordToKeyChecked(max, bbxMaxKey))
        {
            ROS_ERROR_STREAM("[Frontiers] Problems with the octree");
            reply.success = false;
            return reply.success;
        }
        octomap::OcTree::leaf_bbx_iterator it = octree.begin_leafs_bbx(bbxMinKey,bbxMaxKey);
        searchFrontier(octree, it, request, reply, marker_pub, publish);

        reply.success = true;

        return reply.success;
    }
    

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

    
    bool isFrontierTooCloseToObstacles(octomath::Vector3 const& frontier, double safety_margin, octomap::OcTree const& octree, ros::Publisher const& marker_pub, bool publish)
    {
        std::vector<Voxel> explored_space;
        #ifdef RUNNING_ROS
        if(publish)
        { 
            rviz_interface::publish_deleteAll(marker_pub);
        }
        #endif
        // ROS_WARN_STREAM("[Frontier] Checking neighboring obstacles for candidate frontier " << frontier);
        octomath::Vector3  min = octomath::Vector3(frontier.x() - safety_margin, frontier.y() - safety_margin, frontier.z() - safety_margin);
        octomath::Vector3  max = octomath::Vector3(frontier.x() + safety_margin, frontier.y() + safety_margin, frontier.z() + safety_margin);
        octomap::OcTreeKey bbxMinKey, bbxMaxKey;
        if(!octree.coordToKeyChecked(min, bbxMinKey) || !octree.coordToKeyChecked(max, bbxMaxKey))
        {
            ROS_ERROR_STREAM("[Frontiers] Problems with the octree");
            return true;
        }
        int id = 102;
        
        octomap::OcTree::leaf_bbx_iterator it = octree.begin_leafs_bbx(bbxMinKey,bbxMaxKey);   
        visualization_msgs::Marker marker;
        visualization_msgs::MarkerArray known_space_markers;
        while( !(it == octree.end_leafs_bbx()) )
        {
            octomath::Vector3 coord = it.getCoordinate();
            explored_space.push_back(   Voxel ( coord.x(), coord.y(), coord.z(), it.getSize() )   );
            if(isOccupied(coord, octree))
            {
                // ROS_WARN_STREAM("[Frontiers] " << coord << " is occupied.");
                #ifdef RUNNING_ROS
                if (publish) rviz_interface::publish_voxel_free_occupied(coord, true, marker_pub, id, it.getSize(), marker);
                #endif
                // ROS_WARN_STREAM("[Frontier] Candidate frontier had obstacle as neighbor " << frontier);
                return true;
            }
            else
            {
                // ROS_WARN_STREAM("[Frontiers] " << coord << " is free.");
                #ifdef RUNNING_ROS
                if (publish) rviz_interface::publish_voxel_free_occupied(coord, false, marker_pub, id, it.getSize(), marker);
                #endif
            }
            known_space_markers.markers.push_back(marker);
            it++;
            id++;
        }
        if (publish) marker_pub.publish(known_space_markers);
        return false;
    }

    bool meetsOperationalRequirements(octomath::Vector3 const&  candidate, double min_distance, octomath::Vector3 const& current_position, octomap::OcTree const& octree, double safety_distance, geometry_msgs::Point geofence_min, geometry_msgs::Point geofence_max, ros::Publisher const& marker_pub, bool publish)
    {
        if(candidate.x() < geofence_min.x 
            || candidate.y() < geofence_min.y 
            || candidate.x() < geofence_min.y 
            || candidate.x() > geofence_max.x 
            || candidate.y() > geofence_max.y  
            || candidate.z() > geofence_max.z)
        {
            
#ifdef SAVE_LOG            
            log_file << "[Frontiers] Rejected " << candidate << " because it is outside geofence " << std::endl;
#endif
            return false;
        }
//         // Operation restrictions
//         if(candidate.distance(current_position) <= min_distance)
//         {
// #ifdef SAVE_LOG            
//             log_file << "[Frontiers] Rejected " << candidate << " because it's too close (" << candidate.distance(current_position) << "m) to current_position " << current_position << std::endl;
// #endif
//             return false; // below navigation precision
//         }
//         if(isFrontierTooCloseToObstacles(candidate, safety_distance, octree, marker_pub, publish))
//         {
// #ifdef SAVE_LOG            
//             log_file <<"[Frontiers] Rejected " << candidate << " because goal is too close to obstacles" << std::endl;
// #endif
//             return false;
//         }
        return true;
    }

    bool isFrontier(octomap::OcTree& octree, octomath::Vector3 const&  candidate, double sensor_angle) 
    {
        double resolution = octree.getResolution(); 
        int tree_depth = octree.getTreeDepth(); 
        octomap::OcTreeKey key = octree.coordToKey(candidate); 
        int depth = LazyThetaStarOctree::getNodeDepth_Octomap(key, octree); 
        octomath::Vector3 cell_center = octree.keyToCoord(key, depth); 
        double voxel_size = ((tree_depth + 1) - depth) * resolution; 

// #ifdef SAVE_LOG
//         log_file.open ("/ros_ws/src/data/current/frontiers.log", std::ofstream::app);
//         log_file << " == " << candidate << " == " << std::endl;
//         log_file << " Voxel center " << cell_center << std::endl;
// #endif

        bool is_frontier = false;
        if( isExplored(cell_center, octree)
            && !isOccupied(cell_center, octree) ) 
        {
            // Generate neighbors
            LazyThetaStarOctree::unordered_set_pointers neighbors;
            LazyThetaStarOctree::generateNeighbors_frontiers_pointers(neighbors, cell_center, voxel_size, resolution, sensor_angle);
            for(std::shared_ptr<octomath::Vector3> n_coordinates : neighbors)
            {
                if(!isOccupied(*n_coordinates, octree))
                {
                    if(!isExplored(*n_coordinates, octree))
                    {
// #ifdef SAVE_LOG
//                         log_file << "[isFrontier] Neighbor " << *n_coordinates << " UNKNOWN! "  << std::endl;
// #endif
                        ROS_ERROR_STREAM("[isFrontier] Unknown neighbor is " << *n_coordinates);
                        is_frontier = true;
                    }
                    else
                    {
// #ifdef SAVE_LOG
//                 log_file << "[isFrontier] Neighbor " << *n_coordinates << " free. " << std::endl;
// #endif 
                    }
                }
                else
                {
// #ifdef SAVE_LOG
//                 log_file << "[isFrontier] Neighbor " << *n_coordinates << " occupied. " << std::endl;
// #endif

                }
            }
        }
#ifdef SAVE_LOG
        log_file.close();
#endif
        return is_frontier;
    }
}