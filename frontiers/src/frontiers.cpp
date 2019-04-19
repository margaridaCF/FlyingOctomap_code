#include <frontiers.h>
#include <neighbors.h>
#include <ordered_neighbors.h>

#define SAVE_LOG 1
#define RUNNING_ROS 1


namespace Frontiers{

    std::ofstream log_file;

    enum State
    {
        free = 0, occupied = 1 , unknown = 2
    };


    bool isInsideGeofence(octomath::Vector3 const&  candidate, geometry_msgs::Point geofence_min, geometry_msgs::Point geofence_max)
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
        return true;
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
        double resolution = octree.getResolution();
        int frontiers_count = 0;
        visualization_msgs::MarkerArray marker_array;
        LazyThetaStarOctree::unordered_set_pointers analyzed;
        int n_id = 0;
        while( !(it == octree.end_leafs_bbx()) && frontiers_count < request.frontier_amount )
        {
            octomath::Vector3 coord = it.getCoordinate();
            currentVoxel = Voxel (coord.x(), coord.y(), coord.z(), it.getSize());
            grid_coordinates_curr = octomath::Vector3(currentVoxel.x, currentVoxel.y, currentVoxel.z);

            State curr_state = getState(grid_coordinates_curr, octree);

            if( curr_state == free )
            {
                LazyThetaStarOctree::unordered_set_pointers neighbors;
                LazyThetaStarOctree::generateNeighbors_frontiers_pointers(neighbors, grid_coordinates_curr, currentVoxel.size, resolution);
                for(std::shared_ptr<octomath::Vector3> n_coordinates : neighbors)
                {
                    auto out = analyzed.insert(std::make_shared<octomath::Vector3> (n_coordinates->x(), n_coordinates->y(), n_coordinates->z() )   );
                    if(!out.second)
                    {
                        continue;
                    }
                    if(!isInsideGeofence(*n_coordinates, request.min, request.max))
                    {
                        // Octomap's bounding box is not accurate at all. The neighbors are outside the bounding box anyway, we just want to recover data from what is inside the bouding box. This is to enforce the geofence.
                        continue;
                    }   
                    State n_state = getState(*n_coordinates, octree);
                    paintState(n_state, *n_coordinates, marker_array, n_id);
                    n_id++;
                    if(n_state == unknown)
                    {
                        frontiers_msgs::VoxelMsg voxel_msg;
                        voxel_msg.size = currentVoxel.size;
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
                            // ROS_INFO_STREAM("[Frontiers] Added last neighbor. ");
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
    }

    octomap::OcTree::leaf_bbx_iterator processFrontiersRequest(octomap::OcTree const& octree, frontiers_msgs::FrontierRequest const& request, frontiers_msgs::FrontierReply & reply, ros::Publisher const& marker_pub, bool publish )
    {
        reply.header.seq = request.header.seq + 1;
        reply.request_id = request.header.seq;
        reply.header.frame_id = request.header.frame_id;
        double resolution = octree.getResolution();
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
            return octomap::OcTree::leaf_bbx_iterator();
        }
        octomap::OcTree::leaf_bbx_iterator it = octree.begin_leafs_bbx(bbxMinKey,bbxMaxKey);
        searchFrontier(octree, it, request, reply, marker_pub, publish);

        reply.success = true;

        return it;
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

    bool isFrontier(octomap::OcTree& octree, octomath::Vector3 const&  candidate) 
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
            LazyThetaStarOctree::generateNeighbors_frontiers_pointers(neighbors, cell_center, voxel_size, resolution);
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