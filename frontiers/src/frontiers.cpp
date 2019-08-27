#include <frontiers.h>
#include <neighbors.h>
#include <ordered_neighbors.h>

// #define SAVE_LOG 1
// #define RUNNING_ROS 1


namespace Frontiers{

    enum State
    {
        free = 0, occupied = 1 , unknown = 2
    };
    int n_id;


    bool isInsideGeofence(octomath::Vector3 const&  candidate, geometry_msgs::Point geofence_min, geometry_msgs::Point geofence_max)
    {
        if(candidate.x() < geofence_min.x 
            || candidate.y() < geofence_min.y 
            || candidate.x() < geofence_min.y 
            || candidate.x() > geofence_max.x 
            || candidate.y() > geofence_max.y  
            || candidate.z() > geofence_max.z)
        {
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

    void searchFrontier(octomap::OcTree const& octree, Circulator & it, frontiers_msgs::FindFrontiers::Request  &request,
        frontiers_msgs::FindFrontiers::Response &reply, ros::Publisher const& marker_pub, bool publish)
    {
        octomath::Vector3 current_position (request.current_position.x, request.current_position.y, request.current_position.z);
        
        frontiers_msgs::VoxelMsg current_position_voxel_msg;
        current_position_voxel_msg.xyz_m.x = current_position.x();
        current_position_voxel_msg.xyz_m.y = current_position.y();
        current_position_voxel_msg.xyz_m.z = current_position.z();
        OrderedNeighbors allNeighbors (current_position_voxel_msg);
        octomath::Vector3 grid_coordinates_curr, grid_coordinates_toTest;
        Voxel currentVoxel;
        double resolution = octree.getResolution();
        int frontiers_count = 0;
        visualization_msgs::MarkerArray marker_array;
        LazyThetaStarOctree::unordered_set_pointers analyzed;

        if(it.isFinished())
        {
            ROS_ERROR_STREAM("[Frontier] End of iterator");
        }

        if(!(frontiers_count < request.frontier_amount ))
        {
            ROS_ERROR_STREAM("[Frontier] Already found " << frontiers_count << " frontiers out of " << request.frontier_amount);

        }
        while( !(it.isFinished()) && frontiers_count < request.frontier_amount )
        {
            octomath::Vector3 coord = it.getCoordinate();
            currentVoxel = Voxel (coord.x(), coord.y(), coord.z(), it.getSize());
            grid_coordinates_curr = octomath::Vector3(currentVoxel.x, currentVoxel.y, currentVoxel.z);

            State curr_state = getState(grid_coordinates_curr, octree);

            if( curr_state == free )
            {
                LazyThetaStarOctree::unordered_set_pointers neighbors;
                LazyThetaStarOctree::generateNeighbors_frontiers_pointers(neighbors, grid_coordinates_curr, currentVoxel.size, resolution);
                std::list<frontiers_msgs::VoxelMsg> neighborhood;
                double surface_neighborhood = 0;
                for(std::shared_ptr<octomath::Vector3> n_coordinates : neighbors)
                {
                    // Skip neighbours that have already been analyzed. The same neighbours shows up multiple times because the list contains the centers of the sparse voxels starting from the regular grid. 
                    auto out = analyzed.insert(std::make_shared<octomath::Vector3> (n_coordinates->x(), n_coordinates->y(), n_coordinates->z() )   );
                    if(!out.second) continue;
                    // Octomap's bounding box is not accurate at all. The neighbors are outside the bounding box anyway, we just want to recover data from what is inside the bouding box. This is to enforce the geofence.
                    if(!isInsideGeofence(*n_coordinates, request.min, request.max))continue;   
                    State n_state = getState(*n_coordinates, octree);
                    if(n_state == unknown)
                    {
                        #ifdef RUNNING_ROS
                            paintState(n_state, *n_coordinates, marker_array, n_id);
                        #endif
                        n_id++;
                        frontiers_msgs::VoxelMsg voxel_msg;
                        voxel_msg.size = currentVoxel.size;
                        voxel_msg.xyz_m.x = n_coordinates->x();
                        voxel_msg.xyz_m.y = n_coordinates->y();
                        voxel_msg.xyz_m.z = n_coordinates->z();
                        frontiers_count++;
                        neighborhood.insert(neighborhood.begin(),voxel_msg);
                    }
                    else if(n_state == occupied)
                    {
                        surface_neighborhood += currentVoxel.size;
                    }
                }
                for (std::list<frontiers_msgs::VoxelMsg>::iterator it_n = neighborhood.begin(); it_n != neighborhood.end(); ++it_n)
                {
                    frontiers_msgs::VoxelMsg voxel_msg;
                    voxel_msg = *it_n;
                    voxel_msg.occupied_neighborhood = surface_neighborhood;
                    allNeighbors.insert(voxel_msg);
                }
                if( frontiers_count == request.frontier_amount) break;
            }
            it.increment();
        }
        #ifdef RUNNING_ROS
        marker_pub.publish(marker_array);
        #endif

        allNeighbors.buildMessageList(request.frontier_amount, reply);
        reply.success = frontiers_count > 0;
        reply.global_search_it = it.getCounter();
    }

    Circulator processFrontiersRequest(octomap::OcTree const& octree, frontiers_msgs::FindFrontiers::Request  &request,
        frontiers_msgs::FindFrontiers::Response &reply, ros::Publisher const& marker_pub, bool publish )
    {
        double resolution = octree.getResolution();
        octomath::Vector3  max = octomath::Vector3(request.max.x-resolution, request.max.y-resolution, request.max.z-resolution);
        octomath::Vector3  min = octomath::Vector3(request.min.x+resolution, request.min.y+resolution, request.min.z+resolution);
        octomath::Vector3 current_position (request.current_position.x, request.current_position.y, request.current_position.z);

        int i;

        float z_max = max.z();
        float z_min = min.z();
        Circulator it (octree, max, min, request.global_search_it);
        n_id = 100;
        searchFrontier(octree, it, request, reply, marker_pub, publish);
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
                        ROS_ERROR_STREAM("[isFrontier] Unknown neighbor is " << *n_coordinates);
                        is_frontier = true;
                    }
                    else
                    {
                    }
                }
            }
        }
        return is_frontier;
    }
}