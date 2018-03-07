#include <frontiers.h>
#include <neighbors.h>

namespace Frontiers{

    bool processFrontiersRequest(octomap::OcTree const& octree, frontiers_msgs::FrontierRequest const& request, frontiers_msgs::FrontierReply & reply)
    {
        std::ofstream log;
        log.open ("/ros_ws/src/frontiers/processFrontiersRequest.log");

    	reply.header.seq = request.header.seq + 1;
    	reply.request_id = request.header.seq;
    	reply.header.frame_id = request.header.frame_id;
    	octomath::Vector3  max = octomath::Vector3(request.max.x, request.max.y, request.max.z);
    	octomath::Vector3  min = octomath::Vector3(request.min.x, request.min.y, request.min.z);
        int frontiers_count = 0;

        const std::array<octomath::Vector3, 6> rayDirections ({
                octomath::Vector3(1, 0, 0), // FRONT
                octomath::Vector3(0, 1, 0), // LEFT
                octomath::Vector3(0, -1, 0), // RIGHT
                octomath::Vector3(-1, 0, 0), // BACKWARDS
                octomath::Vector3(0, 0, -1), // UP
                octomath::Vector3(0, 0, 1), // DOWN
            });

        bool hasUnExploredNeighbors;
        octomath::Vector3 grid_coordinates_curr, grid_coordinates_toTest;
        int i;

        float z_max = max.z();
        float z_min = min.z();

        Voxel currentVoxel;
        octomap::OcTreeKey bbxMinKey, bbxMaxKey;
        if(!octree.coordToKeyChecked(min, bbxMinKey) || !octree.coordToKeyChecked(max, bbxMaxKey))
        {
            ROS_ERROR_STREAM("Problems with the octree");
            reply.success = false;
        return reply.success;
        }
        octomap::OcTree::leaf_bbx_iterator it = octree.begin_leafs_bbx(bbxMinKey,bbxMaxKey);
        octomath::Vector3 bbxMin (currentVoxel.x, currentVoxel.y, currentVoxel.z);
        octomath::Vector3 bbxMax (currentVoxel.x, currentVoxel.y, currentVoxel.z);
        double resolution = octree.getResolution();
        while( !(it == octree.end_leafs_bbx()) && frontiers_count < request.frontier_amount)
        {
            octomath::Vector3 coord = it.getCoordinate();
            currentVoxel = Voxel (coord.x(), coord.y(), coord.z(), it.getSize());
            //ROS_WARN_STREAM("At cell " << currentVoxel);
            grid_coordinates_curr = octomath::Vector3(currentVoxel.x, currentVoxel.y, currentVoxel.z);
            if( isExplored(grid_coordinates_curr, octree)
                && !isOccupied(grid_coordinates_curr, octree) ) 
            {
                hasUnExploredNeighbors = false;
                // log << "Looking into " << grid_coordinates_curr << "\n";
                // Gerenate neighbors
                std::unordered_set<std::shared_ptr<octomath::Vector3>> neighbors;
                LazyThetaStarOctree::generateNeighbors_pointers(neighbors, grid_coordinates_curr, currentVoxel.size, resolution);
                for(std::shared_ptr<octomath::Vector3> n_coordinates : neighbors)
                {
                    // log << "[N] " << *n_coordinates << "\n";
                    if(!isOccupied(*n_coordinates, octree))
                    {
                        // octomath::Vector3 temp_v = *n_coordinates;
                        // octomath::Vector3 dummy (2, 4, 3);
                        hasUnExploredNeighbors = !isExplored(*n_coordinates, octree) || hasUnExploredNeighbors;
                    }
                }

                // Comb through looking for unexplored
                if(hasUnExploredNeighbors)
                {
                    frontiers_msgs::VoxelMsg voxel_msg;
                    voxel_msg.xyz_m.x = currentVoxel.x;
                    voxel_msg.xyz_m.y = currentVoxel.y;
                    voxel_msg.xyz_m.z = currentVoxel.z;
                    voxel_msg.size = currentVoxel.size;
                    reply.frontiers.push_back(voxel_msg);
                    frontiers_count++;
                }
            }
            it++;
        }
        reply.frontiers_found = frontiers_count;
        reply.success = true;
        log.close();
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
}