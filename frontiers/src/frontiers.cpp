#include <frontiers.h>
#include <neighbors.h>
#include <marker_publishing_utils.h>

namespace Frontiers{
    bool processFrontiersRequest(octomap::OcTree const& octree, frontiers_msgs::FrontierRequest const& request, frontiers_msgs::FrontierReply & reply, ros::Publisher const& marker_pub)
    {
        // std::ofstream log;
        // log.open ("/ros_ws/src/frontiers/processFrontiersRequest.log");
        // ROS_INFO_STREAM( "Request for " << static_cast<int16_t>(request.frontier_amount) );
        double resolution = octree.getResolution();
        reply.header.seq = request.header.seq + 1;
        reply.request_id = request.header.seq;
        reply.header.frame_id = request.header.frame_id;
        octomath::Vector3  max = octomath::Vector3(request.max.x-resolution, request.max.y-resolution, request.max.z-resolution);
        octomath::Vector3  min = octomath::Vector3(request.min.x+resolution, request.min.y+resolution, request.min.z+resolution);
        int frontiers_count = 0;
        octomath::Vector3 current_position (request.current_position.x, request.current_position.y, request.current_position.z);

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
            ROS_ERROR_STREAM("[Frontiers] Problems with the octree");
            reply.success = false;
        return reply.success;
        }
        octomap::OcTree::leaf_bbx_iterator it = octree.begin_leafs_bbx(bbxMinKey,bbxMaxKey);
        // octomath::Vector3 bbxMin (currentVoxel.x, currentVoxel.y, currentVoxel.z);
        // octomath::Vector3 bbxMax (currentVoxel.x, currentVoxel.y, currentVoxel.z);
        while( !(it == octree.end_leafs_bbx()) && frontiers_count < request.frontier_amount)
        {
            octomath::Vector3 coord = it.getCoordinate();
            currentVoxel = Voxel (coord.x(), coord.y(), coord.z(), it.getSize());
            // ROS_WARN_STREAM("Voxel size " << currentVoxel.size);
            grid_coordinates_curr = octomath::Vector3(currentVoxel.x, currentVoxel.y, currentVoxel.z);
            if( isExplored(grid_coordinates_curr, octree)
                && !isOccupied(grid_coordinates_curr, octree) 
                && meetsOperationalRequirements(it.getSize()*2, grid_coordinates_curr, request.min_distance, current_position, octree, request.safety_margin, marker_pub)) 
            {
                hasUnExploredNeighbors = false;
                // log << "Looking into " << grid_coordinates_curr << "\n";
                // Gerenate neighbors
                std::unordered_set<std::shared_ptr<octomath::Vector3>> neighbors;
                LazyThetaStarOctree::generateNeighbors_pointers(neighbors, grid_coordinates_curr, currentVoxel.size, resolution);
                for(std::shared_ptr<octomath::Vector3> n_coordinates : neighbors)
                {
                    if(!isOccupied(*n_coordinates, octree))
                    {
                        hasUnExploredNeighbors = !isExplored(*n_coordinates, octree) || hasUnExploredNeighbors;
                    }
                }

                // Comb through looking for unexplored
                if(hasUnExploredNeighbors)
                {

                    if (isOccupied(coord, octree))
                    {
                        ROS_ERROR_STREAM("[LTStar] Frontier " << coord << " is occupied.");
                        return false;   
                    } 
                    if (!isExplored(coord, octree))
                    {
                        ROS_ERROR_STREAM("[LTStar] Frontier " << coord << " is unknown.");
                        return false;   
                    } 

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
        // log.close();
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

    bool isFrontierTooCloseToObstacles(octomath::Vector3 const& frontier, double safety_margin, octomap::OcTree const& octree, ros::Publisher const& marker_pub)
    {
        rviz_interface::publish_deleteAll(marker_pub);
        // ROS_WARN_STREAM("[Frontier] Checking neighboring obstacles for candidate frontier " << frontier);
        geometry_msgs::Point candidate_frontier;
        rviz_interface::init_point(candidate_frontier, frontier.x(), frontier.y(), frontier.z());
        rviz_interface::publish_marker_safety_margin(candidate_frontier, safety_margin, marker_pub, 101);
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
        while( !(it == octree.end_leafs_bbx()) )
        {
            octomath::Vector3 coord = it.getCoordinate();
            if(isOccupied(coord, octree))
            {
                // ROS_WARN_STREAM("[Frontiers] " << coord << " is occupied.");
                rviz_interface::publish_voxel_free_occupied(coord, true, marker_pub, id, it.getSize());
                // ROS_WARN_STREAM("[Frontier] Candidate frontier had obstacle as neighbor " << frontier);
                return true;
            }
            else
            {
                // ROS_WARN_STREAM("[Frontiers] " << coord << " is free.");
                rviz_interface::publish_voxel_free_occupied(coord, false, marker_pub, id, it.getSize());
            }
            it++;
            id++;
        }
        return false;
    }

    bool meetsOperationalRequirements(double voxel_size, octomath::Vector3 const&  candidate, double min_distance, octomath::Vector3 const& current_position, octomap::OcTree const& octree, double safety_distance, ros::Publisher const& marker_pub)
    {
        // ROS_INFO_STREAM("[Frontiers] meetsOperationalRequirements - voxel_size: " << voxel_size << "; candidate: " << candidate << "; min_distance: " << min_distance << "; current_position:" << current_position);
        // Operation restrictions
        if(voxel_size > 2)
        {
            // ROS_INFO_STREAM("[Frontiers] Rejected " << candidate << " because it's too big " << voxel_size);
            return false; // the neighbors are so spread out it is bound to have unexplored ones and other voxels will capture more accuratly this frontier
        }
        if(candidate.distance(current_position) <= min_distance)
        {
            // ROS_INFO_STREAM("[Frontiers] Rejected " << candidate << " because it's too close (" << candidate.distance(current_position) << "m) to current_position " << current_position);
            return false; // below navigation precision
        }
        if(LazyThetaStarOctree::getCellCenter(candidate, octree) == LazyThetaStarOctree::getCellCenter(current_position, octree) )
        {// start and end in same voxel
            return false;
        }
        if(isFrontierTooCloseToObstacles(candidate, safety_distance, octree, marker_pub))
        {
            return false;
        }
        // ROS_INFO_STREAM("[Frontiers]meetsOperationalRequirements - Approved");
        return true;
    }

    bool isFrontier(octomap::OcTree& octree, octomath::Vector3 const&  candidate) 
    {
        ROS_WARN_STREAM("[fronties] For candidate " << candidate);
        bool is_explored = isExplored(candidate, octree);
        if(!is_explored)
        {
            ROS_WARN_STREAM("[fronties]   not explored.");
            return false;
        }
        bool is_occupied = isOccupied(candidate, octree);
        if(is_occupied)
        {
            ROS_WARN_STREAM("[fronties]   is occupied.");
            return false;
        }
        std::unordered_set<std::shared_ptr<octomath::Vector3>> neighbors;
        double resolution = octree.getResolution();
        int tree_depth = octree.getTreeDepth();
        octomap::OcTreeKey key = octree.coordToKey(candidate);
        int depth = LazyThetaStarOctree::getNodeDepth_Octomap(key, octree);
        double voxel_size = ((tree_depth + 1) - depth) * resolution;
        LazyThetaStarOctree::generateNeighbors_pointers(neighbors, candidate, voxel_size, resolution);
        bool hasUnExploredNeighbors = false;
        for(std::shared_ptr<octomath::Vector3> n_coordinates : neighbors)
        {
            if(!isOccupied(*n_coordinates, octree))
            {
                hasUnExploredNeighbors = !isExplored(*n_coordinates, octree) || hasUnExploredNeighbors;
                if(!isExplored(*n_coordinates, octree))
                {
                    ROS_WARN_STREAM("[fronties]   Unknown neighbors: (" << n_coordinates->x() << "," << n_coordinates->y() << " ," << n_coordinates->z() << " )");
                }
            }
        }
        if(hasUnExploredNeighbors)
        {
            ROS_WARN_STREAM("[fronties]   still has unknown neighbors.");
            return true;
        }
        else
        {
            ROS_WARN_STREAM("[fronties]   no unknown neighbors.");
            return false;
        }
    }
}