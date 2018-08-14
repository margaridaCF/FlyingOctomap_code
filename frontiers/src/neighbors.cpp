#include <neighbors.h>

namespace LazyThetaStarOctree{
	bool addIfUnique(std::unordered_set<std::shared_ptr<octomath::Vector3>> & neighbors, float x, float y, float z )
	{
        octomath::Vector3 toInsert (x, y, z);
        addIfUnique (neighbors, toInsert);
	}

    bool addIfUnique(std::unordered_set<std::shared_ptr<octomath::Vector3>> & neighbors, octomath::Vector3 & toInsert )
    {
        // This is the creation point for neighbors
        // Raw pointers were chosen because they go out of scope since (for the Lazy Theta Star)
        // objects are produce inside finding neighbors methods and later processed in another method
        // octomath::Vector3* toInsert = new octomath::Vector3(x, y, z);
        std::shared_ptr<octomath::Vector3> toInsert_ptr = std::make_shared<octomath::Vector3> (toInsert);
        if(!neighbors.insert(toInsert_ptr).second)
        {
            ROS_ERROR_STREAM("Could not insert coordinates of neighbor, this should not happen - contact maintainer. @AddIfUnique"); 
        }
    }

    // bool addIfUnique_new(std::unordered_set<std::shared_ptr<octomath::Vector3>> & neighbors, std::shared_ptr<octomath::Vector3> toInsert )
    // {
    //     if(!neighbors.insert(toInsert).second)
    //     {
    //         ROS_ERROR_STREAM("Could not insert coordinates of neighbor, this should not happen - contact maintainer. @AddIfUnique"); 
    //     }
    // }

    // double find_voxel_side(octomap::OcTree const& octree, std::shared_ptr<octomath::Vector3> & coordinates, double const* lookup_table)
    // {
    //     octomap::OcTreeKey key = octree.coordToKey(*coordinates);
    //     // find depth of cell
    //     // ROS_WARN_STREAM("Calling getNodeDepth from 173");

    //     int depth = getNodeDepth_Octomap(key, octree);

    //     return findSideLenght(octree.getTreeDepth(), depth, lookup_table);
    // }


    // int findJump(octomap::OcTree const& octree, double const* lookup_table, std::shared_ptr<octomath::Vector3> coordinates, int current_i, double resolution)
    // {
    //     double voxel_side;
    //     try
    //     {
    //         double voxel_side = find_voxel_side(octree, coordinates, lookup_table);
    //     }
    //     catch(const std::out_of_range& oor)
    //     {
    //         // ROS_ERROR_STREAM("[Frontiers] Candidate " << req.candidate << " is in unknown space.");
    //         voxel_side = resolution;
    //     }
    //     int jump = (voxel_side / resolution);
    //     int next_i = jump + current_i;
    //     if(voxel_side > resolution)
    //     {
    //         ROS_WARN_STREAM(*coordinates << " --> " << " voxel_side " << voxel_side << " next = " << jump << " + " << current_i << " = " << next_i);
    //     }

    //     return next_i;
    // }

    // TODO reduce neighbor number by finding cell center and removing duplicates
    void generateNeighbors_pointers(std::unordered_set<std::shared_ptr<octomath::Vector3>> & neighbors, 
        octomath::Vector3 const& center_coords, 
        float node_size, float resolution, bool debug_on/* = false*/)
    {
        // mem_alloc_nanosecs = 0;
        // auto start = std::chrono::high_resolution_clock::now();
        int neighbor_sequence_cell_count = node_size / resolution;
        float frontier_offset = (node_size/2.f);         
        int i;
        // int neighbor_count = 0;
        float extra = resolution / 2.f;     

        float x_start  = center_coords.x() - frontier_offset + extra;
        float y_start  = center_coords.y() - frontier_offset + extra;     
        float z_start  = center_coords.z() - frontier_offset + extra;                      
        // Left right
        float right_x = center_coords.x() + frontier_offset + extra; 
        float left_x  = center_coords.x() - frontier_offset - extra; 
        // Front back
        float front_y  = center_coords.y() + frontier_offset + extra; 
        float back_y   = center_coords.y() - frontier_offset - extra; 
        // Up down      
        float up_z     = center_coords.z() + frontier_offset + extra; 
        float down_z   = center_coords.z() - frontier_offset - extra;

        // optimized
        octomath::Vector3* toInsert;
        bool isInserted;
        for(int i = 0; i < neighbor_sequence_cell_count; i++)
        {
            // int j = 0;
            for(int j = 0; j < neighbor_sequence_cell_count; j++)
            {
                // Left Right
                addIfUnique(neighbors, left_x,                      y_start + (i * resolution),  z_start + (j * resolution));
                addIfUnique(neighbors, right_x,                     y_start + (i * resolution),  z_start + (j * resolution));

                // Front Back
                addIfUnique(neighbors, x_start + (i * resolution),  front_y,                     z_start + (j * resolution));
                addIfUnique(neighbors, x_start + (i * resolution),  back_y,                      z_start + (j * resolution));

                // Up Down
                addIfUnique(neighbors, x_start + (i * resolution),  y_start + (j * resolution),  up_z);
                addIfUnique(neighbors, x_start + (i * resolution),  y_start + (j * resolution),  down_z);

                // neighbor_count++;
            }
        }
        // auto finish = std::chrono::high_resolution_clock::now();
        // auto time_span = finish - start;
        // int total_nanosecs = std::chrono::duration_cast<std::chrono::nanoseconds>(time_span).count();

        // ROS_WARN_STREAM("Total time " << total_nanosecs << " nanoseconds.");
        // double percent = (mem_alloc_nanosecs * 100 / total_nanosecs);
        // ROS_WARN_STREAM("mem_alloc took " << mem_alloc_nanosecs << " - " << percent << "%");
    }

    // void generateNeighbors_pointers_sparse(octomap::OcTree const& octree, double const* lookup_table, std::unordered_set<std::shared_ptr<octomath::Vector3>> & neighbors, 
    //     octomath::Vector3 const& center_coords, 
    //     float node_size, float resolution, bool debug_on/* = false*/)
    // {
    //     int neighbor_sequence_cell_count = node_size / resolution;
    //     float frontier_offset = (node_size/2.f);         
    //     int i;
    //     // int neighbor_index = 0;
    //     float extra = resolution / 2.f;     

    //     float x_start  = center_coords.x() - frontier_offset + extra;
    //     float y_start  = center_coords.y() - frontier_offset + extra;     
    //     float z_start  = center_coords.z() - frontier_offset + extra;                      
    //     // Left right
    //     float right_x = center_coords.x() + frontier_offset + extra; 
    //     float left_x  = center_coords.x() - frontier_offset - extra; 
    //     // Front back
    //     float front_y  = center_coords.y() + frontier_offset + extra; 
    //     float back_y   = center_coords.y() - frontier_offset - extra; 
    //     // Up down      
    //     float up_z     = center_coords.z() + frontier_offset + extra; 
    //     float down_z   = center_coords.z() - frontier_offset - extra;

    //     std::shared_ptr<octomath::Vector3> coord_ptr ;
    //     double voxel_side;
    //     int jump;
    //     int left_x_i = -1;
    //     int left_x_j = -1;
    //     int right_x_i = -1;
    //     int right_x_j = -1;
    //     int front_y_i = -1;
    //     int front_y_j = -1;
    //     int back_y_i = -1;
    //     int back_y_j = -1;
    //     int up_z_i = -1;
    //     int up_z_j = -1;
    //     int down_z_i = -1;
    //     int down_z_j = -1;
    //     // optimized
    //     octomath::Vector3* toInsert;
    //     bool isInserted;
    //     // ROS_WARN_STREAM(" resolution " << resolution);
    //     for(int i_ = 0; i_ < neighbor_sequence_cell_count; i_++)
    //     {
    //         // int j = 0;
    //         for(int j_ = 0; j_ < neighbor_sequence_cell_count; j_++)
    //         {
    //             // Left
    //             if(   (left_x_i < i_) || (left_x_j < j_)   )
    //             {
    //                 coord_ptr = std::make_shared<octomath::Vector3> (octomath::Vector3 (left_x, y_start + (i_ * resolution), z_start + (j_ * resolution)));
    //                 left_x_i = findJump(octree, lookup_table, coord_ptr, i_, resolution);
    //                 left_x_j = findJump(octree, lookup_table, coord_ptr, j_, resolution);
    //                 addIfUnique_new(neighbors, coord_ptr);
    //                 // ROS_WARN_STREAM("[" << i_ << "]  left_x_i " << left_x_i);
    //             }
    //             // Right
    //             // addIfUnique(neighbors, right_x,                     y_start + (i * resolution),  z_start + (j * resolution)); 
    //             if(    (right_x_i < i_ ) || (right_x_j < j_ )   )
    //             {
    //                 coord_ptr = std::make_shared<octomath::Vector3> (    octomath::Vector3 ( right_x, y_start + (i_ * resolution),  z_start + (j_ * resolution) )   );
    //                 right_x_i = findJump(octree, lookup_table, coord_ptr, i_, resolution);
    //                 right_x_j = findJump(octree, lookup_table, coord_ptr, j_, resolution);
    //                 addIfUnique_new(neighbors, coord_ptr);
    //                 // ROS_WARN_STREAM("[" << i_ << "]  right_x_i " << right_x_i);
    //                 // ROS_WARN_STREAM("[" << j_ << "]  right_x_j " << right_x_j);
    //             }

    //             // Front 
    //             // addIfUnique(neighbors, x_start + (i_ * resolution),  front_y,                     z_start + (j_ * resolution));
    //             if(   (front_y_i < i_) || (front_y_j < j_)   ) 
    //             {
    //                 coord_ptr = std::make_shared<octomath::Vector3> (    octomath::Vector3 (x_start + (i_ * resolution), front_y, z_start + (j_ * resolution) )   );
    //                 front_y_i = findJump(octree, lookup_table, coord_ptr, i_, resolution);
    //                 front_y_j = findJump(octree, lookup_table, coord_ptr, j_, resolution);
    //                 addIfUnique_new(neighbors, coord_ptr);
    //             }
    //             // Back
    //             // addIfUnique(neighbors, x_start + (i_ * resolution),  back_y,                      z_start + (j_ * resolution));
    //             if(   (back_y_i < i_) || (back_y_j < j_)   ) 
    //             {
    //                 coord_ptr = std::make_shared<octomath::Vector3> (    octomath::Vector3 (x_start + (i_ * resolution),  back_y, z_start + (j_ * resolution) )   );
    //                 back_y_i = findJump(octree, lookup_table, coord_ptr, i_, resolution);
    //                 back_y_j = findJump(octree, lookup_table, coord_ptr, j_, resolution);
    //                 addIfUnique_new(neighbors, coord_ptr);
    //             }

    //             // Up Down
    //             // addIfUnique(neighbors, x_start + (i_ * resolution),  y_start + (j_ * resolution),  up_z);
    //             if(   (up_z_i < i_) || (up_z_j < j_)   ) 
    //             {
    //                 coord_ptr = std::make_shared<octomath::Vector3> (    octomath::Vector3 (x_start + (i_ * resolution),  y_start + (j_ * resolution),  up_z )   );
    //                 up_z_i = findJump(octree, lookup_table, coord_ptr, i_, resolution);
    //                 up_z_j = findJump(octree, lookup_table, coord_ptr, j_, resolution);
    //                 addIfUnique_new(neighbors, coord_ptr);
    //             }
    //             // addIfUnique(neighbors, x_start + (i_ * resolution),  y_start + (j_ * resolution),  down_z);
    //             if(   (down_z_i < i_) || (down_z_j < j_)   ) 
    //             {
    //                 coord_ptr = std::make_shared<octomath::Vector3> (    octomath::Vector3 (x_start + (i_ * resolution),  y_start + (j_ * resolution),  down_z )   );
    //                 down_z_i = findJump(octree, lookup_table, coord_ptr, i_, resolution);
    //                 down_z_j = findJump(octree, lookup_table, coord_ptr, j_, resolution);
    //                 addIfUnique_new(neighbors, coord_ptr);
    //             }

    //         }
    //     }
    // }

    // double calculate_fraction(double resolution, double margin, int check_only_x_fraction)
    // {
    //     double max_res_voxels_within_margin = margin / resolution;
    //     double x  = std::max (max_res_voxels_within_margin/check_only_x_fraction, 1.0);
    //     double resolution_for_neighbors_m = resolution * x;
    //     // ROS_WARN_STREAM("resolution: " << resolution);
    //     // ROS_WARN_STREAM("margin: " << margin);
    //     // ROS_WARN_STREAM("max_res_voxels_within_margin: " << max_res_voxels_within_margin);
    //     // ROS_WARN_STREAM("check_only_x_fraction: " << check_only_x_fraction);
    //     // ROS_WARN_STREAM("x: " << x);
    //     // ROS_WARN_STREAM("resolution_for_neighbors_m: " << resolution_for_neighbors_m);
    //     return resolution_for_neighbors_m;
    // }

    // void generateNeighbors_pointers_margin(std::unordered_set<std::shared_ptr<octomath::Vector3>> & neighbors, 
    //     octomath::Vector3 const& center_coords, 
    //     float node_size, float resolution,
    //     double margin_neighbor_res, // security margin neighbor count
    //     bool debug_on/* = false*/)
    // {
    //     int neighbor_count_security_margin = std::ceil(node_size/margin_neighbor_res);
    //     int n_count = 0;
    //     // mem_alloc_nanosecs = 0;
    //     // auto start = std::chrono::high_resolution_clock::now();
    //     int neighbor_sequence_cell_count = node_size / resolution;
    //     float frontier_offset = (node_size/2.f);         
    //     int i;
    //     // int neighbor_count = 0;
    //     float extra = resolution / 2.f;     

    //     float x_start  = center_coords.x() - frontier_offset + extra;
    //     float y_start  = center_coords.y() - frontier_offset + extra;     
    //     float z_start  = center_coords.z() - frontier_offset + extra;                      
    //     // Left right
    //     float right_x = center_coords.x() + frontier_offset + extra; 
    //     float left_x  = center_coords.x() - frontier_offset - extra; 
    //     // Front back
    //     float front_y  = center_coords.y() + frontier_offset + extra; 
    //     float back_y   = center_coords.y() - frontier_offset - extra; 
    //     // Up down      
    //     float up_z     = center_coords.z() + frontier_offset + extra; 
    //     float down_z   = center_coords.z() - frontier_offset - extra;

        
    //     int neighbor_sequence_cell_count_j = neighbor_sequence_cell_count;
    //     int i_in_octomap_res;
    //     int j_in_octomap_res;
    //     // optimized
    //     octomath::Vector3* toInsert;
    //     bool isInserted;
    //     for(int i = 0; i < neighbor_count_security_margin; i++)
    //     {
    //         // int j = 0;
    //         for(int j = 0; j < neighbor_count_security_margin; j++)
    //         {
    //             i_in_octomap_res = std::floor( (i * margin_neighbor_res) / resolution );
    //             i_in_octomap_res = std::min(i_in_octomap_res, neighbor_sequence_cell_count_j-1 );

    //             j_in_octomap_res = std::floor( (j * margin_neighbor_res) / resolution );
    //             j_in_octomap_res = std::min(j_in_octomap_res, neighbor_sequence_cell_count_j-1 );

    //             // Left Right
    //             addIfUnique(neighbors, left_x,                      y_start + (i_in_octomap_res * resolution),  z_start + (j_in_octomap_res * resolution));
    //             addIfUnique(neighbors, right_x,                     y_start + (i_in_octomap_res * resolution),  z_start + (j_in_octomap_res * resolution));

    //             // // Front Back
    //             addIfUnique(neighbors, x_start + (i_in_octomap_res * resolution),  front_y,                     z_start + (j_in_octomap_res * resolution));
    //             addIfUnique(neighbors, x_start + (i_in_octomap_res * resolution),  back_y,                      z_start + (j_in_octomap_res * resolution));

    //             // Up Down
    //             addIfUnique(neighbors, x_start + (i_in_octomap_res * resolution),  y_start + (j_in_octomap_res * resolution),  up_z);
    //             addIfUnique(neighbors, x_start + (i_in_octomap_res * resolution),  y_start + (j_in_octomap_res * resolution),  down_z);

    //             // neighbor_count++;
    //         }
    //     }
    // }

    double distanceCalculate(double x1, double y1, double x2, double y2)
    {
        double x = x1 - x2; //calculating number to square in next step
        double y = y1 - y2;
        double dist;

        dist = pow(x, 2) + pow(y, 2);       //calculating Euclidean distance
        dist = sqrt(dist);                  

        return dist;
    }

    bool isInsideBlindR(double n_x, double n_y, double c_x, double c_y, double blind_perimeter)
    {
        double distance = distanceCalculate(n_x, n_y, c_x, c_y);
        // ROS_WARN_STREAM("[neighbors] (" << n_x << ", " << n_y << ") - distance " << distance << " < " << blind_perimeter << " (blind_perimeter)");
        return distance < blind_perimeter;
    }

    void generateNeighbors_frontiers_pointers(std::unordered_set<std::shared_ptr<octomath::Vector3>> & neighbors, 
        octomath::Vector3 const& center_coords, 
        float node_size, float resolution, double sensor_angle_rad, bool debug_on/* = false*/)
    {
        int neighbor_sequence_cell_count = node_size / resolution;
        float frontier_offset = (node_size/2.f);         
        int i;
        float extra = resolution / 2.f;     

        float x_start  = center_coords.x() - frontier_offset + extra;
        float y_start  = center_coords.y() - frontier_offset + extra;     
        float z_start  = center_coords.z() - frontier_offset + extra;                      
        // Left right
        float right_x = center_coords.x() + frontier_offset + extra; 
        float left_x  = center_coords.x() - frontier_offset - extra; 
        // Front back
        float front_y  = center_coords.y() + frontier_offset + extra; 
        float back_y   = center_coords.y() - frontier_offset - extra; 
        // Up down      
        float up_z     = center_coords.z() + frontier_offset + extra; 
        float down_z   = center_coords.z() - frontier_offset - extra;

        
        double blind_perimeter = (frontier_offset + extra) / std::tan(sensor_angle_rad);

        // optimized
        octomath::Vector3* toInsert;
        bool isInserted;
        for(int i = 0; i < neighbor_sequence_cell_count; i++)
        {
            // int j = 0;
            for(int j = 0; j < neighbor_sequence_cell_count; j++)
            {
                // Left Right
                addIfUnique(neighbors, left_x,                      y_start + (i * resolution),  z_start + (j * resolution));
                addIfUnique(neighbors, right_x,                     y_start + (i * resolution),  z_start + (j * resolution));

                // Front Back
                addIfUnique(neighbors, x_start + (i * resolution),  front_y,                     z_start + (j * resolution));
                addIfUnique(neighbors, x_start + (i * resolution),  back_y,                      z_start + (j * resolution));

                // No neighbors in blind spot
                double n_x = x_start + (i * resolution);
                double n_y = y_start + (j * resolution);

                if(!isInsideBlindR(n_x, n_y, center_coords.x(), center_coords.y(), blind_perimeter))
                {
                    // Up Down
                    addIfUnique(neighbors, x_start + (i * resolution),  y_start + (j * resolution),  up_z);
                    addIfUnique(neighbors, x_start + (i * resolution),  y_start + (j * resolution),  down_z);
                }
            }
        }
    }

    bool addSparseNeighbor(std::unordered_set<std::shared_ptr<octomath::Vector3>> & neighbors, double x, double y, double z, octomap::OcTree const& octree)
    {
        octomath::Vector3 toAdd (x, y, z);
        auto res_node = octree.search(x, y, z);
        if(res_node)
        {
            try
            {
                toAdd = getCellCenter(toAdd, octree);
            }
            catch (const std::out_of_range& oor) {
            }
        }
        addIfUnique(neighbors, toAdd);
    }


    void generateNeighbors_filter_pointers(std::unordered_set<std::shared_ptr<octomath::Vector3>> & neighbors, 
        octomath::Vector3 const& center_coords, 
        float node_size, float resolution, octomap::OcTree const& octree, bool debug_on)
    {
        int neighbor_sequence_cell_count = node_size / resolution;
        float frontier_offset = (node_size/2.f);         
        int i;
        float extra = resolution / 2.f;     

        float x_start  = center_coords.x() - frontier_offset + extra;
        float y_start  = center_coords.y() - frontier_offset + extra;     
        float z_start  = center_coords.z() - frontier_offset + extra;                      
        // Left right
        float right_x = center_coords.x() + frontier_offset + extra; 
        float left_x  = center_coords.x() - frontier_offset - extra; 
        // Front back
        float front_y  = center_coords.y() + frontier_offset + extra; 
        float back_y   = center_coords.y() - frontier_offset - extra; 
        // Up down      
        float up_z     = center_coords.z() + frontier_offset + extra; 
        float down_z   = center_coords.z() - frontier_offset - extra;

        octomath::Vector3 toInsert;
        bool isInserted;
        for(int i = 0; i < neighbor_sequence_cell_count; i++)
        {
            // int j = 0;
            for(int j = 0; j < neighbor_sequence_cell_count; j++)
            {
                // Left Right
                addSparseNeighbor(neighbors, left_x,                      y_start + (i * resolution),  z_start + (j * resolution), octree);
                addSparseNeighbor(neighbors, right_x,                     y_start + (i * resolution),  z_start + (j * resolution), octree);

                // Front Back
                addSparseNeighbor(neighbors, x_start + (i * resolution),  front_y,                     z_start + (j * resolution), octree);
                addSparseNeighbor(neighbors, x_start + (i * resolution),  back_y,                      z_start + (j * resolution), octree);

                // Up Down
                addSparseNeighbor(neighbors, x_start + (i * resolution),  y_start + (j * resolution),  up_z, octree);
                addSparseNeighbor(neighbors, x_start + (i * resolution),  y_start + (j * resolution),  down_z, octree);
            }
        }
    }

    
    // Other way to find the depth based on the search code of the octree
    int getNodeDepth_Octomap (const octomap::OcTreeKey& key, 
    	octomap::OcTree const& octree)  
    {
        int tree_depth = octree.getTreeDepth();
        octomap::OcTreeNode* root = octree.getRoot();


        if (root == NULL)
            return -1;
        int depth = tree_depth;
        int depth_tracking = depth;

        // generate appropriate key_at_depth for queried depth
         octomap::OcTreeKey key_at_depth = key;
        if (depth != tree_depth)
            key_at_depth = octree.adjustKeyAtDepth(key, depth);
        //ROS_WARN_STREAM("Depth "<<depth_tracking);

        octomap::OcTreeNode* curNode (root);

        unsigned int diff = tree_depth - depth;

        // follow nodes down to requested level (for diff = 0 it's the last level)
        for (unsigned i=(tree_depth-1); i>=diff; --i) {
            unsigned int pos = computeChildIdx(key_at_depth, i);
            if (octree.nodeChildExists(curNode, pos)) 
            {
                // cast needed: (nodes need to ensure it's the right pointer)
                curNode = static_cast<octomap::OcTreeNode*>( octree.getNodeChild(curNode, pos) );
                depth_tracking--;
                //ROS_WARN_STREAM("Depth "<<depth_tracking);
            } 
            else 
            {
                // we expected a child but did not get it
                // is the current node a leaf already?
                if (! octree.nodeHasChildren(curNode) ) 
                {
                    //ROS_WARN_STREAM("Final depth "<<depth_tracking);
                    return 16-depth_tracking;
                } 
                else 
                {
                    // it is not, search failed
                    //ROS_WARN_STREAM("Failed to find depth ");
      //               std::ostringstream oss_filename;
      //               oss_filename << "/ros_ws/src/data/current/failed_to_find_depth__getNodeDepth_Octomap__" 
      //                   << key[0] << "_" << key[1] << "_" << key[2] << ".bt";
      //               octree.writeBinaryConst(oss_filename.str());
		            std::ostringstream oss;
                    oss << "Failed to find depth, for key " << key[0] << " " << key[1] << " " << key[2] << " stopped at " << depth << ", tree depth is " << octree.getTreeDepth() << ". @getNodeDepth_Octomap";  
                    throw std::out_of_range(oss.str());
                    return -2;
                }
            }
        } // end for
        		  //ROS_WARN_STREAM("Final depth 2 "<<depth_tracking);
        return 16-depth_tracking;
    }

	octomath::Vector3 getCellCenter(octomath::Vector3 const& point_coordinates, octomap::OcTree const& octree)
	{
		// convert to key
		octomap::OcTreeKey key = octree.coordToKey(point_coordinates);
		// find depth of cell
        // ROS_WARN_STREAM("Calling getNodeDepth from 141 for point coordinates " << point_coordinates);

        double depth = getNodeDepth_Octomap(key, octree);  
		// get center coord of cell center at depth
		return octree.keyToCoord(key, depth);
	}

    double findSideLenght(int octreeLevelCount, const int depth, double const* lookup_table)
    {
        int level_count = octreeLevelCount - depth;
        // double side_length = octree.getResolution();
        // for(int i = 0; i < level_count; i++)
        // {
        //     side_length = side_length + side_length;
        // }
        // return side_length;
        return lookup_table[level_count];
    }

    /**
     * @brief      Compares coordinates to cell center to see if it is the cell center.
     *             Finds cell size.
     *             Frees the memory that coordinates points to if those coordinates are in fact the cell center.
     *
     * @param      coordinates  The original coordinates and the storing place of the cell center.
     * @param      octree       The octree
     * @param      cell_size    Variable to store the cell size
     *
     * @return     true if the coordinates correspond to the cell center, false otherise
     */
    octomap::OcTreeKey updatePointerToCellCenterAndFindSize(std::shared_ptr<octomath::Vector3> & coordinates, octomap::OcTree const& octree, double& side_length, double const* lookup_table)
    {
        // convert to key
        octomap::OcTreeKey key = octree.coordToKey(*coordinates);
        // find depth of cell
        // ROS_WARN_STREAM("Calling getNodeDepth from 173");

        int depth = getNodeDepth_Octomap(key, octree);
        side_length = findSideLenght(octree.getTreeDepth(), depth, lookup_table);
        // get center coord of cell center at depth
        // std::cout << std::setprecision(10) << "Depth method: " << depth << std::endl;
        octomath::Vector3 cell_center = octree.keyToCoord(key, depth);
        coordinates = std::make_shared<octomath::Vector3> (octomath::Vector3 (cell_center.x(), cell_center.y(), cell_center.z()));
        return key;
    }

    /**
     * @brief      Compares coordinates to cell center to see if it is the cell center.
     *             Finds cell size.
     *
     * @param      coordinates  The original coordinates and the storing place of the cell center.
     * @param      octree       The octree
     * @param      cell_size    Variable to store the cell size
     *
     * @return     true if the coordinates correspond to the cell center, false otherise
     */
    void updateToCellCenterAndFindSize(octomath::Vector3 & coordinates, octomap::OcTree const& octree, double& side_length, double const* lookup_table)
    {
        // convert to key
        octomap::OcTreeKey key = octree.coordToKey(coordinates);
        // ROS_WARN_STREAM("Calling getNodeDepth from 199 with key " << key[0] << " " << key[1] << " " << key[2]);
        // find depth of cell
        double depth = getNodeDepth_Octomap(key, octree);
        side_length = findSideLenght(octree.getTreeDepth(), depth, lookup_table);
        // get center coord of cell center at depth
        octomath::Vector3 cell_center = octree.keyToCoord(key, depth);
        coordinates = cell_center;
    }

    void fillLookupTable(double resolution, int tree_depth, double lookup_table_ptr[])
    {
        double side_length = resolution;
        lookup_table_ptr[0] = side_length;
        for(int i = 1; i < tree_depth; i++)
        {
            side_length = side_length + side_length;
            lookup_table_ptr[i] = side_length;
        }
    }

	void findDifferentSizeCells_ptr_3D(octomap::OcTree const& octree)
	{
		std::unordered_set<std::shared_ptr<octomath::Vector3>> neighbors_us;
		float resolution = octree.getResolution();
		float node_size; 
    	octomath::Vector3 center_coords;

		std::map <float, int> cellsStatistics;
		std::map <float, float> neighborDurationAverage;
		std::map <float, octomath::Vector3> cells;
		std::map <float, int> groundTruth;
		
		ros::Time start_from_the_top = ros::Time::now();
        octomap::OcTree::leaf_iterator it = octree.begin_leafs();
        int count = 0;
        ros::Duration elapsed_time;
        for (; it != octree.end_leafs(); ++it)
        {
        	node_size = it.getSize(); 
        	center_coords = it.getCoordinate();


        	ros::Time begin = ros::Time::now();
        	generateNeighbors_pointers(neighbors_us, center_coords, node_size, resolution);
        	ros::Time finish = ros::Time::now();


        	count++;
        	if(cells.count(it.getSize()) == 0 )
        	{
        		cellsStatistics[it.getSize()] = 0;
        		groundTruth[it.getSize()] = 0;
        		neighborDurationAverage[it.getSize()] = 0;
        	}
    		cellsStatistics[node_size]++;
			groundTruth[node_size] ++;
    		cells[node_size] = it.getCoordinate();
    		neighborDurationAverage[node_size] = (neighborDurationAverage[node_size] + (finish - begin).nsec) / cellsStatistics[node_size];
    		
        }

    	elapsed_time = ros::Time::now() - start_from_the_top;
        ROS_WARN_STREAM(count << " cells were analysed in " << elapsed_time.nsec << " nano seconds.");

        for (std::map<float, octomath::Vector3>::iterator it=cells.begin(); 
        	it!=cells.end(); ++it)
        {
        	ROS_WARN_STREAM( it->first << " => " << it->second << ". " << cellsStatistics[it->first] 
        	<< " cells of this size were found. Took in average " << neighborDurationAverage[it->first] << " nano seconds.");
        }
    }

    double calculateCellSpace(octomap::OcTree const& octree)
    {
        float node_size; 
        octomap::OcTree::leaf_iterator it = octree.begin_leafs();
        double volume = 0;
        for (; it != octree.end_leafs(); ++it)
        {
            volume += pow(it.getSize(), 3);
        }
        return volume;
    }
}
