#ifndef NEIGHBORS_H
#define NEIGHBORS_H


#include <chrono>
#include <unordered_set>
#include <octomap/math/Vector3.h>
#include <octomap/OcTree.h>
#include <memory>
#include <ros/ros.h>
#include <cmath>
#include <frontiers_msgs/VoxelMsg.h>

namespace LazyThetaStarOctree{

    struct Vector3PointerHash
    {
        std::size_t operator()(const std::shared_ptr<octomath::Vector3> & v) const 
        {
            double scale = 0.0001;
            std::size_t hx = std::hash<float>{}( (int)std::round(v->x() / scale) * scale );
            std::size_t hy = std::hash<float>{}( (int)std::round(v->y() / scale) * scale );
            std::size_t hz = std::hash<float>{}( (int)std::round(v->z() / scale) * scale );

            // ROS_WARN_STREAM(*v << " => " << (int)(v->x() / scale) * scale << "  " << (int)(v->y() / scale) * scale << "  " << (int)(v->z() / scale) * scale);
            // ROS_WARN_STREAM(*v << " => " << hx << "  " << hy << "  " << hz);


            std::size_t return_value = ((hx 
               ^ (hy << 1)) >> 1)
               ^ (hz << 1);
            // std::cout << return_value << std::endl;
            return return_value;

        }
    };

    struct VectorPointerComparatorEqual // for unordered_map
    { 
        bool operator () (const std::shared_ptr<octomath::Vector3> & lhs, const std::shared_ptr<octomath::Vector3> & rhs) const 
        { 
            double scale = 0.0001;
            // ROS_WARN_STREAM("Distance from " << *lhs << " and  " << *rhs << " is " << lhs->distance(*rhs) << " <= " << scale << " returning " << (lhs->distance(*rhs) <= scale)   );
            return lhs->distance(*rhs) <= scale;
            // returns !0 if the two container object keys passed as arguments are to be considered equal.
        } 
    };

    typedef std::unordered_set<std::shared_ptr<octomath::Vector3>, Vector3PointerHash, VectorPointerComparatorEqual> unordered_set_pointers;

	bool addIfUnique(unordered_set_pointers & neighbors, float x, float y, float z );
    bool addIfUnique(unordered_set_pointers & neighbors, octomath::Vector3 & toInsert );
    bool addIfUniqueValue(unordered_set_pointers & neighbors, octomath::Vector3 & toInsert );
	// TODO reduce neighbor number by finding cell center and removing duplicates
	void generateNeighbors_pointers(unordered_set_pointers & neighbors, 
		octomath::Vector3 const& center_coords, 
		float node_size, float resolution, bool debug_on = false);
    void generateNeighbors_frontiers_pointers(unordered_set_pointers & neighbors, 
        octomath::Vector3 const& center_coords, 
        float node_size, float resolution, bool debug_on = false);
    void generateNeighbors_filter_pointers(unordered_set_pointers & neighbors, 
        octomath::Vector3 const& center_coords, 
        float node_size, float resolution, octomap::OcTree const& octree, bool debug_on = false);
    bool addSparseNeighbor(unordered_set_pointers & neighbors, double x, double y, double z, octomap::OcTree const& octree);
    // void generateNeighbors_pointers_sparse(octomap::OcTree const& octree, double const* lookup_table, std::unordered_set<std::shared_ptr<octomath::Vector3>> & neighbors, 
        // octomath::Vector3 const& center_coords, 
        // float node_size, float resolution, bool debug_on = false);
    // void generateNeighbors_pointers_margin(std::unordered_set<std::shared_ptr<octomath::Vector3>> & neighbors, 
    //     octomath::Vector3 const& center_coords, 
    //     float node_size, float resolution, 
    //     double margin_neighbor_res, // security margin neighbor count
    //     bool debug_on = false);

    // Other way to find the depth based on the search code of the octree
    int getNodeDepth_Octomap (const octomap::OcTreeKey& key, 
    	octomap::OcTree const& octree)  ;

	octomath::Vector3 getCellCenter(octomath::Vector3 const& point_coordinates, octomap::OcTree const& octree);

    double findSideLenght(int octreeLevelCount, const int depth, double const* lookup_table);

    // double calculate_fraction(double resolution, double margin, int check_only_x_fraction);

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
    octomap::OcTreeKey updatePointerToCellCenterAndFindSize(std::shared_ptr<octomath::Vector3> & coordinates, octomap::OcTree const& octree, double& side_length, const double lookup_table []);

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
    void updateToCellCenterAndFindSize(octomath::Vector3 & coordinates, octomap::OcTree const& octree, double& side_length, const double lookup_table []);

    void fillLookupTable(double resolution, int tree_depth, double lookup_table_ptr[]);
	void findDifferentSizeCells_ptr_3D(octomap::OcTree const& octree);

    double calculateCellSpace(octomap::OcTree const& octree);
}
#endif // NEIGHBORS_H