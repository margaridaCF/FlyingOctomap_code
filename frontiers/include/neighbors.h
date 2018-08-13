#ifndef NEIGHBORS_H
#define NEIGHBORS_H


#include <chrono>
#include <unordered_set>
#include <octomap/math/Vector3.h>
#include <octomap/OcTree.h>
#include <memory>
#include <ros/ros.h>
#include <cmath>

namespace LazyThetaStarOctree{
	bool addIfUnique(std::unordered_set<std::shared_ptr<octomath::Vector3>> & neighbors, float x, float y, float z );

	// TODO reduce neighbor number by finding cell center and removing duplicates
	void generateNeighbors_pointers(std::unordered_set<std::shared_ptr<octomath::Vector3>> & neighbors, 
		octomath::Vector3 const& center_coords, 
		float node_size, float resolution, bool debug_on = false);
    void generateNeighbors_frontiers_pointers(std::unordered_set<std::shared_ptr<octomath::Vector3>> & neighbors, 
        octomath::Vector3 const& center_coords, 
        float node_size, float resolution, double sensor_angle_rad, bool debug_on = false);
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
    bool isInsideBlindR(double n_x, double n_y, double c_x, double c_y, double blind_perimeter);
}
#endif // NEIGHBORS_H