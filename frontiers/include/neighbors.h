#ifndef NEIGHBORS_H
#define NEIGHBORS_H


#include <chrono>
#include <unordered_set>
#include <octomap/math/Vector3.h>
#include <octomap/OcTree.h>
#include <memory>
#include <ros/ros.h>

namespace LazyThetaStarOctree{
	bool addIfUnique(std::unordered_set<std::shared_ptr<octomath::Vector3>> & neighbors, float x, float y, float z );

	// TODO reduce neighbor number by finding cell center and removing duplicates
	void generateNeighbors_pointers(std::unordered_set<std::shared_ptr<octomath::Vector3>> & neighbors, 
		octomath::Vector3 const& center_coords, 
		float node_size, float resolution, bool is3d = true, bool debug_on = false);


    // Other way to find the depth based on the search code of the octree
    int getNodeDepth_Octomap (const octomap::OcTreeKey& key, 
    	octomap::OcTree const& octree)  ;

	octomath::Vector3 getCellCenter(octomath::Vector3 const& point_coordinates, octomap::OcTree const& octree);

    double findSideLenght(octomap::OcTree const& octree, const int depth);

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
    octomap::OcTreeKey updatePointerToCellCenterAndFindSize(std::shared_ptr<octomath::Vector3> & coordinates, octomap::OcTree const& octree, double& side_length);

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
    void updateToCellCenterAndFindSize(octomath::Vector3 & coordinates, octomap::OcTree const& octree, double& side_length);


	void findDifferentSizeCells_ptr_3D(octomap::OcTree const& octree);

    double calculateCellSpace(octomap::OcTree const& octree);
}
#endif // NEIGHBORS_H