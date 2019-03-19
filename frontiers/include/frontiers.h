#ifndef FRONTIERS_H
#define FRONTIERS_H

#include <frontiers_common.h>

namespace Frontiers{

    bool isCenterGoodGoal(double voxel_side, double octree_resolution, double sensing_distance);
    octomap::OcTree::leaf_bbx_iterator  processFrontiersRequest(octomap::OcTree const& octree, frontiers_msgs::FrontierRequest const& request, frontiers_msgs::FrontierReply& reply, ros::Publisher const& marker_pub, bool publish = true);
    bool meetsOperationalRequirements(octomath::Vector3 const&  candidate, double min_distance, octomath::Vector3 const& current_position, octomap::OcTree const& octree, double safety_distance, geometry_msgs::Point geofence_min, geometry_msgs::Point geofence_max, ros::Publisher const& marker_pub, bool publish);
    bool isOccupied(octomath::Vector3 const& grid_coordinates_toTest, octomap::OcTree const& octree);
    bool isExplored(octomath::Vector3 const& grid_coordinates_toTest, octomap::OcTree const& octree);
    bool isFrontier(octomap::OcTree& octree, octomath::Vector3 const&  candidate, double sensor_angle); 
    void searchFrontier(octomap::OcTree const& octree, octomap::OcTree::leaf_bbx_iterator & it, frontiers_msgs::FrontierRequest const& request, frontiers_msgs::FrontierReply & reply, ros::Publisher const& marker_pub, bool publish);
    
}
#endif // FRONTIERS_H
