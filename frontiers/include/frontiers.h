#ifndef FRONTIERS_H
#define FRONTIERS_H

#include <frontiers_common.h>

namespace Frontiers{

    octomap::OcTree::leaf_bbx_iterator  processFrontiersRequest(octomap::OcTree const& octree, frontiers_msgs::FindFrontiers::Request  &request,
        frontiers_msgs::FindFrontiers::Response &reply, ros::Publisher const& marker_pub, bool publish = true);
    bool isOccupied(octomath::Vector3 const& grid_coordinates_toTest, octomap::OcTree const& octree);
    bool isExplored(octomath::Vector3 const& grid_coordinates_toTest, octomap::OcTree const& octree);
    bool isFrontier(octomap::OcTree& octree, octomath::Vector3 const&  candidate); 
    void searchFrontier(octomap::OcTree const& octree, octomap::OcTree::leaf_bbx_iterator & it, frontiers_msgs::FindFrontiers::Request  &request,
        frontiers_msgs::FindFrontiers::Response &reply, ros::Publisher const& marker_pub, bool publish);
    
}
#endif // FRONTIERS_H
