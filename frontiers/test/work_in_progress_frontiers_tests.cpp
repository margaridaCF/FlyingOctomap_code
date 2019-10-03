#include <gtest/gtest.h>
#include <frontiers.h>
#include <volume.h>
#include <ordered_neighbors.h>

namespace Frontiers_test
{
    // TEST(VolumeTest, Outside)
    // {
    //     octomap::OcTree octree ("data/experimentalDataset.bt");
    //     octomath::Vector3 geofence_min (0, 0, 0);
    //     octomath::Vector3 geofence_max (1, 1, 1);


    //     // double total_entropy = 0;
    //     // std::pair<double, double> explored_volume_meters = volume::calculateVolume(octree, geofence_min, geofence_max, total_entropy);
    //     // ROS_INFO_STREAM(total_entropy);

    //     octomap::OcTreeKey bbxMinKey, bbxMaxKey;
    //     if(!octree.coordToKeyChecked(geofence_min, bbxMinKey) || !octree.coordToKeyChecked(geofence_max, bbxMaxKey))
    //     {
    //         ROS_ERROR_STREAM("[Frontiers] Problems with write_volume_explored_to_csv");
    //     }
    //     octomap::OcTree::leaf_bbx_iterator it = octree.begin_leafs_bbx(bbxMinKey,bbxMaxKey);
    //     bool search = true;
    //     double volume, size, total_entropy;
    //     total_entropy = 0;
    //     while(it != octree.end_leafs_bbx() && search)
    //     {
    //         total_entropy += volume::calculateEntropy(it);
    //         it++;
    //         // search = false;
    //     }
    //     ROS_INFO_STREAM(total_entropy);
    // }

    TEST(OrderedNeighblursTest, Heuristic_2D)
    {
        frontiers_msgs::VoxelMsg voxel_msg;
        voxel_msg.occupied_neighborhood=0;
        voxel_msg.size = 1;
        voxel_msg.xyz_m.x = 0;
        voxel_msg.xyz_m.y = 0;
        voxel_msg.xyz_m.z = 0;
        Frontiers::OrderedNeighbors list (voxel_msg);
        double distance = list.piecewiseFunc_2d(1);
        ASSERT_EQ(0, distance);
         distance = list.piecewiseFunc_2d(2.5);
        ASSERT_EQ(0, distance);
         distance = list.piecewiseFunc_2d(4);
        ASSERT_EQ(17, distance);
         distance = list.piecewiseFunc_2d(5);
        ASSERT_EQ(17, distance);
         distance = list.piecewiseFunc_2d(6);
        ASSERT_NEAR(14.32394487827058, distance, 0.1);
    }
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}