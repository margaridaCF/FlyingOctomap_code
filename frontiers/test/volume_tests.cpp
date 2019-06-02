#include <gtest/gtest.h>
#include <volume.h>

namespace volume
{
	TEST(VolumeTest, Outside)
	{
		octomap::OcTree octree ("data/experimentalDataset.bt");
		octomath::Vector3 min (0, 0, 0);
		octomath::Vector3 max (1, 1, 1);

		octomap::OcTreeKey bbxMinKey, bbxMaxKey;
        if(!octree.coordToKeyChecked(min, bbxMinKey) || !octree.coordToKeyChecked(max, bbxMaxKey))
        {
            ROS_ERROR_STREAM("[Frontiers] Problems with write_volume_explored_to_csv");
        }
		octomap::OcTree::leaf_bbx_iterator it = octree.begin_leafs_bbx(bbxMinKey,bbxMaxKey);
		bool search = true;
		double volume, size;
		while(it != octree.end_leafs_bbx() && search)
		{
			volume = volumeInsideGeofence (min, max, it);
			search = volume != 0;
			size = it.getSize();
			it++;
		}
		ASSERT_NEAR(volume, 0, 0.0001);
	}

	TEST(VolumeTest, volumeInsideGeofence_halfMax)
	{
		octomap::OcTree octree ("data/experimentalDataset.bt");
		octomath::Vector3 min (0, 0, 0);
		octomath::Vector3 max (1, 1, 1);

		octomap::OcTreeKey bbxMinKey, bbxMaxKey;
        if(!octree.coordToKeyChecked(min, bbxMinKey) || !octree.coordToKeyChecked(max, bbxMaxKey))
        {
            ROS_ERROR_STREAM("[Frontiers] Problems with write_volume_explored_to_csv");
        }
		octomap::OcTree::leaf_bbx_iterator it = octree.begin_leafs_bbx(bbxMinKey,bbxMaxKey);
		bool search = true;
		double volume, size;
		while(it != octree.end_leafs_bbx() && search)
		{
			volume = volumeInsideGeofence (min, max, it);
			search = (   volume == ( it.getSize()*it.getSize()*it.getSize() )   ) || (volume == 0);
			size = it.getSize();
			it++;
		}
		ASSERT_NEAR(volume, 0.032, 0.0001);
		ASSERT_NEAR(size, 0.4, 0.01);
	}

	TEST(VolumeTest, sizeInsideGeofenceMax_half)
	{
		
		double side_x = sizeInsideGeofence_max(0.9, 1.1, 0.2, 1);
		ASSERT_NEAR(side_x, 0.1, 0.01);
	}
	TEST(VolumeTest, sizeInsideGeofenceMax_allIn)
	{
		
		double side_x = sizeInsideGeofence_max(0.9, 1.1, 0.2, 1.1);
		ASSERT_NEAR(side_x, 0.2, 0.01);
	}
	TEST(VolumeTest, sizeInsideGeofenceMax_allOut)
	{
		
		double side_x = sizeInsideGeofence_max(0.9, 1.1, 0.2, 0.9);
		ASSERT_NEAR(side_x, 0, 0.01);
	}

	TEST(VolumeTest, sizeInsideGeofenceMax_Octree_allin)
	{
		octomap::OcTree octree ("data/experimentalDataset.bt");
		octomath::Vector3 min (0, 0, 0);
		octomath::Vector3 max (1, 1, 1);

		octomap::OcTreeKey bbxMinKey, bbxMaxKey;
        if(!octree.coordToKeyChecked(min, bbxMinKey) || !octree.coordToKeyChecked(max, bbxMaxKey))
        {
            ROS_ERROR_STREAM("[Frontiers] Problems with write_volume_explored_to_csv");
        }
		octomap::OcTree::leaf_bbx_iterator it = octree.begin_leafs_bbx(bbxMinKey,bbxMaxKey);

		octomath::Vector3 side_max = it.getCoordinate() + octomath::Vector3(it.getSize()/2, it.getSize()/2, it.getSize()/2); 
		octomath::Vector3 side_min = it.getCoordinate() - octomath::Vector3(it.getSize()/2, it.getSize()/2, it.getSize()/2); 
		// ROS_INFO_STREAM("Center: " << it.getCoordinate() << " size " << it.getSize() << ". Sides " << side_min << " to " << side_max << ". Geofence " << min << " to " << max);
		double how_much;
		bool is_inside = side_max.x() > min.x(); 
		double side_x = sizeInsideGeofence_max(side_min.x(), side_max.x(), it.getSize(), max.x());
		ASSERT_EQ(side_x, 0.2);
	}


	TEST(VolumeTest, OutsideOnBorderGeofence)
	{
		octomap::OcTree octree ("data/experimentalDataset.bt");
		octomath::Vector3 min (0, 0, 0);
		octomath::Vector3 max (1, 1, 1);

		octomap::OcTreeKey bbxMinKey, bbxMaxKey;
        if(!octree.coordToKeyChecked(min, bbxMinKey) || !octree.coordToKeyChecked(max, bbxMaxKey))
        {
            ROS_ERROR_STREAM("[Frontiers] Problems with write_volume_explored_to_csv");
        }
		octomap::OcTree::leaf_bbx_iterator it = octree.begin_leafs_bbx(bbxMinKey,bbxMaxKey);
		ASSERT_EQ(volumeInsideGeofence (min, max, it), 0);
	}

	TEST(VolumeTest, OutsideGeofenceY)
	{
		octomap::OcTree octree ("data/experimentalDataset.bt");
		octomath::Vector3 min (0, 0, 0);
		octomath::Vector3 max (1, 1, 1);

		octomap::OcTreeKey bbxMinKey, bbxMaxKey;
        if(!octree.coordToKeyChecked(min, bbxMinKey) || !octree.coordToKeyChecked(max, bbxMaxKey))
        {
            ROS_ERROR_STREAM("[Frontiers] Problems with write_volume_explored_to_csv");
        }
		octomap::OcTree::leaf_bbx_iterator it = octree.begin_leafs_bbx(bbxMinKey,bbxMaxKey);
		it++;
		ASSERT_NEAR(volumeInsideGeofence (min, max, it), 0, 0.01);
	}

	TEST(VolumeTest, OutsideYGeofence_xAllIn)
	{
		octomap::OcTree octree ("data/experimentalDataset.bt");
		octomath::Vector3 min (0, 0, 0);
		octomath::Vector3 max (1, 1, 1);

		octomap::OcTreeKey bbxMinKey, bbxMaxKey;
        if(!octree.coordToKeyChecked(min, bbxMinKey) || !octree.coordToKeyChecked(max, bbxMaxKey))
        {
            ROS_ERROR_STREAM("[Frontiers] Problems with write_volume_explored_to_csv");
        }
		octomap::OcTree::leaf_bbx_iterator it = octree.begin_leafs_bbx(bbxMinKey,bbxMaxKey);
		it++;
		it++;
		ASSERT_NEAR(volumeInsideGeofence (min, max, it), 0, 0.01);
	}
    

}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}