#include <gtest/gtest.h>
#include <frontiers.h>
#include <neighbors.h>

namespace Frontiers_test
{
	void printForMatlab(std::unordered_set<std::shared_ptr<octomath::Vector3>> neighbors)
	{
		std::cout << " x_values = [ ];\n";
		std::cout << " y_values = [ ];\n";
		std::cout << " z_values = [ ];\n";
		for (std::shared_ptr<octomath::Vector3> n : neighbors)
		{
			std::cout << " x_values = [x_values, " << n->x() << "];\n ";
			std::cout << " y_values = [y_values, " << n->y() << "];\n ";
			std::cout << " z_values = [z_values, " << n->z() << "];\n ";
		}
	}

	double sizeInsideGeofence_min(double side_min, double side_max, double size, double min)
	{
		double how_much;
		bool is_inside = side_max > min; 
		// ROS_INFO_STREAM("is_inside: side_max > min <==> " << side_max << " > " << min);
		if(is_inside)
		{
			bool all_in = side_min >= min;
			// ROS_INFO_STREAM("All in? " << side_min << " > " << min);
			if (all_in)
			{
				// ROS_INFO_STREAM("[Min] Yup, all in!");
				return size;
			}
			else
			{
				how_much = std::abs(min - side_max);
				// ROS_INFO_STREAM("[Min] Nope, just " << how_much << " = std::abs(" << min << " - " << side_max << ")");
				return how_much;
			}
		}
		else
		{
			// ROS_INFO_STREAM("[Min] Completely out!");
			return 0;
		}
	}

	double sizeInsideGeofence_max(double side_min, double side_max, double size, double max)
	{
		double how_much;
		bool is_inside = side_min < max; 
		// ROS_INFO_STREAM("is_inside: side_min < max <==> " << side_min << " < " << max);
		if(is_inside)
		{
			bool all_in = side_max <= max;
			// ROS_INFO_STREAM("All in? " << side_max << " <= " << max);
			if (all_in)
			{
				// ROS_INFO_STREAM("[Max] Yup, all in!");
				return size;
			}
			else
			{
				how_much = std::abs(max - side_min);
				// ROS_INFO_STREAM("[Max] Nope, just " << how_much << " = std::abs(" << max << " - " << side_min<< ")");
				return how_much;
			}
		}
		else
		{
			// ROS_INFO_STREAM("[Max] Completely out!");
			return 0;
		}
	}

	double oneSide(double side_min, double side_max, double size, double min, double max)
	{
		double inside_min = sizeInsideGeofence_min(side_min, side_max, size, min);
		double inside_max = sizeInsideGeofence_max(side_min, side_max, size, max);
		bool inside = inside_min > 0 && inside_max > 0 ;
		if(inside)
		{
			double side = std::min(inside_min, inside_max);
			return side;
		}
		else
		{
			return 0;
		}
	}

	double volumeInsideGeofence(octomath::Vector3 const& min, octomath::Vector3 const& max, octomap::OcTree::leaf_bbx_iterator const& it)
	{
		octomath::Vector3 side_max = it.getCoordinate() + octomath::Vector3(it.getSize()/2, it.getSize()/2, it.getSize()/2); 
		octomath::Vector3 side_min = it.getCoordinate() - octomath::Vector3(it.getSize()/2, it.getSize()/2, it.getSize()/2); 
		ROS_INFO_STREAM("Center: " << it.getCoordinate() << " size " << it.getSize() << ". Sides " << side_min << " to " << side_max << ". Geofence " << min << " to " << max);

		double side_x = oneSide(side_min.x(), side_max.x(), it.getSize(), min.x(), max.x());
		if(side_x != 0)
		{
			double side_y = oneSide(side_min.y(), side_max.y(), it.getSize(), min.y(), max.y());
			if(side_y != 0)
			{
				double side_z = oneSide(side_min.z(), side_max.z(), it.getSize(), min.z(), max.z());
				double volume = side_x * side_y * side_z;
				ROS_INFO_STREAM("Volume: " << volume);
				return volume;
			}
		}
		return 0;
		
	}

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