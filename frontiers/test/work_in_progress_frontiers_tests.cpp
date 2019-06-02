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
		ROS_INFO_STREAM("is_inside: side_max > min <==> " << side_max << " > " << min);
		if(is_inside)
		{
			bool all_in = side_min >= min;
			ROS_INFO_STREAM("All in? " << side_min << " > " << min);
			if (all_in)
			{
				ROS_INFO_STREAM("Yup, all in!");
				return size;
			}
			else
			{
				how_much = std::abs(min - side_max);
				ROS_INFO_STREAM("Nope, just " << how_much << " = std::abs(" << min << " - " << side_max << ")");
				return how_much;
			}
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
		ROS_INFO_STREAM("Center: " << it.getCoordinate() << " size " << it.getSize() << ". Sides " << side_min << " to " << side_max << ". Geofence Min " << min);
		double how_much;
		bool is_inside = side_max.x() > min.x(); 
		double side_x = sizeInsideGeofence_min(side_min.x(), side_max.x(), it.getSize(), min.x());
		return side_x;
		// ROS_INFO_STREAM("is_inside: side_max > min <==> " << side_max.x() << " > " << min.x());
		// if(is_inside)
		// {
		// 	bool all_in = side_min.x() >= min.x();
		// 	ROS_INFO_STREAM("All in? " << side_min.x() << " > " << min.x());
		// 	if (all_in)
		// 	{
		// 		ROS_INFO_STREAM("Yup, all in!");
		// 		return it.getSize();
		// 	}
		// 	else
		// 	{
		// 		how_much = std::abs(min.x() - side_max.x());
		// 		ROS_INFO_STREAM("Nope, just " << how_much << " = std::abs(" << min.x() << " - " << side_max.x() << ")");
		// 		return how_much;
		// 	}
		// }
		// else
		// {
		// 	return 0;
		// }

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

	TEST(VolumeTest, InsideOnBorderGeofence)
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
		ASSERT_NEAR(volumeInsideGeofence (min, max, it), 0.2, 0.01);
	}

	TEST(VolumeTest, InsideGeofence)
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
		ASSERT_NEAR(volumeInsideGeofence (min, max, it), 0.2, 0.01);
	}
    

}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}