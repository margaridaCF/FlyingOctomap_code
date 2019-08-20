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

	frontiers_msgs::FindFrontiers::Response getFrontiers(octomap::OcTree const& octree, frontiers_msgs::FindFrontiers::Request  &request, ros::Publisher const& marker_pub, Frontiers::search_function search)
	{
		octomath::Vector3 min (request.min.x, request.min.y, request.min.z);
		octomath::Vector3 max (request.max.x, request.max.y, request.max.z);
		frontiers_msgs::FindFrontiers::Response reply;
		int start_it = 0;
        Frontiers::Circulator it (octree, max, min, start_it);
    	search(octree, it, request, reply, marker_pub, false);
    	return reply;
	}

	bool compareVoxelMsg(frontiers_msgs::VoxelMsg & a, frontiers_msgs::VoxelMsg & b)
	{
		bool equal = (a.xyz_m.x == b.xyz_m.x);
		equal = equal && (a.xyz_m.y == b.xyz_m.y);
		equal = equal && (a.xyz_m.z == b.xyz_m.z);
		equal = equal && (a.size == b.size);
		return equal;
	}

	TEST(CirculatorTest, SearchFrontiers)
	{
		ros::Publisher marker_pub;
		octomap::OcTree octree ("data/hilt_measurements.bt");
		frontiers_msgs::FindFrontiers::Request  request;
		request.min.x = -40;
		request.max.x = 30;
		
		request.min.y = -18;
		request.max.y = 20;

		request.min.z = 4;
		request.max.z = 35;
		request.frontier_amount = 1;
		auto start = std::chrono::high_resolution_clock::now();
		frontiers_msgs::FindFrontiers::Response reply_v1 = getFrontiers(octree, request, marker_pub, Frontiers::searchFrontier);
		auto end = std::chrono::high_resolution_clock::now(); 
		auto time_span = end - start;
		ROS_INFO_STREAM("v1 took " << std::chrono::duration_cast<std::chrono::microseconds>(time_span).count());

		start = std::chrono::high_resolution_clock::now();
		frontiers_msgs::FindFrontiers::Response reply_v2 = getFrontiers(octree, request, marker_pub, Frontiers::searchFrontier_optimized);
		end = std::chrono::high_resolution_clock::now(); 
		time_span = end - start;
		ROS_INFO_STREAM("v2 took " << std::chrono::duration_cast<std::chrono::microseconds>(time_span).count());

		ASSERT_EQ(reply_v1.frontiers.size(), reply_v2.frontiers.size());
		int index = 0;
		while(index < reply_v1.frontiers.size())
		{
			ASSERT_TRUE(   compareVoxelMsg( reply_v1.frontiers[index], reply_v2.frontiers[index] )   );
			index++;
		}
	}



}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}