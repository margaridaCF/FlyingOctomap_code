#ifndef VOLUME_h
#define VOLUME_h

#include <octomap/math/Vector3.h>
#include <octomap/OcTree.h>
#include <ros/ros.h>
#include <frontiers.h>

namespace volume
{
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
		// ROS_INFO_STREAM("Center: " << it.getCoordinate() << " size " << it.getSize() << ". Sides " << side_min << " to " << side_max << ". Geofence " << min << " to " << max);

		double side_x = oneSide(side_min.x(), side_max.x(), it.getSize(), min.x(), max.x());
		if(side_x != 0)
		{
			double side_y = oneSide(side_min.y(), side_max.y(), it.getSize(), min.y(), max.y());
			if(side_y != 0)
			{
				double side_z = oneSide(side_min.z(), side_max.z(), it.getSize(), min.z(), max.z());
				double volume = side_x * side_y * side_z;
				// ROS_INFO_STREAM("Volume: " << volume);
				return volume;
			}
		}
		return 0;
		
	} 

	double calculateEntropy(double probability_occupied)
	{
		// double p_occupied = it->getOccupancy();
		// ROS_INFO_STREAM("p_occupied " << p_occupied);
		// ROS_INFO_STREAM("p_occupied*std::log2(p_occupied)");
		// ROS_INFO_STREAM("p_occupied*" << std::log2(p_occupied));
		// ROS_INFO_STREAM(p_occupied*std::log2(p_occupied));
		// ROS_INFO_STREAM((1-p_occupied)*std::log2(1-p_occupied));
		return -(probability_occupied*std::log2(probability_occupied)+(1-probability_occupied)*std::log2(1-probability_occupied));
	}

	std::pair<double, double> calculateVolume(octomap::OcTree const& octree, octomath::Vector3 const& min, octomath::Vector3 const& max, double & total_entropy)
	{
		double min_volume = octree.getResolution() * octree.getResolution() *octree.getResolution();
		octomap::OcTreeKey bbxMinKey, bbxMaxKey;
        if(!octree.coordToKeyChecked(min, bbxMinKey) || !octree.coordToKeyChecked(max, bbxMaxKey))
        {
            ROS_ERROR_STREAM("[Frontiers] Problems with write_volume_explored_to_csv");
        }
		octomap::OcTree::leaf_bbx_iterator it = octree.begin_leafs_bbx(bbxMinKey,bbxMaxKey);
		double free = 0;
		double occupied = 0;
		double volume_d;
		total_entropy = 0;
		while(it != octree.end_leafs_bbx())
		{
			volume_d = volumeInsideGeofence (min, max, it);
			if(Frontiers::isOccupied(it.getCoordinate(), octree))
			{
				occupied += volume_d;
			}
			else
			{
				free += volume_d;
			}
			total_entropy += (volume_d/min_volume)*calculateEntropy(it->getOccupancy());
			it++;
		}

		double total_volume = (max.x() - min.x()) * (max.y() - min.y()) *(max.z() - min.z());
		double unknown_volume = total_volume - (free + occupied); 
		total_entropy += (unknown_volume/min_volume)*calculateEntropy(0.5);
		std::pair<double, double> volume_pair = std::make_pair (free, occupied);
		return volume_pair;
	}
}

#endif // VOLUME_h
