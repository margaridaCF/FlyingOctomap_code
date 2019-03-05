#include <next_best_view.h>

namespace NextBestView
{
	NextBestViewSM::NextBestViewSM(double distance_inFront, double distance_behind, int circle_divisions, double frontier_safety_margin)
		:geofence_min(geofence_min), geofence_max(geofence_max), path_safety_margin(path_safety_margin)
	{
		oppairs = observation_lib::OPPairs(circle_divisions, frontier_safety_margin, distance_inFront, distance_behind);
	}

	bool NextBestViewSM::FindNext(int amount, std::vector<observation_lib::OPPair> oppairs, int request_number)
	{
		if(request_number != current_request) return false;
		else return true;

	}
	
	void NextBestViewSM::NewRequest(octomap::OcTree* new_octree, int request_number, int amount, geometry_msgs::Point max, geometry_msgs::Point min)
	{
		current_request = request_number;
		octree = new_octree;

		// reset iterator
        double resolution = octree->getResolution();
        octomath::Vector3  max_v = octomath::Vector3(max.x-resolution, max.y-resolution, max.z-resolution);
        octomath::Vector3  min_v = octomath::Vector3(min.x+resolution, min.y+resolution, min.z+resolution);
		octomap::OcTreeKey bbxMinKey, bbxMaxKey;
        if(!octree->coordToKeyChecked(min_v, bbxMinKey) || !octree->coordToKeyChecked(max_v, bbxMaxKey))
        {
            ROS_ERROR_STREAM("[Frontiers] Problems with the octree");
        }
		it = octree->begin_leafs_bbx(bbxMinKey,bbxMaxKey);
	}

}