#include <next_best_view.h>
#include <frontiers.h>

namespace NextBestView
{
	NextBestViewSM::NextBestViewSM(double distance_inFront, double distance_behind, int circle_divisions, double frontier_safety_margin, ros::Publisher const& marker_pub, bool publish)
		:geofence_min(geofence_min), geofence_max(geofence_max), path_safety_margin(path_safety_margin), marker_pub(marker_pub)
	{
		oppairs = observation_lib::OPPairs(circle_divisions, frontier_safety_margin, distance_inFront, distance_behind);
	}

	bool NextBestViewSM::FindNext(frontiers_msgs::FrontierRequest const& request, std::vector<observation_lib::OPPair> oppairs)
	{
		if(request.request_number != current_request) return false;
		
		frontiers_msgs::FrontierReply reply;
    	Frontiers::searchFrontier(*octree, it, request, reply, marker_pub, publish);




		return true;

	}
	
	void NextBestViewSM::NewRequest(octomap::OcTree* new_octree, frontiers_msgs::FrontierRequest const& request)
	{
		current_request = request.request_number;
		octree = new_octree;

		// reset iterator
        double resolution = octree->getResolution();
        octomath::Vector3  max = octomath::Vector3(request.max.x-resolution, request.max.y-resolution, request.max.z-resolution);
        octomath::Vector3  min = octomath::Vector3(request.min.x+resolution, request.min.y+resolution, request.min.z+resolution);
		octomap::OcTreeKey bbxMinKey, bbxMaxKey;
        if(!octree->coordToKeyChecked(min, bbxMinKey) || !octree->coordToKeyChecked(max, bbxMaxKey))
        {
            ROS_ERROR_STREAM("[Frontiers] Problems with the octree");
        }
		it = octree->begin_leafs_bbx(bbxMinKey,bbxMaxKey);
	}

}