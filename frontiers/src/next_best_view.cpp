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
	
	void NextBestViewSM::NewRequest(octomap::OcTree* new_octree, int request_number, int amount)
	{
		current_request = request_number;
		octree = new_octree;

		// reset iterator
	}
}