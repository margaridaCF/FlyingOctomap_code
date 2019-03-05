#include <next_best_view.h>

namespace NextBestView
{


	bool NextBestViewSM::FindNextOPPairs(int amount, std::vector<observation_lib::OPPair> oppairs, int request_number)
	{
		if(request_number != current_request) return false;
		else return true;
	}
	
	void NextBestViewSM::NewRequest(octomap::OcTree* new_octree, int request_number, int amount)
	{
		current_request = request_number;
		octree = new_octree;
	}
}