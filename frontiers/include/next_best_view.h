#ifndef NBV_H
#define NBV_H

#include <frontiers_common.h>

#include <observation_maneuver.h>

namespace Frontiers{

	class NextBestViewSM
	{
		ros::Publisher const& marker_pub;
		bool publish;
		int current_request;
        std::set<octomath::Vector3, Vector3Hash> empty_space_neighbours; 
		octomap::OcTree* octree;
		octomap::OcTree::leaf_bbx_iterator it;

		geometry_msgs::Point 				geofence_min, geofence_max;
    	double 								path_safety_margin;
	    observation_lib::OPPairs 			oppairs;

	public:
		NextBestViewSM(double distance_inFront, double distance_behind, int circle_divisions, double frontier_safety_margin, ros::Publisher const& marker_pub, bool publish);
		~NextBestViewSM(){}
		void ProcessOPPairs(){}
		void NewRequest(octomap::OcTree* new_octree, frontiers_msgs::FrontierRequest const& request);
		bool FindNext(frontiers_msgs::FrontierRequest const& request, frontiers_msgs::FrontierReply & reply, std::vector<observation_lib::OPPair> oppairs);
	};


}
#endif //NBV_H