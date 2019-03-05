#ifndef NBV_H
#define NBV_H

#include <vector>
#include <cmath>
#include <sstream>
#include <iostream>
#include <fstream>
#include <chrono>
#include <algorithm>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <marker_publishing_utils.h>
#include <octomap/OcTree.h>
#include <octomap/math/Vector3.h>

#include <observation_maneuver.h>
#include <frontiers_msgs/FrontierReply.h>
#include <frontiers_msgs/FrontierRequest.h>

namespace NextBestView{

    struct Vector3Hash
    {
        std::size_t operator()(const octomath::Vector3 & v) const 
        {
            int scale = 0.00001;
            std::size_t hx = std::hash<float>{}( (int)(v.x() / scale) * scale );
            std::size_t hy = std::hash<float>{}( (int)(v.y() / scale) * scale );
            std::size_t hz = std::hash<float>{}( (int)(v.z() / scale) * scale );
            std::size_t return_value = ((hx 
               ^ (hy << 1)) >> 1)
               ^ (hz << 1);
            return return_value;
        }
    };

	class NextBestViewSM
	{
		int current_request;
        std::set<octomath::Vector3, Vector3Hash> empty_space_neighbours; 
		octomap::OcTree* octree;
		octomap::OcTree::leaf_bbx_iterator it;

		geometry_msgs::Point 				geofence_min, geofence_max;
    	double 								path_safety_margin;
	    observation_lib::OPPairs 			oppairs;

	public:
		NextBestViewSM(double distance_inFront, double distance_behind, int circle_divisions, double frontier_safety_margin);
		~NextBestViewSM(){}
		void ProcessOPPairs(){}
		void NewRequest(octomap::OcTree* new_octree, int request_number, int amount, geometry_msgs::Point max, geometry_msgs::Point min);
		bool FindNext(int amount, std::vector<observation_lib::OPPair> oppairs, int request_number);
	};


}
#endif //NBV_H