#ifndef DATA_COLLECTION_H
#define DATA_COLLECTION_H

#include <frontiers.h>
#include <gtest/gtest.h>

namespace collect_data
{
	void checkFrontiers(octomap::OcTree& octree, frontiers_msgs::FindFrontiers::Request  &request,
        frontiers_msgs::FindFrontiers::Response &reply);
	bool isUnknown(octomap::OcTree& octree, octomath::Vector3 candidate);

}


#endif // DATA_COLLECTION_H