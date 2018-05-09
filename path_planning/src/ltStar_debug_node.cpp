#include <ros/ros.h>
#include <ltStar_temp.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/Marker.h>
#include <marker_publishing_utils.h>
#include <std_srvs/Empty.h>

namespace LazyThetaStarOctree
{
    octomap::OcTree* octree;
	ros::Publisher marker_pub;

	void ltstar_callback(const path_planning_msgs::LTStarRequest & path_request)
	{
		rviz_interface::publish_deleteAll(marker_pub);
		path_planning_msgs::LTStarReply reply;
		reply.waypoint_amount = 0;
		reply.success = false;

		std::stringstream ss;
		std::string path = "/ros_ws/src/data/";
		ss << path << "(" << path_request->start.x << "; " << path_request->start.y << "; " << path_request->start.z << ")_(" 
			<<  path_request->goal.x << "; " << path_request->goal.y << "; " << path_request->goal.z << ").bt";
		octree->writeBinary(ss.str());
		ROS_WARN_STREAM("[LTStar] Request message " << *path_request);
		LazyThetaStarOctree::processLTStarRequest(*octree, *path_request, reply, marker_pub, pauseGazebo, unpauseGazebo, true);
		if(reply.waypoint_amount == 1)
		{
			ROS_ERROR_STREAM("[LTStar] The resulting path has only one waypoint. It should always have at least start and goal. Here is the request message (the octree was saved to /data) " << *path_request);
			ROS_ERROR_STREAM("[LTStar] And here is the reply " << reply);
		}

		visualization_msgs::MarkerArray waypoint_array;
		visualization_msgs::MarkerArray arrow_array;
		visualization_msgs::Marker marker_temp;
		// Publish to rviz
		for (int i = 0; i < reply.waypoint_amount; ++i)
		{

			octomath::Vector3 candidate (reply.waypoints[i].x, reply.waypoints[i].y, reply.waypoints[i].z);
			std::unordered_set<std::shared_ptr<octomath::Vector3>> neighbors;
	        octomap::OcTreeKey key = octree->coordToKey(candidate);
	        double depth = getNodeDepth_Octomap(key, *octree);
	        double side_length = findSideLenght(*octree, depth);
	        rviz_interface::build_waypoint(candidate, side_length, (0.3*i)/reply.waypoint_amount, i, marker_temp);
	        waypoint_array.markers.push_back( marker_temp );
	        if(i !=0)
	        {

				visualization_msgs::Marker marker_temp;
				octomath::Vector3 prev_candidate (reply.waypoints[i-1].x, reply.waypoints[i-1].y, reply.waypoints[i-1].z);
				rviz_interface::build_arrow_path(candidate, prev_candidate, i, marker_temp);
	        	// ROS_WARN_STREAM("[LTStar] " << i << " Publish arrow from " << candidate << " to " << prev_candidate << marker_temp);
				arrow_array.markers.push_back( marker_temp );
	        }
		}
		marker_pub.publish(arrow_array);
		marker_pub.publish(waypoint_array);
	}
	
}

int main(int argc, char **argv)
{
	LazyThetaStarOctree::publish_free_corridor_arrows = true;
	ros::init(argc, argv, "ltstar_async_node");
	ros::NodeHandle nh;
	LazyThetaStarOctree::marker_pub = nh.advertise<visualization_msgs::MarkerArray>("ltstar_path", 1);
	octomap::OcTree octree ("/ros_ws/src/path_planning/test/data/d.bt");
	path_planning_msgs::LTStarRequest request;
	request.header.seq = 2;
	request.request_id = 3;
	request.start.x = 1.38375;
	request.start.y = -0.677482;
	request.start.z = 2.88732;
	request.goal.x = 10.5;
	request.goal.y = -5.5;
	request.goal.z = 2.5;
	request.max_search_iterations = 5000;
	request.safety_margin = 1;
	LazyThetaStarOctree::ltstar_callback(request);
  	ros::spin();
}
