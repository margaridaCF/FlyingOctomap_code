#include <ros/ros.h>
#include <ltStar_temp.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/Marker.h>
#include <marker_publishing_utils.h>
#include <std_srvs/Empty.h>

#include <tf2/LinearMath/Transform.h>


namespace LazyThetaStarOctree
{
    octomap::OcTree* octree;
	double sidelength_lookup_table  [16];
	ros::Publisher ltstar_reply_pub;
	ros::Publisher marker_pub;

	bool octomap_init;
	bool publish_free_corridor_arrows;

	void learnTf()
	{
    	tf2::Vector3 origin (0, 0, 0);
    	tf2::Vector3 yAxis(0, 1, 0);
    	tf2::Quaternion aroundY (yAxis, M_PI/2);
    	tf2::Quaternion no_rotation (0, 0, 0, 1);
		tf2::Transform rotation;
		rotation.setOrigin(origin);
		rotation.setRotation(aroundY);

		tf2::Transform translation_to_center;
		translation_to_center.setOrigin(tf2::Vector3(-0.5, 0, 0));
		translation_to_center.setRotation(no_rotation);


		tf2::Transform translation_from_center;
		translation_from_center.setOrigin(tf2::Vector3(0.5, 0, 0));
		translation_from_center.setRotation(no_rotation);

		tf2::Transform final_transform =  rotation  * translation_to_center;

      	tf2::Vector3 toTest_start (0, 0, 0);
		tf2::Vector3 toTest_end (1, 0, 0);
      	
		rviz_interface::publish_arrow_path_unreachable(
			octomath::Vector3 (toTest_start.getX(), toTest_start.getY(), toTest_start.getZ()), 
			octomath::Vector3 (toTest_end.getX(), toTest_end.getY(), toTest_end.getZ()), 
			marker_pub, 10);	

		toTest_start = final_transform * toTest_start;
		toTest_end = final_transform * toTest_end;


		toTest_start = translation_from_center * toTest_start;
		toTest_end = translation_from_center * toTest_end;

		rviz_interface::publish_arrow_path_unreachable(
			octomath::Vector3 (toTest_start.getX(), toTest_start.getY(), toTest_start.getZ()), 
			octomath::Vector3 (toTest_end.getX(), toTest_end.getY(), toTest_end.getZ()), 
			marker_pub, 2);	
	}

	
	void octomap_callback(const octomap_msgs::Octomap::ConstPtr& octomapBinary){
		delete octree;
		ROS_WARN_STREAM("Got the octomap");
		octree = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*octomapBinary);
	    LazyThetaStarOctree::fillLookupTable(octree->getResolution(), octree->getTreeDepth(), sidelength_lookup_table); 
		octomap_init = true;
	}

	void ltstar_benchmark_callback_debug(const path_planning_msgs::LTStarBenchmarkRequest::ConstPtr& path_request)
	{
		rviz_interface::publish_deleteAll(marker_pub);
		path_planning_msgs::LTStarReply reply;
		reply.waypoint_amount = 0;
		reply.success = false;
		if(octomap_init)
		{
			learnTf();



			// octomath::Vector3 disc_initial(path_request->start.x, path_request->start.y, path_request->start.z);
			// octomath::Vector3 disc_final(path_request->goal.x, path_request->goal.y, path_request->goal.z);
			// octomath::Vector3 geofence(path_request->safety_margin, path_request->safety_margin, path_request->safety_margin);
			// getCorridorOccupancy_reboot(*octree, disc_initial, disc_final, geofence, marker_pub, true);
			// getCorridorOccupancy       (*octree, disc_initial, disc_final, geofence, marker_pub, true);
		}
		else
		{
			ROS_ERROR_STREAM("[LTStar] Cannot generate path because no octomap has been received.");
			reply.success=false;
			reply.request_id = path_request->request_id;
			reply.waypoint_amount = 0;
		}
		
	}
}

int main(int argc, char **argv)
{
	LazyThetaStarOctree::publish_free_corridor_arrows = true;
	ros::init(argc, argv, "ltstar_debug_node");
	ros::NodeHandle nh;
	LazyThetaStarOctree::marker_pub = nh.advertise<visualization_msgs::MarkerArray>("ltstar_path", 1);
	ros::Subscriber octomap_sub = nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 10, LazyThetaStarOctree::octomap_callback);
	ros::Subscriber ltstar_benchmark_sub = nh.subscribe<path_planning_msgs::LTStarBenchmarkRequest>("ltstar_request_benchmark", 10, LazyThetaStarOctree::ltstar_benchmark_callback_debug);
	LazyThetaStarOctree::ltstar_reply_pub = nh.advertise<path_planning_msgs::LTStarReply>("ltstar_reply", 10);

  	ros::spin();
}
