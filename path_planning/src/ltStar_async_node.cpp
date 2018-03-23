#include <ros/ros.h>
#include <ltStar_temp.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/Marker.h>

namespace LazyThetaStarOctree
{
    octomap::OcTree* octree;
	ros::Publisher ltstar_reply_pub;
	ros::Publisher marker_pub;
		
	bool octomap_init;

	void publish_cube(octomath::Vector3 & candidate, double size, int color, int waypoint_id)
	{	
		uint32_t shape = visualization_msgs::Marker::CUBE;
		visualization_msgs::Marker marker;
	    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
	    marker.header.frame_id = "/map";
	    marker.header.stamp = ros::Time::now();
	    marker.ns = "waypoint ";
    	marker.id = waypoint_id;
    	marker.type = shape;
    	marker.action = visualization_msgs::Marker::ADD;
    	marker.pose.position.x = candidate.x();
	    marker.pose.position.y = candidate.y();
	    marker.pose.position.z = candidate.z();
	    marker.pose.orientation.w = 1.0;
	    marker.scale.x = size;
	    marker.scale.y = size;
	    marker.scale.z = size;
	    marker.color.r = color;
	    marker.color.g = color;
	    marker.color.b = color;
	    marker.color.a = 0.5;
	    
	    marker.lifetime = ros::Duration(5);
	    marker_pub.publish(marker);
	}

	void publish_arrow(octomath::Vector3 & start, octomath::Vector3 & goal, int request_id)
	{
		uint32_t shape = visualization_msgs::Marker::ARROW;
		visualization_msgs::Marker marker;
	    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
	    marker.header.frame_id = "/map";
	    marker.header.stamp = ros::Time::now();
	    marker.ns = "path";
    	marker.id = request_id;
    	marker.type = shape;
    	geometry_msgs::Point goal_point;
    	goal_point.x = goal.x();
	    goal_point.y = goal.y();
	    goal_point.z = goal.z();
        marker.points.push_back(goal_point);
    	marker.action = visualization_msgs::Marker::ADD;
    	geometry_msgs::Point start_point;
    	start_point.x = start.x();
	    start_point.y = start.y();
	    start_point.z = start.z();
        marker.points.push_back(start_point);
	    marker.pose.orientation.w = 1.0;
	    marker.scale.x = 0.1;
	    marker.scale.y = 0.3;
	    marker.scale.z = 0;
	    marker.color.r = 200;
	    marker.color.g = 100;
	    marker.color.b = 0;
	    marker.color.a = 1;
	    
	    marker.lifetime = ros::Duration(5);
	    marker_pub.publish(marker);
	}

	bool check_status(path_planning_msgs::LTStarNodeStatus::Request  &req,
        path_planning_msgs::LTStarNodeStatus::Response &res)
	{
		res.is_accepting_requests = octomap_init;
	  	return true;
	}

	void ltstar_callback(const path_planning_msgs::LTStarRequest::ConstPtr& path_request)
	{
		path_planning_msgs::LTStarReply reply;
		reply.waypoint_amount = 0;
		reply.success = false;
		if(octomap_init)
		{
			LazyThetaStarOctree::processLTStarRequest(*octree, *path_request, reply);
			if(reply.waypoint_amount == 1)
			{
				ROS_ERROR_STREAM("[LTStar] The resulting path has only one waypoint. It should always have at least start and goal. Here is the request message (the octree was saved to /data) " << *path_request);
				ROS_ERROR_STREAM("[LTStar] And here is the reply " << reply);
				octree->writeBinary("/data/one_waypointed_path.bt");
			}
		}
		else
		{
			reply.success=false;
			reply.request_id = path_request->request_id;
			reply.waypoint_amount = 0;
		}
		ltstar_reply_pub.publish(reply);

		// Publish to rviz
		for (int i = 0; i < reply.waypoint_amount; ++i)
		{
			octomath::Vector3 candidate (reply.waypoints[i].x, reply.waypoints[i].y, reply.waypoints[i].z);
			std::unordered_set<std::shared_ptr<octomath::Vector3>> neighbors;
	        double resolution = octree->getResolution();
	        int tree_depth = octree->getTreeDepth();
	        octomap::OcTreeKey key = octree->coordToKey(candidate);
	        int depth = LazyThetaStarOctree::getNodeDepth_Octomap(key, *octree);
	        double voxel_size = ((tree_depth + 1) - depth) * resolution;
	        publish_cube(candidate, voxel_size, (200*i)/reply.waypoint_amount, i );
	        if(i !=0)
	        {
				octomath::Vector3 prev_candidate (reply.waypoints[i-1].x, reply.waypoints[i-1].y, reply.waypoints[i-1].z);
	        	publish_arrow(candidate, prev_candidate, path_request->request_id);
	        }
		}
	}
	
	void octomap_callback(const octomap_msgs::Octomap::ConstPtr& octomapBinary){
		delete octree;
		octree = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*octomapBinary);
		octomap_init = true;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ltstar_async_node");
	ros::NodeHandle nh;
	ros::ServiceServer ltstar_status_service = nh.advertiseService("ltstar_status", LazyThetaStarOctree::check_status);
	ros::Subscriber octomap_sub = nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 10, LazyThetaStarOctree::octomap_callback);
	ros::Subscriber ltstars_sub = nh.subscribe<path_planning_msgs::LTStarRequest>("ltstar_request", 10, LazyThetaStarOctree::ltstar_callback);
	LazyThetaStarOctree::ltstar_reply_pub = nh.advertise<path_planning_msgs::LTStarReply>("ltstar_reply", 10);
	LazyThetaStarOctree::marker_pub = nh.advertise<visualization_msgs::Marker>("ltstar_path", 1);

	ros::spin();
}