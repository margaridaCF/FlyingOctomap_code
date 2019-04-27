#include <ros/ros.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <atomic>

#include <goal_state_machine.h>
#include <marker_publishing_utils.h>
#include <architecture_msgs/FindNextGoal.h>
#include <frontiers_msgs/FindFrontiers.h>


namespace goal_sm_node
{
    ros::Publisher marker_pub;


	octomap::OcTree* octree;
	octomap::OcTree* octree_inUse;
	std::atomic<bool> is_octree_inUse;
	std::atomic<bool> new_octree;
	bool octomap_init;

	std::shared_ptr<goal_state_machine::GoalStateMachine> goal_state_machine;
	ros::ServiceClient find_frontiers_client;

	void octomap_cb(const octomap_msgs::Octomap::ConstPtr& octomapBinary){
		if (!is_octree_inUse)
		{
			delete octree;
		}
		is_octree_inUse = false;
		new_octree = true;
		octree = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*octomapBinary);
		octomap_init = true;
	}

    bool find_next_goal(architecture_msgs::FindNextGoal::Request  &req,
		architecture_msgs::FindNextGoal::Response &res)
	{

		return true;
	}

    void init_state_variables(ros::NodeHandle& nh)
    {
		new_octree = false;
		is_octree_inUse = false;

		// Geofence
	    octomath::Vector3 geofence_min , geofence_max ;
        float x, y, z;
        nh.getParam("geofence_min/x", x);
        nh.getParam("geofence_min/y", y);
        nh.getParam("geofence_min/z", z);
        geofence_min = octomath::Vector3  (x, y, z);
        nh.getParam("geofence_max/x", x);
        nh.getParam("geofence_max/y", y);
        nh.getParam("geofence_max/z", z);
        geofence_max = octomath::Vector3  (x, y, z);

        // Goal state machine
        double sensing_distance, distance_inFront, distance_behind, circle_divisions, ltstar_safety_margin;
        nh.getParam("oppairs/sensing_distance", sensing_distance);
        nh.getParam("oppairs/distance_inFront", distance_inFront);
        nh.getParam("oppairs/distance_behind",  distance_behind);
        nh.getParam("oppairs/circle_divisions",  circle_divisions);

        nh.getParam("path/safety_margin", ltstar_safety_margin);


        ros::ServiceClient check_visibility_client;
    	ros::ServiceClient check_flightCorridor_client;// = nh.serviceClient<lazy_theta_star_msgs::CheckFlightCorridor>("is_fligh_corridor_free");
        rviz_interface::PublishingInput pi(marker_pub, true, "oppairs" );
    	goal_state_machine = std::make_shared<goal_state_machine::GoalStateMachine>(find_frontiers_client, distance_inFront, distance_behind, circle_divisions, geofence_min, geofence_max, pi, check_flightCorridor_client, ltstar_safety_margin, sensing_distance, check_visibility_client);

    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "goal_sm_node");
    ros::NodeHandle nh;

    goal_sm_node::marker_pub 	  				  = nh.advertise<visualization_msgs::MarkerArray>("goal_sm", 1);
    goal_sm_node::find_frontiers_client           = nh.serviceClient<frontiers_msgs::FindFrontiers>("find_frontiers");
	ros::ServiceServer find_next_goal_service     = nh.advertiseService("find_next_goal", goal_sm_node::find_next_goal);
    ros::Subscriber    octomap_sub         		  = nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 10, goal_sm_node::octomap_cb);

    goal_sm_node::init_state_variables(nh);

    ros::spin();
}