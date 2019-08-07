#include <ros/ros.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <atomic>

#include <goal_state_machine.h>
#include <marker_publishing_utils.h>
#include <architecture_msgs/FindNextGoal.h>
#include <architecture_msgs/DeclareUnobservable.h>
#include <architecture_msgs/PositionMiddleMan.h>
#include <frontiers_msgs/FindFrontiers.h>


namespace goal_sm_node
{
    ros::Publisher marker_pub;
    int count;

    octomap::OcTree* octree;
    octomap::OcTree* octree_inUse;
    std::atomic<bool> delete_octree_in_use;
    std::atomic<bool> discard_octree;
    bool octomap_init;

    std::shared_ptr<goal_state_machine::GoalStateMachine> goal_state_machine;
    ros::ServiceClient find_frontiers_client,  current_position_client, declare_unobservable_service;


    bool getUavPositionServiceCall(geometry_msgs::Point& current_position)
    {
        architecture_msgs::PositionMiddleMan srv;
        if(current_position_client.call(srv))
        {
            current_position = srv.response.current_position;
            return true;
        }
        else
        {
            ROS_WARN("[Goal SM] Current position middle man node not accepting requests.");
            return false;
        }
    }

    void updateOctree()
    {
        if (delete_octree_in_use){
            delete octree_inUse;
            count ++;
        }
        octree_inUse = octree;
        delete_octree_in_use = false;
        discard_octree = false;

        count ++;
    }

    bool find_next_goal(architecture_msgs::FindNextGoal::Request  &req,
        architecture_msgs::FindNextGoal::Response &res)
    {
        geometry_msgs::Point current_position;
        while(!getUavPositionServiceCall(current_position));
        Eigen::Vector3d current_position_e (current_position.x, current_position.y, current_position.z);

        std::ofstream log_file;
        std::stringstream aux_envvar_home (std::getenv("HOME"));
        std::string folder_name = aux_envvar_home.str() + "/Flying_Octomap_code/src/data";
        log_file.open (folder_name+"/current/state_manager.log", std::ofstream::app);
        log_file<<std::endl<<" ===== [Goal SM] ===== "<<std::endl;

        if(req.new_map)
        {
            log_file<<"[Goal SM] New map. "<<std::endl;
            goal_state_machine->NewMap();
            updateOctree();
            goal_state_machine->octree = octree_inUse;
            goal_state_machine->findFrontiers_CallService(current_position_e);
        }
        res.success = goal_state_machine->NextGoal(current_position_e);

        if(res.success)
        {
            goal_state_machine->getFlybyStart(res.start_flyby);
            goal_state_machine->getFlybyEnd(res.end_flyby);
            res.unknown = goal_state_machine->get_current_frontier();
            goal_state_machine->publishGoalToRviz(current_position);
            geometry_msgs::Point frontier_geom = goal_state_machine->get_current_frontier();
            log_file << "[Goal SM] Next unknown is (" << res.unknown.x << ", " << res.unknown.y << ", " << res.unknown.y << "). Viewed from (" << res.start_flyby.x << ", " << res.start_flyby.y << ", " << res.start_flyby.z << ") to (" << res.end_flyby.x << ", " << res.end_flyby.y << ", " << res.end_flyby.z << ")" <<  std::endl;
        }
        else
        {
            log_file << "[Goal SM] No goal available " << std::endl;
        }
        log_file.close();
		return true;
	}

    void octomap_cb(const octomap_msgs::Octomap::ConstPtr& octomapBinary){
        if (discard_octree)
        {
            delete octree;
        }
        else
        {
            delete_octree_in_use = true;
            count++;
        }
        discard_octree = true;
        octree = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*octomapBinary);
        octomap_init = true;
    }

	bool declare_unobservable(architecture_msgs::DeclareUnobservable::Request  &req,
		architecture_msgs::DeclareUnobservable::Response &res)
	{
        goal_state_machine->DeclareUnobservable();
        return true;
	}

    void init_state_variables(ros::NodeHandle& nh)
    {
        delete_octree_in_use = false;
        discard_octree = true;
        count  = 0;

		// Geofence
	    geometry_msgs::Point geofence_min , geofence_max ;
        float x, y, z;
        nh.getParam("geofence_min/x", geofence_min.x);
        nh.getParam("geofence_min/y", geofence_min.y);
        nh.getParam("geofence_min/z", geofence_min.z);
        nh.getParam("geofence_max/x", geofence_max.x);
        nh.getParam("geofence_max/y", geofence_max.y);
        nh.getParam("geofence_max/z", geofence_max.z);

        // Goal state machine
        double sensing_distance, distance_inFront, distance_behind, circle_divisions, ltstar_safety_margin;
        nh.getParam("oppairs/sensing_distance", sensing_distance);
        nh.getParam("oppairs/distance_inFront", distance_inFront);
        nh.getParam("oppairs/distance_behind",  distance_behind);
        nh.getParam("oppairs/circle_divisions",  circle_divisions);

        nh.getParam("path/safety_margin", ltstar_safety_margin);

        int range;
        nh.getParam("/octomap_builder/sensor_model/max_range", range);

        ros::ServiceClient check_visibility_client;
    	ros::ServiceClient check_flightCorridor_client;// = nh.serviceClient<lazy_theta_star_msgs::CheckFlightCorridor>("is_fligh_corridor_free");
        rviz_interface::PublishingInput pi(marker_pub, true, "oppairs" );
    	goal_state_machine = std::make_shared<goal_state_machine::GoalStateMachine>(find_frontiers_client, distance_inFront, distance_behind, circle_divisions, geofence_min, geofence_max, pi, ltstar_safety_margin, sensing_distance, range);


    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "goal_sm_node");
    ros::NodeHandle nh;

    goal_sm_node::marker_pub 	  					= nh.advertise<visualization_msgs::MarkerArray>("goal_sm", 1);
    goal_sm_node::find_frontiers_client         	= nh.serviceClient<frontiers_msgs::FindFrontiers>("find_frontiers");
    goal_sm_node::current_position_client   		= nh.serviceClient<architecture_msgs::PositionMiddleMan> ("get_current_position");
	ros::ServiceServer find_next_goal_service   	= nh.advertiseService("find_next_goal", goal_sm_node::find_next_goal);
	ros::ServiceServer declare_unobservable_service	= nh.advertiseService("declare_unobservable", goal_sm_node::declare_unobservable);
    ros::Subscriber    octomap_sub         			= nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 10, goal_sm_node::octomap_cb);

    goal_sm_node::init_state_variables(nh);

    ros::spin();
}