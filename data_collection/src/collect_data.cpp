#include <collect_data.h>
#include <neighbors.h>


namespace collect_data
{
	bool isUnknown(octomap::OcTree& octree, octomath::Vector3 candidate)
	{
		try
		{
			octomap::OcTreeKey key = octree.coordToKey(candidate); 
	        LazyThetaStarOctree::getNodeDepth_Octomap(key, octree); 
	        return false;
		}
		catch(const std::out_of_range& oor)
		{
			return true;
		}
	}

	void writeCSVFile(std::chrono::system_clock::time_point &start_time, std::chrono::system_clock::time_point &end_time, frontiers_msgs::FindFrontiers::Request  &request, frontiers_msgs::FindFrontiers::Response &reply)
	{

		std::ofstream csv_file;
		csv_file.open (LazyThetaStarOctree::folder_name + "/current/lazyThetaStar_computation_time.csv", std::ofstream::app);
		csv_file << "success,computation_time_millis,request_frontier_amount,frontier_amount,dataset_name" << std::endl;
		csv_file.open (folder_name + "/current/exploration_measurements.csv", std::ofstream::app);
		std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
		std::chrono::milliseconds millis = std::chrono::duration_cast<std::chrono::milliseconds>(time_span);
		csv_file << (resulting_path.size()>0);
		csv_file << "," << reply.frontiers.size() > 0;
		csv_file << "," << millis.count();
		csv_file << "," << request.frontier_amount;
		csv_file << "," << reply.frontiers.size();
		// csv_file << ",(" <<  std::setprecision(2) << disc_initial.x() << "_"  << disc_initial.y() << "_"  << disc_initial.z() << ")";
		csv_file << "," << publish_input.dataset_name << std::endl;
		csv_file.close();
	}

	void emulateGoal()
	{
		// load octree
        rviz_interface::PublishingInput pi(marker_pub, false);
	    geometry_msgs::Point geofence_min , geofence_max ;
        double sensing_distance, distance_inFront, distance_behind, circle_divisions, ltstar_safety_margin;
    	goal_state_machine::GoalStateMachine goal_state_machine (find_frontiers_client, distance_inFront, distance_behind, circle_divisions, geofence_min, geofence_max, pi, ltstar_safety_margin, sensing_distance);

	}
}