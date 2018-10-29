#include <ltStar_temp.h>
#include <gtest/gtest.h>
#include <queue>


namespace LazyThetaStarOctree{
	typedef bool (*lazyThetaStar_function)(octomap::OcTree & octree, lazy_theta_star_msgs::LTStarRequest const& request, lazy_theta_star_msgs::LTStarReply & reply, const double sidelength_lookup_table[], PublishingInput const& publish_input) ;


	bool checkPreconditions(InputData input, PublishingInput publish_input)
	{		
		// Initial node is not occupied
		octomap::OcTreeNode* originNode = input.octree.search(input.start);
		if(  originNode  ){}
		else
		{
			std::ofstream log_file;
			log_file.open (LazyThetaStarOctree::folder_name + "/current/tests.log", std::ofstream::app);
			log_file << "[Precondition failed] Start node in unknown space. " << input.start << std::endl;
			log_file.close();
			return false;
		}
		if(input.octree.isNodeOccupied(originNode) != false)
		{
			std::ofstream log_file;
			log_file.open (LazyThetaStarOctree::folder_name + "/current/tests.log", std::ofstream::app);
			log_file << "[Precondition failed] Start node in occupied space" << input.start << std::endl;
			log_file.close();
			return false;
		}
		// Final node is not occupied
		octomap::OcTreeNode* finalNode = input.octree.search(input.goal);
		if(  finalNode  ) {}
		else
		{
			std::ofstream log_file;
			log_file.open (LazyThetaStarOctree::folder_name + "/current/tests.log", std::ofstream::app);
			log_file << "[Precondition failed] Final node in unknown space" << input.goal << std::endl;
			log_file.close();
			return false;
		}
		if(input.octree.isNodeOccupied(finalNode) != false)
		{
			std::ofstream log_file;
			log_file.open (LazyThetaStarOctree::folder_name + "/current/tests.log", std::ofstream::app);
			log_file << "[Precondition failed] Final node in occupied space" << input.goal << std::endl;
			log_file.close();
			return false;
		}
		float distance = weightedDistance(input.start, input.goal);
		if( distance > input.octree.getResolution()*200)
		{
			std::ofstream log_file;
			log_file.open (LazyThetaStarOctree::folder_name + "/current/tests.log", std::ofstream::app);
			log_file << "[Precondition failed] Distance too big. From " << input.start << " to " << input.goal << " is " << distance << " ( > " << input.octree.getResolution()*200 << " ) " << std::endl;
			log_file.close();
			return false;

		}
		// The path is clear from start to finish  
		if(is_flight_corridor_free	(input, publish_input, false))
		{
			std::ofstream log_file;
			log_file.open (LazyThetaStarOctree::folder_name + "/current/tests.log", std::ofstream::app);
			log_file << "[Spars][Precondition failed] This is a test with obstacles but there are none, skipping. From " << input.start << " to " << input.goal << std::endl;
			log_file.close();
			return false;

		}
		return true;
	}

	bool checkPostConditions(octomap::OcTree & octree, lazy_theta_star_msgs::LTStarReply reply, InputData input)
	{
		double sidelength_lookup_table  [octree.getTreeDepth()];
	   	LazyThetaStarOctree::fillLookupTable( octree.getResolution(), octree.getTreeDepth(), sidelength_lookup_table); 
		octomath::Vector3 result_start (reply.waypoints[0].position.x, reply.waypoints[0].position.y, reply.waypoints[0].position.z);
		int last_waypoint_index = reply.waypoint_amount - 1;
		octomath::Vector3 result_goal (reply.waypoints[last_waypoint_index].position.x, reply.waypoints[last_waypoint_index].position.y, reply.waypoints[last_waypoint_index].position.z);

		double cell_size_goal = -1;
		octomath::Vector3 cell_center_coordinates_goal = input.goal;
		updateToCellCenterAndFindSize( cell_center_coordinates_goal, octree, cell_size_goal, sidelength_lookup_table);
		if(result_goal.distance( cell_center_coordinates_goal ) >= cell_size_goal   )
		{
			ROS_ERROR_STREAM("Distance from result last point " << result_goal << " and center of the voxel the goal is in " << cell_center_coordinates_goal << " is larger than the size of that voxel." << cell_size_goal);
			return false;
		}
		double cell_size_start = -1;
		octomath::Vector3 cell_center_coordinates_start = input.start;
		updateToCellCenterAndFindSize( cell_center_coordinates_start, octree, cell_size_start, sidelength_lookup_table);
		if( result_start.distance( cell_center_coordinates_start ) >=  cell_size_start   )
		{
			ROS_ERROR_STREAM("Distance from result first point " << result_start << " and center of the voxel the start is in " << cell_center_coordinates_start << " is larger than the size of that voxel." << cell_size_start);
			return false;
		}
		return true;
	}
	
	bool testStraightLinesForwardWithObstacles(octomap::OcTree & octree, octomath::Vector3 disc_initial, octomath::Vector3 disc_final, lazyThetaStar_function processLazyThetaStar, int const& max_time_secs = 55, double safety_margin = 2, std::string dataset_name = "unnamed")
	{
		ros::Publisher marker_pub;
		PublishingInput publish_input( marker_pub, true, dataset_name);
		InputData input( octree, disc_initial, disc_final, safety_margin);
		ResultSet statistical_data;
		double sidelength_lookup_table  [octree.getTreeDepth()];
	   	LazyThetaStarOctree::fillLookupTable( octree.getResolution(), octree.getTreeDepth(), sidelength_lookup_table); 


		std::ofstream log_file_;
		log_file_.open (LazyThetaStarOctree::folder_name + "/current/tests.log", std::ofstream::app);
		log_file_ << "[Spars] Testing from " << input.start << " to " << input.goal << "; safety_margin: " << safety_margin << "; max_time_secs: " << max_time_secs << std::endl;
		log_file_.close();
		
		// std::list<octomath::Vector3> resulting_path = lazyThetaStar_(input, statistical_data, sidelength_lookup_table, publish_input, max_time_secs, true);

		if(!checkPreconditions(input, publish_input))
		{
			return false;
		}

		lazy_theta_star_msgs::LTStarRequest request;
        request.request_id = 0;
        request.header.frame_id = "world";
        request.start.x = disc_initial.x();
        request.start.y = disc_initial.y();
        request.start.z = disc_initial.z();
        request.goal.x = disc_final.x();
        request.goal.y = disc_final.y();
        request.goal.z = disc_final.z();
        request.max_time_secs = max_time_secs;
        request.safety_margin = safety_margin;
        lazy_theta_star_msgs::LTStarReply reply;

		if( ! processLazyThetaStar(octree, request, reply, sidelength_lookup_table, publish_input) )
		{
			ROS_ERROR_STREAM("Failure from " << disc_initial << " to " << disc_final);
			return false;
		}
		else
		{
			if(reply.success)
			{
				if (!checkPostConditions(octree, reply, input)) return false;
			}
		}
		if(0 != ThetaStarNode::OustandingObjects())
		{
			ROS_ERROR_STREAM("Memory leak from ThetaStarNode objects.");
		}
		return true;
	}

	void collectDate(octomap::OcTree & octree, double max_time_secs, double safety_margin, std::string dataset_name, lazyThetaStar_function processLazyThetaStar, std::list<octomath::Vector3> points)
	{
	  	
		int count = points.size()-1;
	    int i;
	    for ( i = 0; i < count; )
	    {
	    	octomath::Vector3 current = *(points.begin());
	    	points.erase(points.begin());
	    	for (std::list<octomath::Vector3>::iterator it = points.begin(); it != points.end(); ++it)
	    	{
	    		testStraightLinesForwardWithObstacles(octree, current, *it, processLazyThetaStar, max_time_secs, safety_margin, dataset_name);
	    		testStraightLinesForwardWithObstacles(octree, *it, current, processLazyThetaStar, max_time_secs, safety_margin, dataset_name);
	    	}
	    	++i;
	    }
	}

	// TEST(LazyThetaStarMeasurements, SparseNeighbors)
	// {
	// 	std::ofstream csv_file;
	// 	csv_file.open (LazyThetaStarOctree::folder_name + "/current/lazyThetaStar_computation_time.csv", std::ofstream::app);
	// 	csv_file << "success,computation_time_millis,path_lenght_straight_line_meters,path_lenght_total_meters,has_obstacle,start,goal,safety_margin_meters,max_search_duration_seconds,iteration_count,obstacle_hit_count,total_obstacle_checks,dataset_name" << std::endl;
	// 	csv_file.close();

	//     octomap::OcTree octree ("data/3dPuzzle_05.bt");
	//     double max_time_secs = 60;
	//     lazyThetaStar_function processLazyThetaStar = processLTStarRequest;

	//     std::list<octomath::Vector3> points = {
	// 	  		/*A = */octomath::Vector3 (0.28, 13.3, 90),
	// 	  		/*B = */octomath::Vector3 (-21.6, 20.3, 63),
	// 	  		/*C = */octomath::Vector3 (-34.9, 20.2, 32.0),
	// 	  		/*D = */octomath::Vector3 (-20.4, 12.4, 11.5),
	// 	  		/*E = */octomath::Vector3 (41.4, 22.4, 32),
	// 	  		/*F = */octomath::Vector3 (21.6, 23.5, 12.5),
	//   			octomath::Vector3 (0.0, -5.0, 10),
	//   			octomath::Vector3 (-5.0, 0.0, 40),
	//   			octomath::Vector3 (-20, 12.5, 40.0),
	//   			octomath::Vector3 (0, 32, 40.0),
	//   			octomath::Vector3 (0, 32, 15.0),
	//   			octomath::Vector3 (30, 47, 15.0),
	//   			octomath::Vector3 (30, 47, 50.0),
	//   			octomath::Vector3 (48, 4, 50.0),
	//   			octomath::Vector3 (48, 4, 20.0),
	//   			octomath::Vector3 (31, 2, 20.0),
	//   			octomath::Vector3 (31, 2, 45.0),
	//   			octomath::Vector3 (16, 37, 45.0),
	//   			octomath::Vector3 (16, 37, 10.0),
	//   			octomath::Vector3 (-3, 31, 77.0),
	//   			octomath::Vector3 (-7, 13, 26),
	//   			octomath::Vector3 (10, 17, 26),
	//   			octomath::Vector3 (10, 17, 46),
	//   			octomath::Vector3 (14, 0, 46),
	//   			octomath::Vector3 (14, 0, 26),
	//   			octomath::Vector3 (-10, 18, 63),
	//   			octomath::Vector3 (6, -6, 63),
	//   			octomath::Vector3 (6, -6, 63),
	//   			octomath::Vector3 (5, 32, 63),
	//   			octomath::Vector3 (27, 30, 63),
	//   			octomath::Vector3 (37, 9, 63),
	//   			octomath::Vector3 (-2.8, 0.7, 10),
	//   			octomath::Vector3 (-2.8, 0.7, 15),
	//   			octomath::Vector3 (-2.2, 9.7, 20),
	//   			octomath::Vector3 (-3, 6.7, 20),
	//   			octomath::Vector3 (8.7, 22.7, 20),
	//   			octomath::Vector3 (5.46, 25.9, 15),
	//   			octomath::Vector3 (7, 21.7, 20),
	// 		};

	//     collectDate(octree, max_time_secs, 3.9, "3Dpuzzle_ortho_3.9margin_sparse", 	processLazyThetaStar, points);
	//     collectDate(octree, max_time_secs, 5, 	"3Dpuzzle_ortho_5margin_sparse", 	processLazyThetaStar, points);
	//     collectDate(octree, max_time_secs, 5.4, "3Dpuzzle_ortho_5.4margin_sparse", 	processLazyThetaStar, points);
	// }

	// TEST(LazyThetaStarMeasurements, OriginalCode)
	// {
	// 	std::ofstream csv_file;
	// 	csv_file.open (LazyThetaStarOctree::folder_name + "/current/lazyThetaStar_computation_time.csv", std::ofstream::app);
	// 	csv_file << "success,computation_time_millis,path_lenght_straight_line_meters,path_lenght_total_meters,has_obstacle,start,goal,safety_margin_meters,max_search_duration_seconds,iteration_count,obstacle_hit_count,total_obstacle_checks,dataset_name" << std::endl;
	// 	csv_file.close();

	//     octomap::OcTree octree ("data/3dPuzzle_05.bt");
	//     double max_time_secs = 60;
	//     lazyThetaStar_function processLazyThetaStar = processLTStarRequest_original;

	//     std::list<octomath::Vector3> points = {
	// 	  		/*A = */octomath::Vector3 (0.28, 13.3, 90),
	// 	  		/*B = */octomath::Vector3 (-21.6, 20.3, 63),
	// 	  		/*C = */octomath::Vector3 (-34.9, 20.2, 32.0),
	// 	  		/*D = */octomath::Vector3 (-20.4, 12.4, 11.5),
	// 	  		/*E = */octomath::Vector3 (41.4, 22.4, 32),
	// 	  		/*F = */octomath::Vector3 (21.6, 23.5, 12.5),
	//   			octomath::Vector3 (0.0, -5.0, 10),
	//   			octomath::Vector3 (-5.0, 0.0, 40),
	//   			octomath::Vector3 (-20, 12.5, 40.0),
	//   			octomath::Vector3 (0, 32, 40.0),
	//   			octomath::Vector3 (0, 32, 15.0),
	//   			octomath::Vector3 (30, 47, 15.0),
	//   			octomath::Vector3 (30, 47, 50.0),
	//   			octomath::Vector3 (48, 4, 50.0),
	//   			octomath::Vector3 (48, 4, 20.0),
	//   			octomath::Vector3 (31, 2, 20.0),
	//   			octomath::Vector3 (31, 2, 45.0),
	//   			octomath::Vector3 (16, 37, 45.0),
	//   			octomath::Vector3 (16, 37, 10.0),
	//   			octomath::Vector3 (-3, 31, 77.0),
	//   			octomath::Vector3 (-7, 13, 26),
	//   			octomath::Vector3 (10, 17, 26),
	//   			octomath::Vector3 (10, 17, 46),
	//   			octomath::Vector3 (14, 0, 46),
	//   			octomath::Vector3 (14, 0, 26),
	//   			octomath::Vector3 (-10, 18, 63),
	//   			octomath::Vector3 (6, -6, 63),
	//   			octomath::Vector3 (5, 32, 63),
	//   			octomath::Vector3 (27, 30, 63),
	//   			octomath::Vector3 (37, 9, 63),
	//   			octomath::Vector3 (-2.8, 0.7, 10),
	//   			octomath::Vector3 (-2.8, 0.7, 15),
	//   			octomath::Vector3 (-2.2, 9.7, 20),
	//   			octomath::Vector3 (-3, 6.7, 20),
	//   			octomath::Vector3 (8.7, 22.7, 20),
	//   			octomath::Vector3 (5.46, 25.9, 15),
	//   			octomath::Vector3 (7, 21.7, 20),
	// 		};

	//     collectDate(octree, max_time_secs, 3.9, "3Dpuzzle_ortho_3.9margin_original", 	processLazyThetaStar, points);
	//     collectDate(octree, max_time_secs, 5, 	"3Dpuzzle_ortho_5margin_original", 		processLazyThetaStar, points);
	//     collectDate(octree, max_time_secs, 5.4, "3Dpuzzle_ortho_5.4margin_original", 	processLazyThetaStar, points);
	// }

	TEST(LazyThetaStarMeasurements, SparseNeighbors_experimental)
	{
		std::ofstream csv_file;
		csv_file.open (LazyThetaStarOctree::folder_name + "/current/lazyThetaStar_computation_time.csv", std::ofstream::app);
		csv_file << "success,computation_time_millis,path_lenght_straight_line_meters,path_lenght_total_meters,has_obstacle,start,goal,safety_margin_meters,max_search_duration_seconds,iteration_count,obstacle_hit_count,total_obstacle_checks,dataset_name" << std::endl;
		csv_file.close();

	    double max_time_secs = 1;
	    lazyThetaStar_function processLazyThetaStar = processLTStarRequest;

	    std::list<octomath::Vector3> points = {
			octomath::Vector3 (6.22, -7.23, 7),
			octomath::Vector3 (0.69, -11.2, 7),
			octomath::Vector3 (-23.5, -28.4, 3),
			octomath::Vector3 (-15.7, -5.08, 5),
			octomath::Vector3 (-8.34, -10.9, 7),
			octomath::Vector3 (-5.35, -14, 7),
			octomath::Vector3 (-3.9, -14.76, 7),
			octomath::Vector3 (4.12, -2.21, 7),
			octomath::Vector3 (7.49, -6.98, 7),
			octomath::Vector3 (11.3, -6.71, 7),
			octomath::Vector3 (16.8, -4.22, 5),
			octomath::Vector3 (13.9, -18.2, 3),
			};


	    octomap::OcTree octree_v1 ("data/20180821_1207_5647_filtered.bt");
	    collectDate(octree_v1, max_time_secs, 3.9, "20180821_1207_5647_filtered_experimental_sparse", 	processLazyThetaStar, points);
	    collectDate(octree_v1, max_time_secs, 5, 	"20180821_1207_5647_filtered_experimental_sparse", 	processLazyThetaStar, points);
	    collectDate(octree_v1, max_time_secs, 5.4, "20180821_1207_5647_filtered_experimental_sparse", 	processLazyThetaStar, points);


	    octomap::OcTree octree_v2 ("data/20180821_1110_42936_raw.bt");
	    collectDate(octree_v2, max_time_secs, 3.9, "20180821_1110_42936_raw_experimental_sparse", 	processLazyThetaStar, points);
	    collectDate(octree_v2, max_time_secs, 5, 	"20180821_1110_42936_raw_experimental_sparse", 	processLazyThetaStar, points);
	    collectDate(octree_v2, max_time_secs, 5.4, "20180821_1110_42936_raw_experimental_sparse", 	processLazyThetaStar, points);

	    octomap::OcTree octree_v3 ("data/20180821_1110_42712_raw.bt");
	    collectDate(octree_v3, max_time_secs, 3.9, "20180821_1110_42712_raw_experimental_sparse", 	processLazyThetaStar, points);
	    collectDate(octree_v3, max_time_secs, 5, 	"20180821_1110_42712_raw_experimental_sparse", 	processLazyThetaStar, points);
	    collectDate(octree_v3, max_time_secs, 5.4, "20180821_1110_42712_raw_experimental_sparse", 	processLazyThetaStar, points);

	}


	TEST(LazyThetaStarMeasurements, Original_experimental)
	{
		std::ofstream csv_file;
		csv_file.open (LazyThetaStarOctree::folder_name + "/current/lazyThetaStar_computation_time.csv", std::ofstream::app);
		csv_file << "success,computation_time_millis,path_lenght_straight_line_meters,path_lenght_total_meters,has_obstacle,start,goal,safety_margin_meters,max_search_duration_seconds,iteration_count,obstacle_hit_count,total_obstacle_checks,dataset_name" << std::endl;
		csv_file.close();

	    double max_time_secs = 1;
	    lazyThetaStar_function processLazyThetaStar = processLTStarRequest_original;

	    std::list<octomath::Vector3> points = {
			octomath::Vector3 (6.22, -7.23, 7),
			octomath::Vector3 (0.69, -11.2, 7),
			octomath::Vector3 (-23.5, -28.4, 3),
			octomath::Vector3 (-15.7, -5.08, 5),
			octomath::Vector3 (-8.34, -10.9, 7),
			octomath::Vector3 (-5.35, -14, 7),
			octomath::Vector3 (-3.9, -14.76, 7),
			octomath::Vector3 (4.12, -2.21, 7),
			octomath::Vector3 (7.49, -6.98, 7),
			octomath::Vector3 (11.3, -6.71, 7),
			octomath::Vector3 (16.8, -4.22, 5),
			octomath::Vector3 (13.9, -18.2, 3),
			};


	    octomap::OcTree octree_v1 ("data/20180821_1207_5647_filtered.bt");
	    collectDate(octree_v1, max_time_secs, 3.9, "20180821_1207_5647_filtered_experimental_original", 	processLazyThetaStar, points);
	    collectDate(octree_v1, max_time_secs, 5, 	"20180821_1207_5647_filtered_experimental_original", 	processLazyThetaStar, points);
	    collectDate(octree_v1, max_time_secs, 5.4, "20180821_1207_5647_filtered_experimental_original", 	processLazyThetaStar, points);


	    octomap::OcTree octree_v2 ("data/20180821_1110_42936_raw.bt");
	    collectDate(octree_v2, max_time_secs, 3.9, "20180821_1110_42936_raw_experimental_original", 	processLazyThetaStar, points);
	    collectDate(octree_v2, max_time_secs, 5, 	"20180821_1110_42936_raw_experimental_original", 	processLazyThetaStar, points);
	    collectDate(octree_v2, max_time_secs, 5.4, "20180821_1110_42936_raw_experimental_original", 	processLazyThetaStar, points);

	    octomap::OcTree octree_v3 ("data/20180821_1110_42712_raw.bt");
	    collectDate(octree_v3, max_time_secs, 3.9, "20180821_1110_42712_raw_experimental_original", 	processLazyThetaStar, points);
	    collectDate(octree_v3, max_time_secs, 5, 	"20180821_1110_42712_raw_experimental_original", 	processLazyThetaStar, points);
	    collectDate(octree_v3, max_time_secs, 5.4, "20180821_1110_42712_raw_experimental_original", 	processLazyThetaStar, points);

	}


}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
