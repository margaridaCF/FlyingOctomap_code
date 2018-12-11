#include <ltStar_lib_ortho.h>
#include <gtest/gtest.h>
#include <queue>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <visualization_msgs/Marker.h>


#include <chrono>

namespace LazyThetaStarOctree
{
	bool testStraightLinesForwardWithObstacles(octomap::OcTree octree, octomath::Vector3 disc_initial, octomath::Vector3 disc_final,
		int const& max_time_secs = 55, double safety_margin = 2, std::string dataset_name = "unnamed")
	{

		std::ofstream log_file_;
		log_file_.open (LazyThetaStarOctree::folder_name + "/current/tests.log", std::ofstream::app);
		log_file_ << "Testing from " << disc_initial << " to " << disc_final << "; safety_margin: " << safety_margin << "; max_time_secs: " << max_time_secs << std::endl;
		log_file_.close();

		ros::Publisher marker_pub;
		ResultSet statistical_data;
		double sidelength_lookup_table  [octree.getTreeDepth()];
		PublishingInput publish_input( marker_pub, true, dataset_name);
		InputData input( octree, disc_initial, disc_final, safety_margin);
	   	LazyThetaStarOctree::fillLookupTable( octree.getResolution(), octree.getTreeDepth(), sidelength_lookup_table); 
		generateOffsets(octree.getResolution(), safety_margin, dephtZero, semiSphereOut );
		
		bool success = false;
		// Initial node is not occupied
		octomap::OcTreeNode* originNode = octree.search(disc_initial);
		if(  originNode  ){}
		else
		{
			std::ofstream log_file;
			log_file.open (LazyThetaStarOctree::folder_name + "/current/tests.log", std::ofstream::app);
			log_file << "[Precondition failed] Start node in unknown space. " << disc_initial << std::endl;
			log_file.close();
			return false;
		}
		if(octree.isNodeOccupied(originNode) != false)
		{
			std::ofstream log_file;
			log_file.open (LazyThetaStarOctree::folder_name + "/current/tests.log", std::ofstream::app);
			log_file << "[Precondition failed] Start node in occupied space" << disc_initial << std::endl;
			log_file.close();
			return false;
		}
		// Final node is not occupied
		octomap::OcTreeNode* finalNode = octree.search(disc_final);
		if(  finalNode  ) {}
		else
		{
			std::ofstream log_file;
			log_file.open (LazyThetaStarOctree::folder_name + "/current/tests.log", std::ofstream::app);
			log_file << "[Precondition failed] Final node in unknown space" << disc_final << std::endl;
			log_file.close();
			return false;
		}
		if(octree.isNodeOccupied(finalNode) != false)
		{
			std::ofstream log_file;
			log_file.open (LazyThetaStarOctree::folder_name + "/current/tests.log", std::ofstream::app);
			log_file << "[Precondition failed] Final node in occupied space" << disc_final << std::endl;
			log_file.close();
			return false;
		}
		// The path is clear from start to finish  
		if(is_flight_corridor_free	(input, publish_input, false))
		{
			std::ofstream log_file;
			log_file.open (LazyThetaStarOctree::folder_name + "/current/tests.log", std::ofstream::app);
			log_file << "[Precondition failed] This is a test with obstacles but there are none, skipping. From " << disc_initial << " to " << disc_final << std::endl;
			log_file.close();
			return false;

		}
		
		// std::list<octomath::Vector3> resulting_path = lazyThetaStar_(input, statistical_data, sidelength_lookup_table, publish_input, max_time_secs, true);

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

		if( ! processLTStarRequest(octree, request, reply, sidelength_lookup_table, publish_input) )
		{
			ROS_ERROR_STREAM("Failure from " << disc_initial << " to " << disc_final);
			return false;
		}
		else
		{
			ROS_ERROR_STREAM("This is me");
			// CANONICAL: straight line, no issues
			// 2 waypoints: The center of start voxel & The center of the goal voxel
			if(reply.success)
			{
				octomath::Vector3 result_start (reply.waypoints[0].position.x, reply.waypoints[0].position.y, reply.waypoints[0].position.z);
				int last_waypoint_index = reply.waypoint_amount - 1;
				octomath::Vector3 result_goal (reply.waypoints[last_waypoint_index].position.x, reply.waypoints[last_waypoint_index].position.y, reply.waypoints[last_waypoint_index].position.z);

				double cell_size_goal = -1;
				octomath::Vector3 cell_center_coordinates_goal = disc_final;
				updateToCellCenterAndFindSize( cell_center_coordinates_goal, octree, cell_size_goal, sidelength_lookup_table);
				if(result_goal.distance( cell_center_coordinates_goal ) >= cell_size_goal   )
				{
					ROS_ERROR_STREAM("Distance from result last point " << result_goal << " and center of the voxel the goal is in " << cell_center_coordinates_goal << " is larger than the size of that voxel." << cell_size_goal);
					return false;
				}
				double cell_size_start = -1;
				octomath::Vector3 cell_center_coordinates_start = disc_initial;
				updateToCellCenterAndFindSize( cell_center_coordinates_start, octree, cell_size_start, sidelength_lookup_table);
				if( result_start.distance( cell_center_coordinates_start ) >=  cell_size_start   )
				{
					ROS_ERROR_STREAM("Distance from result first point " << result_start << " and center of the voxel the start is in " << cell_center_coordinates_start << " is larger than the size of that voxel." << cell_size_start);
					return false;
				}
			}
		}
		if(0 != ThetaStarNode::OustandingObjects())
		{
			ROS_ERROR_STREAM("Memory leak from ThetaStarNode objects.");
		}
		return true;
	}

	void collectDate(octomap::OcTree & octree, double max_time_secs, double safety_margin, std::string dataset_name)
	{
	  	std::list<octomath::Vector3> points = {
		  		/*A = */octomath::Vector3 (0.28, 13.3, 95),
		  // 		/*B = */octomath::Vector3 (-21.6, 20.3, 63),
		  // 		/*C = */octomath::Vector3 (-34.9, 20.2, 32.0),
		  // 		/*D = */octomath::Vector3 (-20.4, 12.4, 11.5),
		  // 		/*E = */octomath::Vector3 (41.4, 22.4, 32),
		  // 		/*F = */octomath::Vector3 (21.6, 23.5, 12.5),
		  // 		octomath::Vector3 (0, 0, 50),
				// octomath::Vector3 (0, -20, 30),
				octomath::Vector3 (-70, -20, 30), // <---
				// octomath::Vector3 (-70, 10, 95),
				// octomath::Vector3 (50, 10, 95),
				// octomath::Vector3 (50, 10, 65),
				// octomath::Vector3 (-70, 10, 65),
				// octomath::Vector3 (-35, 10, 50),
				// octomath::Vector3 (-35, 10, 10),
				// octomath::Vector3 (-70, 10, 10),
				// octomath::Vector3 (-70, 20, 30),
				// octomath::Vector3 (20, 20, 30),
				// octomath::Vector3 (-20, 20, 50),
				// octomath::Vector3 (-20, 20, 65) ,
				// octomath::Vector3 (70, 20, 65),
				// octomath::Vector3 (30, 20, 65),
				// octomath::Vector3 (0, 13, 60)  ,
				// octomath::Vector3 (0, 30, 100),
				// octomath::Vector3 (0, 30, 80),
				// octomath::Vector3 (0, 30, 100),
				// octomath::Vector3 (0, 0, 100),
				// octomath::Vector3 (0, 0, 75),
				// octomath::Vector3 (10, 0, 75),
				// octomath::Vector3 (10, 15, 75),
				// octomath::Vector3 (14, 15, 75),
				// octomath::Vector3 (14, 15, 20),
				// octomath::Vector3 (1, 15, 20),
				// octomath::Vector3 (1, 15, 10),
				// octomath::Vector3 (19.6, 29.7, 10),
				// octomath::Vector3 (19.6, 29.7,100),
				// octomath::Vector3 (19.6, 29.7, 30),
				// octomath::Vector3 (24.4, 16.8, 32),
				// octomath::Vector3 (32.5, 18.5, 32),
				// octomath::Vector3 (51.5, 33.5, 32),
				// octomath::Vector3 (51.5, 33.5, 12),
				// octomath::Vector3 (40.2, 26.8, 12),
				// octomath::Vector3 (40.2, 50, 12),
				// octomath::Vector3 (-30, 50, 12),
				// octomath::Vector3 (0, 50, 12),
				// octomath::Vector3 (0, 30, 25),
				// octomath::Vector3 (0, -10, 25),
				// octomath::Vector3 (0, -10, 10),
				// octomath::Vector3 (0, -30, 10),
				// octomath::Vector3 (-20, -30, 10),
				// octomath::Vector3 (-20, 0, 10)
			};
		int count = points.size()-1;
	    int i;
	    for ( i = 0; i < count; )
	    {
	    	octomath::Vector3 current = *(points.begin());
	    	points.erase(points.begin());
	    	for (std::list<octomath::Vector3>::iterator it = points.begin(); it != points.end(); ++it)
	    	{
	    		testStraightLinesForwardWithObstacles(octree, current, *it, max_time_secs, safety_margin, dataset_name);
	    		// testStraightLinesForwardWithObstacles(octree, *it, current, max_time_secs, safety_margin, dataset_name);
	    	}
	    	++i;
	    }
	}

	TEST(LazyThetaStarMeasurements, AllCombinations)
	{
		std::ofstream csv_file;
		csv_file.open (LazyThetaStarOctree::folder_name + "/current/lazyThetaStar_computation_time.csv", std::ofstream::app);
		csv_file << "success,computation_time_millis,path_lenght_straight_line_meters,path_lenght_total_meters,has_obstacle,start,goal,safety_margin_meters,max_search_duration_seconds,iteration_count,obstacle_hit_count,total_obstacle_checks,dataset_name" << std::endl;
		csv_file.close();

	    octomap::OcTree octree ("data/3Dpuzzle_sparse_complete.bt");
	    double max_time_secs = 1;

	    collectDate(octree, max_time_secs, 5, "3Dpuzzle_ortho_5margin");
	    collectDate(octree, max_time_secs, 5.4, "3Dpuzzle_ortho_5.4margin");
	    collectDate(octree, max_time_secs, 7, "3Dpuzzle_ortho_7margin");
	}

	

}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}