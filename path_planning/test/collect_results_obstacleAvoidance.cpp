#include <gtest/gtest.h>
#include <ltStar_temp.h>

namespace LazyThetaStarOctree
{	
    bool equal (const geometry_msgs::Point & a, const geometry_msgs::Point & b, 
		const double theta = 0.00000000000000000001)
 	{
 
		bool is_x_equal = std::abs(a.x - b.x) < theta;
		bool is_y_equal = std::abs(a.y - b.y) < theta;
		bool is_z_equal = std::abs(a.z - b.z) < theta;
 
		return is_x_equal && is_y_equal && is_z_equal;
 	}

 	void runAndWriteCSV(path_planning_msgs::LTStarRequest request, octomap::OcTree & octree, PublishingInput publish_input, std::string distance_label, double distance_meters, std::string csv_file_name)
 	{
 		std::chrono::system_clock::time_point start, end;
 		std::chrono::duration<double> time_span;
		std::chrono::milliseconds millis;
		bool is_start_clear;
		double number_of_trials = 100;

	 	start = std::chrono::system_clock::now();
		for (int i = 0; i < number_of_trials; ++i)
		{
			is_start_clear = LazyThetaStarOctree::is_flight_corridor_free(LazyThetaStarOctree::InputData(octree, octomath::Vector3(request.start.x, request.start.y, request.start.z), octomath::Vector3(request.goal.x, request.goal.y, request.goal.z), request.safety_margin), publish_input);
		}
		end = std::chrono::system_clock::now();
		time_span = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
		millis = std::chrono::duration_cast<std::chrono::milliseconds>(time_span);

		std::ofstream csv_file;
		csv_file.open (folder_name+csv_file_name, std::ofstream::app);
		// csv_file << "computation_time_millis,distance_label,path_lenght_total_meters,margin_label,margin,start,goal,dataset_name" << std::endl;
		csv_file <<  std::setprecision(2) << millis.count()/number_of_trials;
		csv_file << "," << distance_label;
		csv_file << "," << distance_meters;
		csv_file << "," << "small";
		csv_file << "," << request.safety_margin;
		csv_file << ",(" <<  std::setprecision(2) << request.start.x << "_"  << request.start.y << "_"  << request.start.z << ")";
		csv_file << ",(" <<  std::setprecision(2) << request.goal.x << "_"  << request.goal.y << "_"  << request.goal.z << ")";
		csv_file << "," << publish_input.dataset_name << std::endl;
		csv_file.close();
 	}

    void testResults(path_planning_msgs::LTStarRequest request, octomap::OcTree & octree, std::string dataset_name)
    {
		ros::Publisher marker_pub;
    	std::string distance_label;
    	octomath::Vector3 disc_initial (request.start.x, request.start.y, request.start.z);
    	octomath::Vector3 disc_final (request.goal.x, request.goal.y, request.goal.z);
    	PublishingInput publish_input (marker_pub, false, dataset_name);

    	double distance_meters = weightedDistance(disc_initial, disc_final);
    	if(distance_meters <= 10)
    	{
    		distance_label = "close";
    	}
    	else
    	{
    		distance_label = "far";
    	}


		double sidelength_lookup_table  [octree.getTreeDepth()];
	   	LazyThetaStarOctree::fillLookupTable(octree.getResolution(), octree.getTreeDepth(), sidelength_lookup_table); 


		request.safety_margin = 2;
		runAndWriteCSV(request, octree, publish_input, distance_label, distance_meters, "/lazyThetaStar_obstacle_avoidance_computation_time_margin_2.csv");
		request.safety_margin = 5;
		runAndWriteCSV(request, octree, publish_input, distance_label, distance_meters, "/lazyThetaStar_obstacle_avoidance_computation_time_margin_5.csv");
		request.safety_margin = 8;
		runAndWriteCSV(request, octree, publish_input, distance_label, distance_meters, "/lazyThetaStar_obstacle_avoidance_computation_time_margin_8.csv");

    }

	TEST(LazyThetaStarTests, LazyThetaStar_20180821_1207_5647_filtered_all)
	{
		octomap::OcTree octree ("data/20180821_1207_5647_filtered.bt");
		path_planning_msgs::LTStarRequest request;
		request.header.seq = 2;
		request.request_id = 3;
		request.start.x = 6.22;
		request.start.y = -7.23;
		request.start.z = 7;
		request.goal.x = 0.69;
		request.goal.y = -11.2;
		request.goal.z = 7;
		request.max_search_iterations = 120;
		request.safety_margin = 5;
		testResults(request, octree, "20180821_1207_5647_filtered_ortho" );

		request.start.x = 11.3;
		request.start.y = -6.71;
		request.start.z = 7;
		request.goal.x = -8.34;
		request.goal.y = -10.9;
		request.goal.z = 7;
		testResults(request, octree, "20180821_1207_5647_filtered_ortho" );

		request.start.x = -5.35;
		request.start.y = -14;
		request.start.z = 7;
		request.goal.x = 4.12;
		request.goal.y = -2.21;
		request.goal.z = 7;
		testResults(request, octree, "20180821_1207_5647_filtered_ortho" );


		request.start.x = -3.9;
		request.start.y = -14.76;
		request.start.z = 7;
		request.goal.x = 7.49;
		request.goal.y = -6.98;
		request.goal.z = 7;
		testResults(request, octree, "20180821_1207_5647_filtered_ortho" );

		request.start.x =  16.8;
		request.start.y = -4.22;
		request.start.z = 5;
		request.goal.x = -23.5;
		request.goal.y = -28.4;
		request.goal.z = 3;
		testResults(request, octree, "20180821_1207_5647_filtered_ortho" );

		request.start.x =  -15.7;
		request.start.y = -5.08;
		request.start.z = 5;
		request.goal.x = 13.9;
		request.goal.y = -18.2;
		request.goal.z = 3;
		testResults(request, octree, "20180821_1207_5647_filtered_ortho" );
	}

	TEST(LazyThetaStarTests, LazyThetaStar_20180821_1110_43042_raw_all)
	{
		octomap::OcTree octree ("data/20180821_1110_43042_raw.bt");
		path_planning_msgs::LTStarRequest request;
		request.header.seq = 2;
		request.request_id = 3;
		request.start.x = 6.22;
		request.start.y = -7.23;
		request.start.z = 7;
		request.goal.x = 0.69;
		request.goal.y = -11.2;
		request.goal.z = 7;
		request.max_search_iterations = 120;
		request.safety_margin = 5;
		testResults(request, octree, "20180821_1110_43042_raw_ortho" );

		request.start.x = 11.3;
		request.start.y = -6.71;
		request.start.z = 7;
		request.goal.x = -8.34;
		request.goal.y = -10.9;
		request.goal.z = 7;
		testResults(request, octree, "20180821_1110_43042_raw_ortho" );

		request.start.x = -5.35;
		request.start.y = -14;
		request.start.z = 7;
		request.goal.x = 4.12;
		request.goal.y = -2.21;
		request.goal.z = 7;
		testResults(request, octree, "20180821_1110_43042_raw_ortho" );


		request.start.x = -3.9;
		request.start.y = -14.76;
		request.start.z = 7;
		request.goal.x = 7.49;
		request.goal.y = -6.98;
		request.goal.z = 7;
		testResults(request, octree, "20180821_1110_43042_raw_ortho" );

		request.start.x =  16.8;
		request.start.y = -4.22;
		request.start.z = 5;
		request.goal.x = -23.5;
		request.goal.y = -28.4;
		request.goal.z = 3;
		testResults(request, octree, "20180821_1110_43042_raw_ortho" );

		request.start.x =  -15.7;
		request.start.y = -5.08;
		request.start.z = 5;
		request.goal.x = 13.9;
		request.goal.y = -18.2;
		request.goal.z = 3;
		testResults(request, octree, "20180821_1110_43042_raw_ortho" );
	}

	TEST(LazyThetaStarTests, LazyThetaStar_20180821_1110_42936_raw_all)
	{
		octomap::OcTree octree ("data/20180821_1110_42936_raw.bt");
		path_planning_msgs::LTStarRequest request;
		request.header.seq = 2;
		request.request_id = 3;
		request.start.x = 6.22;
		request.start.y = -7.23;
		request.start.z = 7;
		request.goal.x = 0.69;
		request.goal.y = -11.2;
		request.goal.z = 7;
		request.max_search_iterations = 120;
		request.safety_margin = 5;
		testResults(request, octree, "20180821_1110_42936_raw_ortho" );

		request.start.x = 11.3;
		request.start.y = -6.71;
		request.start.z = 7;
		request.goal.x = -8.34;
		request.goal.y = -10.9;
		request.goal.z = 7;
		testResults(request, octree, "20180821_1110_42936_raw_ortho" );

		request.start.x = -5.35;
		request.start.y = -14;
		request.start.z = 7;
		request.goal.x = 4.12;
		request.goal.y = -2.21;
		request.goal.z = 7;
		testResults(request, octree, "20180821_1110_42936_raw_ortho" );


		request.start.x = -3.9;
		request.start.y = -14.76;
		request.start.z = 7;
		request.goal.x = 7.49;
		request.goal.y = -6.98;
		request.goal.z = 7;
		testResults(request, octree, "20180821_1110_42936_raw_ortho" );

		request.start.x =  16.8;
		request.start.y = -4.22;
		request.start.z = 5;
		request.goal.x = -23.5;
		request.goal.y = -28.4;
		request.goal.z = 3;
		testResults(request, octree, "20180821_1110_42936_raw_ortho" );

		request.start.x =  -15.7;
		request.start.y = -5.08;
		request.start.z = 5;
		request.goal.x = 13.9;
		request.goal.y = -18.2;
		request.goal.z = 3;
		testResults(request, octree, "20180821_1110_42936_raw_ortho" );
	}


	TEST(LazyThetaStarTests, LazyThetaStar_20180821_1110_42712_raw_all)
	{
		octomap::OcTree octree ("data/20180821_1110_42712_raw.bt");
		path_planning_msgs::LTStarRequest request;
		request.header.seq = 2;
		request.request_id = 3;
		request.start.x = 6.22;
		request.start.y = -7.23;
		request.start.z = 7;
		request.goal.x = 0.69;
		request.goal.y = -11.2;
		request.goal.z = 7;
		request.max_search_iterations = 120;
		request.safety_margin = 5;
		testResults(request, octree, "20180821_1110_42712_raw_ortho" );

		request.start.x = 11.3;
		request.start.y = -6.71;
		request.start.z = 7;
		request.goal.x = -8.34;
		request.goal.y = -10.9;
		request.goal.z = 7;
		testResults(request, octree, "20180821_1110_42712_raw_ortho" );

		request.start.x = -5.35;
		request.start.y = -14;
		request.start.z = 7;
		request.goal.x = 4.12;
		request.goal.y = -2.21;
		request.goal.z = 7;
		testResults(request, octree, "20180821_1110_42712_raw_ortho" );


		request.start.x = -3.9;
		request.start.y = -14.76;
		request.start.z = 7;
		request.goal.x = 7.49;
		request.goal.y = -6.98;
		request.goal.z = 7;
		testResults(request, octree, "20180821_1110_42712_raw_ortho" );

		request.start.x =  16.8;
		request.start.y = -4.22;
		request.start.z = 5;
		request.goal.x = -23.5;
		request.goal.y = -28.4;
		request.goal.z = 3;
		testResults(request, octree, "20180821_1110_42712_raw_ortho" );

		request.start.x =  -15.7;
		request.start.y = -5.08;
		request.start.z = 5;
		request.goal.x = 13.9;
		request.goal.y = -18.2;
		request.goal.z = 3;
		testResults(request, octree, "20180821_1110_42712_raw_ortho" );
	}
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}