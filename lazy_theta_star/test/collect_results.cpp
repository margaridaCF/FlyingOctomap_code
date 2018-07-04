#include <gtest/gtest.h>
#include <ltStar_temp.h>

namespace LazyThetaStarOctree
{
	timespec diff(timespec start, timespec end)
    {
        timespec temp;
        if ((end.tv_nsec-start.tv_nsec)<0) {
            temp.tv_sec = end.tv_sec-start.tv_sec-1;
            // temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
        } else {
            temp.tv_sec = end.tv_sec-start.tv_sec;
            temp.tv_nsec = end.tv_nsec-start.tv_nsec;
        }
        return temp;
    }

    void extractResults (octomap::OcTree octree, octomath::Vector3 disc_initial, octomath::Vector3 disc_final, std::string dataset_name, int max_search_iterations = 500)
    {
    	octomath::Vector3 direction =  disc_final-disc_initial;
		octomath::Vector3 return_value;

		bool occupied_cell_was_hit = octree.castRay(disc_final, direction, return_value, false, direction.norm());
		if(!occupied_cell_was_hit)
		{
			octomap::OcTreeNode* search_result = octree.search(return_value);
			if (search_result == NULL)
			{
				ASSERT_TRUE (false) << "Goal is in unknown space";
			}
			else
			{
				ROS_WARN_STREAM("This is known obstacle space. ");
			}
		}
		else
		{
			ROS_WARN_STREAM("This is free space");
		}

		
		ResultSet statistical_data;
		timespec time1, time2;
    	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);
		std::list<octomath::Vector3> resulting_path = lazyThetaStar_(octree, disc_initial, disc_final, statistical_data, max_search_iterations);
		clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
    	double total_nSecs_overall = diff(time1,time2).tv_nsec;
    	std::ofstream waypoints_file;
    	waypoints_file.open("/home/mfaria/Margarida/20170802_lazyThetaStar/experimental data/euroc_compare/newImplementation.log", std::ios_base::app);
    	waypoints_file << " ===== " << dataset_name << " ===== " << std::endl;
		waypoints_file << " iterations used: " << statistical_data.iterations_used  << "; Took " << total_nSecs_overall << " nano seconds." << std::endl;
		for (octomath::Vector3 waypoint : resulting_path)
		{
			waypoints_file << waypoint << std::endl;
		}
		waypoints_file << std::endl;
		ASSERT_GT(resulting_path.size(), 0);
    }

// 1490700010.397750750   Local:  Calculating path from (0, 0, 0) to 2.6, -2.3, 0.4)  iterations used: 27; Took 1565925 nano seconds.
	/*TEST(LazyThetaStarTests, run1_1490700010)
	{
		std::string dataset_name = "Euroc Run 1 @ 1490700010";
		octomap::OcTree octree ("data/run_1_1490700010.bt");
		octomath::Vector3 disc_initial(0, 0, 0);
		octomath::Vector3 disc_final  (2.6, -2.3, 0.4);
		int max_search_iterations = 27;
		extractResults(octree, disc_initial, disc_final, dataset_name);
	}*/

// 1490700011.624896175   Global: Calculating path from (2.2, 4.8, 1.2) to -2.5, -0.5, 2.2)  iterations used: 43270; Took 4 seconds.
	/*TEST(LazyThetaStarTests, run1_1490700011)
	{
		std::string dataset_name = "Euroc Run 1 @ 1490700011";
		octomap::OcTree octree ("data/run_1_1490700011.bt");
		octomath::Vector3 disc_initial( 2.2, 4.8, 1.2);
		octomath::Vector3 disc_final  (-2.5, -0.5, 2.2);
		int max_search_iterations = 43270;
		extractResults(octree, disc_initial, disc_final, dataset_name, max_search_iterations);
	}*/


// 1490700086.444189348   Global: Calculating path from (-2.6, -3.9, 3.2) to 2.2, 4.9, 1.5)  iterations used: 54; Took 42690669 nano seconds.
	/*TEST(LazyThetaStarTests, run1_1490700086)
	{
		std::string dataset_name = "Euroc Run 1 @ 1490700086";
		octomap::OcTree octree ("data/run_1_1490700086.bt");
		octomath::Vector3 disc_initial(-2.6, -3.9, 3.2);
		octomath::Vector3 disc_final  ( 2.2, 4.9, 1.5);
		int max_search_iterations = 1000;
		extractResults(octree, disc_initial, disc_final, dataset_name, max_search_iterations);
	}*/

// (-2.75488459312, -3.89351167009, 3.24224048416) to (2.20000004768, 4.90000009537, 1.5)
	TEST(LazyThetaStarTests, run1_1490700080)
	{
		std::string dataset_name = "Euroc Run 2 @ final map";
		octomap::OcTree octree ("data/run_2.bt");
		octomath::Vector3 disc_initial(2, 6, 1.5);
		octomath::Vector3 disc_final  (-1, 2.5, 1.5);
		int max_search_iterations = 1000;
		extractResults(octree, disc_initial, disc_final, dataset_name, max_search_iterations);
	}
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}