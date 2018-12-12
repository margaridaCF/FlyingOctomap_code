#include <ltStar_lib_ortho.h>
#include <gtest/gtest.h>
#include <queue>


namespace LazyThetaStarOctree{

	
	bool testStraightLinesForwardWithObstacles(octomap::OcTree & octree, octomath::Vector3 disc_initial, octomath::Vector3 disc_final,
		int const& max_time_secs = 55, double safety_margin = 2, std::string dataset_name = "unnamed")
	{

		std::ofstream log_file_;
		log_file_.open (LazyThetaStarOctree::folder_name + "/current/tests.log", std::ofstream::app);
		log_file_ << "[Ortho] Testing from " << disc_initial << " to " << disc_final << "; safety_margin: " << safety_margin << "; max_time_secs: " << max_time_secs << std::endl;
		log_file_.close();

		ros::Publisher marker_pub;
		ResultSet statistical_data;
		double sidelength_lookup_table  [octree.getTreeDepth()];
		PublishingInput publish_input( marker_pub, true, dataset_name);
		InputData input( octree, disc_initial, disc_final, safety_margin);
	   	LazyThetaStarOctree::fillLookupTable( octree.getResolution(), octree.getTreeDepth(), sidelength_lookup_table); 
		generateOffsets(octree.getResolution(), safety_margin, dephtZero, semiSphereOut );
		

		lazy_theta_star_msgs::LTStarRequest request;
        request.request_id = 60;
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
		
		if(0 != ThetaStarNode::OustandingObjects())
		{
			ROS_ERROR_STREAM("Memory leak from ThetaStarNode objects.");
		}
		return true;
	}

	void collectDate(octomap::OcTree & octree, double max_time_secs, double safety_margin, std::string dataset_name, std::list<octomath::Vector3>  points)
	{
		int count = points.size()-1;
	    int i;
	    for ( i = 0; i < count; )
	    {
	    	octomath::Vector3 current = *(points.begin());
	    	points.erase(points.begin());
	    	for (std::list<octomath::Vector3>::iterator it = points.begin(); it != points.end(); ++it)
	    	{
	    		testStraightLinesForwardWithObstacles(octree, current, *it, max_time_secs, safety_margin, dataset_name);
	    		testStraightLinesForwardWithObstacles(octree, *it, current, max_time_secs, safety_margin, dataset_name);
	    	}
	    	++i;
	    }
	}

	void collectData_differentMargins(octomap::OcTree & octree, std::list<octomath::Vector3>  points, std::string dataset_name)
	{

	    double max_time_secs = 1;

	    collectDate(octree, max_time_secs, 3.9, dataset_name+"_ortho", points);
	    collectDate(octree, max_time_secs, 5, dataset_name+"_ortho", points);
	    collectDate(octree, max_time_secs, 5.4, dataset_name+"_ortho", points);
	}


	
	TEST(LazyThetaStarMeasurements, SparseNeighbors_Original_duplicateRealFlights)
	{
		std::ofstream csv_file;
		csv_file.open (LazyThetaStarOctree::folder_name + "/current/lazyThetaStar_computation_time.csv", std::ofstream::app);
		csv_file << "success,computation_time_millis,path_lenght_straight_line_meters,path_lenght_total_meters,has_obstacle,start,goal,safety_margin_meters,max_search_duration_seconds,iteration_count,obstacle_hit_count,total_obstacle_checks,dataset_name" << std::endl;
		csv_file.close();

	    double max_time_secs = 60;

	    
	    std::vector<std::list<octomath::Vector3>> points =  {{
			octomath::Vector3 (6, -6, 7),
			octomath::Vector3 (7.49, -6.98, 7)
			}};
		std::vector<std::string> dataset_names = {"20180821_1110_A"};

		points.insert(points.begin(), {
			octomath::Vector3 (-3.9, -14.76, 7),
			octomath::Vector3 (7.49, -6.98, 7)
			});
		dataset_names.insert(dataset_names.begin(), "20180821_1110_B");
		points.insert(points.begin(), {
			octomath::Vector3 (7.49, -6.98, 7),
			octomath::Vector3 (-3.9, -14.76, 7)
			});
		dataset_names.insert(dataset_names.begin(), "20180821_1110_C");
		points.insert(points.begin(), {
			octomath::Vector3 (-3.9, -14.76, 7),
			octomath::Vector3 (7.49, -6.98, 7)
			});
		dataset_names.insert(dataset_names.begin(), "20180821_1110_D");
		points.insert(points.begin(), {
			octomath::Vector3 (5.65, -9.26, 7),
			octomath::Vector3 (-3.73, -17.2, 7)
			});
		dataset_names.insert(dataset_names.begin(), "20180823_1115_A");
		points.insert(points.begin(), {
			octomath::Vector3 (-6.61, -21, 7),
			octomath::Vector3 (5.65, -9.26, 7)
			});
		dataset_names.insert(dataset_names.begin(), "20180823_1115_B");
		points.insert(points.begin(), {
			octomath::Vector3 (-5.35,-14,7),
			octomath::Vector3 (6.13,-4.07,7)
			});
		dataset_names.insert(dataset_names.begin(), "20180821_1154_A");
		points.insert(points.begin(), {
			octomath::Vector3 (-6.14,-4,7),
			octomath::Vector3 (-5.35,-14,7)
			});
		dataset_names.insert(dataset_names.begin(), "20180821_1154_B");
		points.insert(points.begin(), {
			octomath::Vector3 (5.65, -9.26, 7),
			octomath::Vector3 (-3.73, -17.2, 7)
			});
		dataset_names.insert(dataset_names.begin(), "20180823_1313_A");
		points.insert(points.begin(), {
			octomath::Vector3 (5.65, -9.26, 7),
			octomath::Vector3 (-3.73, -17.2, 7)
			});
		dataset_names.insert(dataset_names.begin(), "20180823_1313_B");
		points.insert(points.begin(), {
			octomath::Vector3 (5.65, -9.26, 7),
			octomath::Vector3 (-3.73, -17.2, 7)
			});
		dataset_names.insert(dataset_names.begin(), "20180823_1313_C");
		points.insert(points.begin(), {
			octomath::Vector3 (6,-6,7.0),
			octomath::Vector3 (-3.41,-16.96, 7)
			});
		dataset_names.insert(dataset_names.begin(), "20180821_1000_A");

		for (int i = 0; i < dataset_names.size(); ++i)
		{
			ROS_ERROR_STREAM(dataset_names[i]);
			octomap::OcTree octree_v1 ("data/"+dataset_names[i]+".bt");
		    collectDate(octree_v1, max_time_secs, 3.9, 	dataset_names[i]+"_ortho", points[i]);
		    collectDate(octree_v1, max_time_secs, 5, 	dataset_names[i]+"_ortho", points[i]);
		    collectDate(octree_v1, max_time_secs, 5.4, 	dataset_names[i]+"_ortho", points[i]);
			
		}
	}
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
