#ifndef GOAL_STATE_MACHINE_H
#define GOAL_STATE_MACHINE_H

#include <frontiers_msgs/FindFrontiers.h>
#include <unordered_set>
#include <architecture_math.h>
#include <marker_publishing_utils.h>
#include <observation_maneuver.h>
#include <iostream>
#include <fstream>
#include <octomap/OcTree.h>

#define SAVE_LOG 1

namespace goal_state_machine
{


    struct PairHash
    {
        std::size_t operator()(const std::pair <Eigen::Vector3d, Eigen::Vector3d> & v) const 
        {
            int scale = 0.00001;
            std::size_t hx = std::hash<float>{}( (int)(v.second.x() / scale) * scale );
            std::size_t hy = std::hash<float>{}( (int)(v.second.y() / scale) * scale );
            std::size_t hz = std::hash<float>{}( (int)(v.first.z() / scale) * scale );
            std::size_t return_value = ((hx 
               ^ (hy << 1)) >> 1)
               ^ (hz << 1);
            return return_value;
        }
    };


    struct PairComparatorEqual // big tolerance
    { 
    	double distance(const Eigen::Vector3d  & lhs, const Eigen::Vector3d  &rhs) const
    	{
            double distance = (lhs - rhs).stableNorm();
            if(std::isnan(distance))
			{
				return  0;
			}
			else
			{
				return distance;
			}
    		
    	}

        bool operator () (const std::pair <Eigen::Vector3d, Eigen::Vector3d> & lhs, std::pair <Eigen::Vector3d, Eigen::Vector3d> & rhs) const 
        { 
        	// This large scale allows to skip over calculations for similiar locations
            double scale = 1;
            // ROS_WARN_STREAM("Distance from " << *lhs << " and  " << *rhs << " is " << lhs->distance(*rhs) << " <= " << scale << " returning " << (lhs->distance(*rhs) <= scale)   );

            bool first = distance(lhs.first, rhs.first) <= scale;

            // ROS_INFO_STREAM("First: (" << lhs.first.x() << "," << lhs.first.y() << "," << lhs.first.z() << ")" );
            // ROS_INFO_STREAM("First: (" << rhs.first.x() << "," << rhs.first.y() << "," << rhs.first.z() << ")" );
            if (!first)	return first;
            // returns !0 if the two container object keys passed as arguments are to be considered equal.
            // ROS_INFO_STREAM("Second: (" << lhs.second.x() << "," << lhs.second.y() << "," << lhs.second.z() << ")" );
            // ROS_INFO_STREAM("Second: (" << rhs.second.x() << "," << rhs.second.y() << "," << rhs.second.z() << ")" );

            return distance(lhs.second, rhs.second) <= scale;

        } 
    };
    typedef std::unordered_set<std::pair <Eigen::Vector3d, Eigen::Vector3d>, PairHash, PairComparatorEqual> unobservable_pair_set; 


	class GoalStateMachine
	{
	    rviz_interface::PublishingInput 	pi;
		frontiers_msgs::FindFrontiers 		frontier_srv;
	    geometry_msgs::Point 				geofence_min, geofence_max, success_flyby_start, success_flyby_end;
	    ros::ServiceClient &				find_frontiers_client;
	    bool 								has_more_goals, resetOPPair_flag;
	    bool								is_oppairs_side;
	    bool								new_map;
	    bool								global;
	    bool								first_request;
	    bool								first_global_request;
    	double 								path_safety_margin;
    	double 								sensing_distance;
    	double 								local_fence_side;
    	double 								flyby_length;
		double 								sidelength_lookup_table[];
	    observation_lib::OPPairs 			oppairs_side, oppairs_under;
        unobservable_pair_set	 			unobservable_set; 
	    int 								frontier_index;
        int 								oppair_id;
        int 								frontier_request_count;
        int 								range;
		std::ofstream 						log_file;

		observation_lib::OPPairs& getCurrentOPPairs();
		bool is_flightCorridor_free(double flight_corridor_width) ;
		bool IsOPPairValid() ;
    	bool IsVisible(Eigen::Vector3d unknown);
		bool is_inside_geofence(Eigen::Vector3d target) const;
		bool hasNextFrontier() const;
		void resetOPPair(Eigen::Vector3d& uav_position);
		bool pointToNextGoal(Eigen::Vector3d& uav_position);
		bool IsObservable(Eigen::Vector3d const& viewpoint);
		bool checkFligthCorridor(double flight_corridor_width, Eigen::Vector3d& start, Eigen::Vector3d& end, ros::Publisher const& marker_pub);
		bool fillLocalGeofence();
		void saveSuccesfulFlyby();
		bool findFrontiers_CallService(Eigen::Vector3d& uav_position);
    	bool IsOPStartReachable();

	    
	    
	public:
    	octomap::OcTree* octree;
		geometry_msgs::Point get_current_frontier() ;
		void get_current_frontier(Eigen::Vector3d& frontier) ;
		GoalStateMachine(ros::ServiceClient& find_frontiers_client, double distance_inFront, double distance_behind, int circle_divisions, geometry_msgs::Point& geofence_min, geometry_msgs::Point& geofence_max, rviz_interface::PublishingInput pi, double path_safety_margin, double sensing_distance, int range, double local_fence_side);
		~GoalStateMachine()
		{
			log_file.close();
		}
		bool findFrontiersAllMap(Eigen::Vector3d& uav_position);
		void NewMap();
		bool NextGoal(Eigen::Vector3d& uav_position);
		void DeclareUnobservable();
		bool IsObservable(Eigen::Vector3d const& unobservable, Eigen::Vector3d const& viewpoint);
		void publishGoalToRviz(geometry_msgs::Point current_position);
		void initLookupTable(double resolution, int tree_depth);
		bool isGlobal()
		{
			return global;
		}
		int getUnobservableSetSize()
		{
			return unobservable_set.size();
		}

		bool getFlybyEnd(geometry_msgs::Point & flyby_end)
		{
			if (has_more_goals)
			{
				flyby_end.x = getCurrentOPPairs().get_current_end().x();
				flyby_end.y = getCurrentOPPairs().get_current_end().y();
				flyby_end.z = getCurrentOPPairs().get_current_end().z();
			}
			else
			{
				ROS_ERROR("[goal_state_machine] Asked for 3d flyby end but no goal is available.");
				log_file << "[goal_state_machine] Asked for 3d flyby end but no goal is available." << std::endl;
			}
			return has_more_goals;
		}

		bool getFlybyStart(geometry_msgs::Point & start)
		{
			if (has_more_goals)
			{
				start.x = getCurrentOPPairs().get_current_start().x();
				start.y = getCurrentOPPairs().get_current_start().y();
				start.z = getCurrentOPPairs().get_current_start().z();
			}
			else
			{
				ROS_ERROR("[goal_state_machine] Asked for 3d flyby start but no goal is available.");
				log_file << "[goal_state_machine] Asked for 3d flyby start but no goal is available." << std::endl;
			}
			return has_more_goals;
		}

		bool getFlybyStart(Eigen::Vector3d & flyby_start)
		{
			if (has_more_goals)
			{
				flyby_start.x() = getCurrentOPPairs().get_current_start().x();
				flyby_start.y() = getCurrentOPPairs().get_current_start().y();
				flyby_start.z() = getCurrentOPPairs().get_current_start().z();
			}
			else
			{
				ROS_ERROR("[goal_state_machine] Asked for 3d flyby end but no goal is available.");
				log_file << "[goal_state_machine] Asked for 3d flyby end but no goal is available." << std::endl;
			}
			return has_more_goals;
		}


	};


}

#endif // GOAL_STATE_MACHINE_H