#ifndef GOAL_STATE_MACHINE_H
#define GOAL_STATE_MACHINE_H

#include <frontiers_msgs/FrontierReply.h>
#include <unordered_set>
#include <architecture_math.h>
#include <marker_publishing_utils.h>
#include <observation_maneuver.h>
#include <lazy_theta_star_msgs/CheckFlightCorridor.h>

#define SAVE_LOG 1

namespace goal_state_machine
{

	class GoalStateMachine
	{
	    rviz_interface::PublishingInput 	pi;
	    frontiers_msgs::FrontierReply & 	frontiers_msg;
	    geometry_msgs::Point 				geofence_min, geofence_max;
	    ros::ServiceClient &				check_flightCorridor_client;
	    bool 								has_more_goals, resetOPPair_flag;
	    int 								frontier_index;
    	double 								path_safety_margin;
    	double 								sensing_distance;
	    observation_lib::OPPairs 			oppairs_side, oppairs_under;
	    bool								is_oppairs_side;
        std::unordered_set<octomath::Vector3, architecture_math::Vector3Hash> unobservable_set; 


		observation_lib::OPPairs& getCurrentOPPairs();
		bool is_flightCorridor_free() ;
		bool IsOPPairValid() ;
		bool is_inside_geofence(Eigen::Vector3d target) const;
		bool hasNextFrontier() const;
		void resetOPPair(Eigen::Vector3d& uav_position);
		bool pointToNextGoal(Eigen::Vector3d& uav_position);
	    
	    
	public:
		geometry_msgs::Point get_current_frontier() const;
		GoalStateMachine(frontiers_msgs::FrontierReply & frontiers_msg, double distance_inFront, double distance_behind, int circle_divisions, geometry_msgs::Point& geofence_min, geometry_msgs::Point& geofence_max, rviz_interface::PublishingInput pi, ros::ServiceClient& check_flightCorridor_client, double path_safety_margin, double sensing_distance);
		~GoalStateMachine(){}
		void NewFrontiers(frontiers_msgs::FrontierReply & new_frontiers_msg);
		bool NextGoal(Eigen::Vector3d& uav_position);
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
			}
			return has_more_goals;
		}

		bool get2DFlybyStart(Eigen::Vector2d & start)
		{
			if (has_more_goals)
			{
				start.x() = getCurrentOPPairs().get_current_start().x();
				start.y() = getCurrentOPPairs().get_current_start().y();
			}
			else
			{
				ROS_ERROR("[goal_state_machine] Asked for 2d flyby start but no goal is available.");
			}
			return has_more_goals;
		}

		bool get2DFlybyEnd(Eigen::Vector2d & end)
		{
			if (has_more_goals)
			{
				end.x() = getCurrentOPPairs().get_current_end().x();
				end.y() = getCurrentOPPairs().get_current_end().y();
			}
			else
			{
				ROS_ERROR("[goal_state_machine] Asked for 2d flyby end but no goal is available.");
			}
			return has_more_goals;
		}

	};


}

#endif // GOAL_STATE_MACHINE_H