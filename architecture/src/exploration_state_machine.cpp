#include <exploration_state_machine.h>
#define SAVE_CSV 1

namespace exploration_sm
{
	ExplorationStateMachine::ExplorationStateMachine()
		:csv_open(false)
	{
		switchState(visit_waypoints, false);
	}

	exploration_state_t ExplorationStateMachine::getState()
	{
		return current_state;
	}

	void ExplorationStateMachine::openCSV()
	{
		if(csv_open) return;
		csv_open = true;
	    std::stringstream aux_envvar_home (std::getenv("HOME"));
	    std::string folder_name = aux_envvar_home.str() + "/Flying_Octomap_code/src/data/current";
	    ROS_INFO_STREAM("[State Manager] csv at " << folder_name << "/state_machine_execution_times.csv");
    	csv_file.open (folder_name+"/state_machine_execution_times.csv", std::ofstream::app);
    	csv_file << "timeline,visit_waypoints_millis,global_exploration_millis,local_exploration_millis,ltstar_millis" << std::endl;
	    operation_start = std::chrono::high_resolution_clock::now();
	    timeline_start = std::chrono::high_resolution_clock::now();
	}
	
	void ExplorationStateMachine::calculateAndSaveCsv(bool global)
	{
		std::pair <double, double> millis_count = calculateTime(); 
		switch(current_state)
		{
			case visit_waypoints:
                csv_file << millis_count.first <<  ","<<millis_count.second<<",,," << std::endl;
				break;

			case exploration_start:
				if(global)
				{
                	csv_file << millis_count.first << ",,"<<millis_count.second<<",," << std::endl;
				}
				else
				{
					csv_file << millis_count.first << ",,,"<<millis_count.second<<"," << std::endl;
				}
				break;

			case waiting_path_response:
                csv_file << millis_count.first <<  ",,,,"<<millis_count.second << std::endl;
				break;

			case finished_exploring:
				csv_file.close();
				break;
		}
        operation_start = std::chrono::high_resolution_clock::now();
	}

	void ExplorationStateMachine::switchState(exploration_state_t new_state, bool global)
	{
		#ifdef SAVE_CSV	
        calculateAndSaveCsv(global);
	    #endif
		current_state = new_state;
		switch(current_state)
		{
			case visit_waypoints:
				ROS_WARN("[Exploration SM] visit_waypoints");
				break;
			case exploration_start:
				ROS_WARN("[Exploration SM] exploration_start");
				break;
			case generating_path:
				ROS_WARN("[Exploration SM] generating_path");
				break;
			case waiting_path_response:
				ROS_WARN("[Exploration SM] waiting_path_response");
				break;
			case finished_exploring:
				ROS_WARN("[Exploration SM] finished_exploring");
				break;
			default:
				ROS_ERROR_STREAM("[Exploration SM] Something went very wrong. Exploration state is " << current_state);
		}
	}

    std::pair<double, double> ExplorationStateMachine::calculateTime()
    {
        auto end_millis         = std::chrono::high_resolution_clock::now();
        
        auto time_span          = std::chrono::duration_cast<std::chrono::duration<double>>(end_millis - operation_start);
        double operation_millis = std::chrono::duration_cast<std::chrono::milliseconds>(time_span).count();

        time_span               = std::chrono::duration_cast<std::chrono::duration<double>>(end_millis - timeline_start);
        double timeline_millis  = std::chrono::duration_cast<std::chrono::milliseconds>(time_span).count();
        operation_start         = std::chrono::high_resolution_clock::now();
        return std::make_pair (timeline_millis, operation_millis);
    }
}



            