#ifndef EXPLORATION_STATE_MACHINE_H
#define EXPLORATION_STATE_MACHINE_H
#include <ros/ros.h>

#include <sstream>
#include <fstream>
#include <string>
#include <chrono>
#define SAVE_CSV 1

namespace exploration_sm
{
    enum exploration_state_t {exploration_start= 1, generating_path = 2, waiting_path_response = 3, visit_waypoints = 4, finished_exploring = 5};
	class ExplorationStateMachine
	{
	public:
		ExplorationStateMachine();
		~ExplorationStateMachine()
		{
			csv_file.close();
		}
		exploration_state_t getState();
	    void switchState(exploration_state_t new_state,bool global);
		void openCSV();
		
	private:
    	
    	exploration_state_t current_state;
		void calculateAndSaveCsv(bool global);

        #ifdef SAVE_CSV
        	bool csv_open;
    		std::pair<double, double> calculateTime();
		    std::ofstream csv_file, csv_file_success;
		    std::chrono::high_resolution_clock::time_point operation_start, timeline_start;
	        std::chrono::high_resolution_clock::time_point end_millis;
	        std::chrono::duration<double> time_span;
	        std::chrono::milliseconds millis;
	        double visit_waypoints_millis, goalSM_millis, ltstar_millis;
        #endif

	};
}

#endif // EXPLORATION_STATE_MACHINE_H