#ifndef EXPLORATION_STATE_MACHINE_H
#define EXPLORATION_STATE_MACHINE_H
#include <ros/ros.h>

namespace exploration_sm
{
    enum exploration_state_t {exploration_start= 1, generating_path = 2, waiting_path_response = 3, visit_waypoints = 4, finished_exploring = 5};
	class ExplorationStateMachine
	{

	public:
		ExplorationStateMachine();
		~ExplorationStateMachine(){}
		exploration_state_t getState();
	    void switchState(exploration_state_t new_state);
		
	private:
    	
    	exploration_state_t current_state;


	};
}

#endif // EXPLORATION_STATE_MACHINE_H