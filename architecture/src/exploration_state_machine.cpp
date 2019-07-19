#include <exploration_state_machine.h>

namespace exploration_sm
{
	ExplorationStateMachine::ExplorationStateMachine()
	{
		switchState(visit_waypoints);
	}

	exploration_state_t ExplorationStateMachine::getState()
	{
		return current_state;
	}

	void ExplorationStateMachine::switchState(exploration_state_t new_state)
	{
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
}