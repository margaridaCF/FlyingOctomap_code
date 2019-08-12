# FlyingOctomap
An implementation of different algorithms over the octree implementation of octomap.

The code was developed with a test-driven process, that is why so far there are no nodes only tests.
Also, it is a work in progress. The separation of the interface and the implementation is on the roadmap.

Developed by M. Faria, in the context of MarineUAS program, Innovative Training Network in Autonomous Unmanned Aerial Systems for Marine and Costal Monitoring.

# lazy_theta_star package
Generating paths in large, outdoor, 3D scenarios, online is crucial to autonomous vehicles in various environments like air or underwater. Deterministic algorithms can give more guarantees on the action taken in a given situation, a feature highly important in standardization and verification.
The lazy_theta_star package offers the implementation of Lazy Theta \*, extended for 3D and implemented over octomap. This combination addresses all of the mentioned concerns.

A video with path examples can be seen in https://youtu.be/EMfS2lRTAZY

Lazy Theta * is an any-angle variation of A*. Primarily developed for games and solving the problem in 2D, it focuses on reducing the number of expensive obstacles avoidance tests and generating a trajectory usable without postprocessing for smoothing. http://aigamedev.com/open/tutorial/lazy-theta-star/
It has several attributes:
- Deterministic. A chance to unit test and to give guarantees.
- Obstacle detection within corridors, not lines. 
- Unknown space is treated as an obstacle, it will not pass through an unseen obstacle.
- Succinct trajectories. Any-angle means it will give a trajectory with the minimum number of waypoints.  Increased straight lines without post-processing.
Lazy Theta * is implemented over octomap, which also brings features:
- Outdoors. As voxels are grouped into same-state (free/unknown/occupied) larger voxels, larger distances can be searched for a path.
- Online. The grouping of the search space enables it to be conducted online for more cases.
In the talk includes a quick outline of the algorithm, the paths it yields, how to set it up and how to reason if it is an appropriate planner for your scenario.

## Run unit tests:
1. Create data folder inside test folder
2. Move there the datasets
	- (-10.3054; -18.2637; 2.34813)_(-8.5; 6.5; 3.5)_badNodeAdded.bt - http://margaridacf.weebly.com/uploads/4/9/7/5/4975687/run_2.bt
	- (-11.2177; -18.2778; 2.39616)_(-8.5; 6.5; 3.5)_throughWall.bt - http://margaridacf.weebly.com/uploads/4/9/7/5/4975687/_-11.2177__-18.2778__2.39616___-8.5__6.5__3.5__throughwall.bt
	- run_2.bt - http://margaridacf.weebly.com/uploads/4/9/7/5/4975687/run_2.bt
	- offShoreOil_1m.bt https://github.com/margaridaCF/FlyingOctomap/blob/master/lazy_theta_star/test/data/offShoreOil_1m.bt

## How to use ltStar_async_node:

- The ltStar_async_node.cpp encapsulates the calling of the API.
- lazy_theta_star package contains the source code. The lazy_theta_star_msgs contains the messages and services used with the ltStar_asyncply node.
- This node expects an LTStarRequest message and replies with an LTStarReply message. This asynchronous mode of communication frees the calling node for other processing. The LTStarNodeStatus service replies successfully when the node is ready to receive requested (the octomap messages are being received).
- Request message:       
int16 request_id - identifier to later coordinate with the reply       
geometry_msgs/Point start - coordinates of the starting point (local in octomap frame)       
geometry_msgs/Point goal - coordinates of the goal (local in octomap frame)       
int16 max_search_iterations - maximum amount of seconds spent in the search (if time breaks out the reply message has false in the success variable)       
float32 safety_margin - width and height of the corridor that is verified to be free around the path.      

- Reply message:       
bool success - true if a path was found, false otherwise      
uint32 request_id - the same id as the request message that it is replying to       
uint32 waypoint_amount - total number of waypoints in the path (if successful)       
geometry_msgs/Point[] waypoints - the coordinates (in local octomap frame) of each waypoint      

This package was developed using px4's sitl inside a docker container. The instructions can be found on _generate_plots packages, jupyter notebook Paper B - SITL. In try 9.

The funtion that implements Lazy Theta Star is lazyThetaStar_, inside lazy_theta_star/ltStar_lib.cpp

The package has been introduced at Roscon 2018! 

[![RosCon 2018 presentation video](https://img.youtube.com/vi/UbR8OUqfwe0/0.jpg)](https://www.youtube.com/watch?v=UbR8OUqfwe0)

## Integrate kinematic constraints

One question that popped up a lot was how to integrate kinematic constraints directly at trajectory generation level. For the benefit of everyone, here is the answer.

The original use case has holonomic vehicles in mind (rotary wing UAVs). However, it is possible to modify the implementation to include any constraints that need to be considered to evaluate a position as a candidate for the path.

When neighbors are generated, the obstacle avoidance is the check done currently. The method could be extended to include any constraint,  kinematics being on example. Another place to look is the generation of neighbors.
The places in the code to look into are at ltStar_lib.cpp (for the main branch): 
- generate neighbors method at line 743 that refers to neighbors.cpp line 18. The neighbors considered are up, down, left, right (it is over a sparse occupancy grid - octomap) 
- the line of sight checks at line 310 currently look for obstacles

If you need this in your work just let me know to start working on it.

## Corresponding paper

There is a paper that gives more technical details on the implementation and considerations. If you find this code helpful please cite the following paper:
> @article{Faria2018,
> doi = {10.1007/s10846-018-0798-4},
> issn = {1573-0409},
> journal = {Journal of Intelligent {\&} Robotic Systems},
> month = {mar},
> title = {{Applying Frontier Cells Based Exploration and Lazy Theta* Path Planning over Single Grid-Based World Representation for Autonomous Inspection of Large 3D Structures with an UAS}},
> year = {2018}
> }

You can download the paper [here]( https://www.researchgate.net/publication/323994301_Applying_Frontier_Cells_Based_Exploration_and_Lazy_Theta_Path_Planning_over_Single_Grid-Based_World_Representation_for_Autonomous_Inspection_of_Large_3D_Structures_with_an_UAS)



# Acknowlegements

This project has received funding from the European Union’s Horizon 2020 research and innovation programme under the Marie Sklodowska-Curie grant agreement No 642153

This code reflects only the author's view and that the Agency is not responsible for any use that may be made of the information it contains.

This project has received the support of the Fundación Andaluza para el Desarrollo Aeroespacial (FADA) on the Center for Advanced Aerospace Technologies (CATEC) and of the Robotics, Vision and Control Group (GRVC) of the Universidad de Sevilla.

Done with the support of Antidio Viguria at the Centro Avanzado de Tecnologas Aeroespaciales, Sevilla, Spain. And Ivan Maza at the Robotics, Vision and Control Group, University of Seville, Sevilla, Spain
