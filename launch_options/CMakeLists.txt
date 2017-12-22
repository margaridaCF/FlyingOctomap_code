
cmake_minimum_required (VERSION 2.6)
project(launch_options)

find_package(catkin REQUIRED COMPONENTS geometry_msgs rosconsole roscpp roslib rostime  )

include_directories(include ${catkin_INCLUDE_DIRS} )
link_directories(${catkin_LIBRARY_DIRS})


catkin_package(CATKIN_DEPENDS geometry_msgs )

add_executable(turtle_teleop_key src/teleop_pandora.cpp)
target_link_libraries(turtle_teleop_key ${catkin_LIBRARIES})
add_dependencies(turtle_teleop_key turtlesim_gencpp)





