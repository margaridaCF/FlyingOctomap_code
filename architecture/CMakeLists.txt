cmake_minimum_required(VERSION 2.8.3)
project(architecture)

add_definitions(-std=c++11 )
set(CMAKE_BUILD_TYPE Debug)

find_package(Boost REQUIRED)
find_package(catkin_simple REQUIRED)

###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS} )


###########################
## Catkin simple macros  ##
###########################
# find_package(...) is called with the QUIET option and witho the REQUIRED option
# adds the local include folder and any catkin include directories to the include path with CMake's include_directories(...) macro
# discover and build any ROS messages, services, and actions which reside in the msg, srv, action action folders, respectively
# needs build_depend's on message_generation and run_depend on message_runtime
# discovers and builds dynamic_reconfigure config files from the cfg folder
# needs build_depend's on dynamic_reconfigure
catkin_simple(ALL_DEPS_REQUIRED)

# add_library
# target_link_libraries(my_lib ${catkin_LIBRARIES}) to link your new library against any catkin libraries you have build depended on in your package.xml
# does some bookkeeping so that your library target can be implicitly used later
cs_add_library(goal_state_lib src/goal_state_machine.cpp)
# cs_add_targets_to_package(frontiers_msgs)

 # works just like cs_add_library, but it calls CMake's add_executable(...) instead.
cs_add_executable(mav_comms_node src/mav_comms_node.cpp)
target_link_libraries(mav_comms_node)

cs_add_executable(ual_server src/ual_server.cpp)
target_link_libraries(ual_server)

cs_add_executable(state_manager_node src/state_manager_node.cpp)
target_link_libraries(state_manager_node goal_state_lib)

cs_add_executable(current_position_provider_node src/current_position_provider_node.cpp)
target_link_libraries(current_position_provider_node)

cs_add_executable(gps_utm_tf_broadcaster_node src/gps_utm_tf_broadcaster_node.cpp)
target_link_libraries(gps_utm_tf_broadcaster_node )

cs_install()
# cs_install_scripts(scripts/my_script.py)
cs_export()

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_frontiers_msgs.cpp)
if(CATKIN_ENABLE_TESTING)
#   get_filename_component(TEST_WITH_DATA_TEST_DIR "test/data" ABSOLUTE)

  # #WIP
  # catkin_add_gtest(work_in_progress_architecture_tests 
  #   test/work_in_progress_architecture_tests.cpp
  #   WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  # target_link_libraries(work_in_progress_architecture_tests ${catkin_LIBRARIES} )
  #MATH
  catkin_add_gtest(architecture_tests 
    test/architecture_tests.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(architecture_tests ${catkin_LIBRARIES} )
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
