cmake_minimum_required(VERSION 2.8.3)
project(frontiers)

add_definitions(-std=c++11 )
set(CMAKE_BUILD_TYPE Debug)

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
cs_add_library(neighbors src/neighbors.cpp)
cs_add_library(frontiers_lib src/frontiers.cpp)
cs_add_library(next_best_view_lib src/next_best_view.cpp )
# cs_add_targets_to_package(frontiers_msgs)

 # works just like cs_add_library, but it calls CMake's add_executable(...) instead.
cs_add_executable(next_best_view_node src/neighbors.cpp src/frontiers.cpp src/next_best_view_node.cpp)
target_link_libraries(next_best_view_node next_best_view_lib )

cs_add_executable(frontiers_async_node src/frontiers_async_node.cpp)
target_link_libraries(frontiers_async_node frontiers_lib neighbors)

cs_add_executable(frontiers_debug_node src/frontiers_debug_node.cpp)
target_link_libraries(frontiers_debug_node frontiers_lib neighbors)

cs_install()
# cs_install_scripts(scripts/my_script.py)

cs_export()

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_frontiers_msgs.cpp)
if(CATKIN_ENABLE_TESTING)
  get_filename_component(TEST_WITH_DATA_TEST_DIR "test/data" ABSOLUTE)

  # Frontiers @ work in progress ##
  catkin_add_gtest(workInProgress_frontiers 
    test/work_in_progress_frontiers_tests.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(workInProgress_frontiers ${catkin_LIBRARIES} next_best_view_lib frontiers_lib neighbors  )

  # WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  # Octree @ neighbors ##
  # catkin_add_gtest(neighborTests 
  #   test/neighbor_tests.cpp
  #   WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  # target_link_libraries(neighborTests ${catkin_LIBRARIES} frontiers_lib neighbors)
  # catkin_add_gtest(frontiers_tests 
  #   test/frontier_lib_tests.cpp
  #   WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  # target_link_libraries(frontiers_tests ${catkin_LIBRARIES} frontiers_lib neighbors)
  # catkin_add_gtest(next_best_view_tests 
  #   test/next_best_view_tests.cpp
  #   WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  # target_link_libraries(next_best_view_tests ${catkin_LIBRARIES} next_best_view_lib)

endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
