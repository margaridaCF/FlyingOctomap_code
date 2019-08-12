cmake_minimum_required(VERSION 2.8.3)
project(lazy_theta_star)

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


########### 
## Build ##
###########
# cs_add_library(ltStar_lib src/ltStar_lib.cpp)
cs_add_library(ltStar_lib_ortho src/ltStar_lib_ortho.cpp)

cs_add_executable(save_octomap_node src/save_octomap_node.cpp )
# target_link_libraries(save_octomap_node  ${catkin_LIBRARIES} ltStar_lib )

cs_add_executable(ltStar_async_node src/ltStar_async_node.cpp )
target_link_libraries(ltStar_async_node  ${catkin_LIBRARIES} ltStar_lib_ortho )

# cs_add_executable(ltStar_async_node_sparse src/ltStar_async_node_sparse.cpp )
# target_link_libraries(ltStar_async_node_sparse  ${catkin_LIBRARIES} ltStar_lib )

# cs_add_executable(ltStar_command_path_node src/ltStar_command_path_node.cpp )
# target_link_libraries(ltStar_command_path_node  ${catkin_LIBRARIES} ltStar_lib )

cs_install()

cs_export()

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)

  get_filename_component(TEST_WITH_DATA_TEST_DIR "test/data" ABSOLUTE)
  ## Data Collection ##
  # catkin_add_gtest(ltStarOctree-dataCollection 
  #   test/ltStarOctree_dataCollection.cpp
  #   WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  # target_link_libraries(ltStarOctree-dataCollection ${catkin_LIBRARIES} ltStarOctree)

  ## Octree @ analyze data ##
  # catkin_add_gtest(analyzeData 
  #   test/analyze_point_cloud.cpp
  #   WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  # target_link_libraries(analyzeData ${catkin_LIBRARIES} )

  # ## Octree @ collect results ##
  # catkin_add_gtest(collectResults 
  #   test/collect_results.cpp
  #   WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  # target_link_libraries(collectResults ${catkin_LIBRARIES} ltStar_lib)
  # catkin_add_gtest(collectResults_ortho 
  #   test/collect_results_ortho.cpp
  #   WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  # target_link_libraries(collectResults_ortho ${catkin_LIBRARIES} ltStar_lib_ortho)
  
  # catkin_add_gtest(collectResults_obstacleAvoidance_ortho 
  #   test/collect_results_obstacleAvoidance_ortho.cpp
  #   WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  # target_link_libraries(collectResults_obstacleAvoidance_ortho ${catkin_LIBRARIES} ltStar_lib_ortho)

  # catkin_add_gtest(collectResults_obstacleAvoidance
  #   test/collect_results_obstacleAvoidance.cpp
  #   WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  # target_link_libraries(collectResults_obstacleAvoidance ${catkin_LIBRARIES} ltStar_lib)

  # # Octree @ work in progress ##
#   catkin_add_gtest(workInProgress 
#     test/work_in_progress_test.cpp
#     WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
#   target_link_libraries(workInProgress ${catkin_LIBRARIES} ltStar_lib_ortho)

#   # Octree @ visibility ##
#   catkin_add_gtest(visibilityTests 
#     test/visibility_test.cpp
#     WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
#   target_link_libraries(visibilityTests ${catkin_LIBRARIES} ltStar_lib_ortho)

#   ## Lazy Theta Star @ orthogonal planes ##
#   catkin_add_gtest(orthogonalPlanesTests 
#     test/orthogonal_planes_tests.cpp
#     WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
#   target_link_libraries(orthogonalPlanesTests ${catkin_LIBRARIES} ltStar_lib_ortho)

#   ## Octree @ depth and size ##
#   catkin_add_gtest(depthSizeTest 
#     test/depth_voxelSide_tests.cpp
#     WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
#   target_link_libraries(depthSizeTest ${catkin_LIBRARIES} ltStar_lib_ortho)


#   ## Lazy Theta Star @ open ##
#   catkin_add_gtest(openTests 
#     test/open_tests.cpp
#     WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
#   target_link_libraries(openTests ${catkin_LIBRARIES} )

#   catkin_add_gtest(ltstarOrthoTests 
#     test/lazyTheta_tests_ortho.cpp
#     WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
#   target_link_libraries(ltstarOrthoTests ${catkin_LIBRARIES} ltStar_lib_ortho)


  ## Lazy Theta Star @ Collect measurements for 3d puzzle ## 
  catkin_add_gtest(collect_results_3d_puzzle_ortho 
    test/collect_results_3d_puzzle_ortho.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(collect_results_3d_puzzle_ortho ${catkin_LIBRARIES} ltStar_lib_ortho)
#   catkin_add_gtest(collect_results_3d_puzzle_sparse 
#     test/collect_results_3d_puzzle_sparse.cpp
#     WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
#   target_link_libraries(collect_results_3d_puzzle_sparse ${catkin_LIBRARIES} ltStar_lib)

  # ## Lazy Theta Star @ Collect measurements for experimental snapshots ## 
  # catkin_add_gtest(collect_results_experimental_sparse 
  #   test/collect_results_experimental_sparse.cpp
  #   WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  # target_link_libraries(collect_results_experimental_sparse ${catkin_LIBRARIES} ltStar_lib)
  # catkin_add_gtest(collect_results_experimental_ortho
  #   test/collect_results_experimental_ortho.cpp
  #   WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  # target_link_libraries(collect_results_experimental_ortho ${catkin_LIBRARIES} ltStar_lib_ortho)

endif()

