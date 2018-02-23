cmake_minimum_required(VERSION 2.8.3)
project(path_planning)

add_definitions(-std=c++11 )
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS roscpp octomap_msgs octomap_ros 
   std_msgs
   message_generation)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  StartGoal.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES thrun
  CATKIN_DEPENDS roscpp  octomap_msgs octomap_ros message_runtime
#  DEPENDS system_lib
)


########### 
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS} )
add_executable(ltStar_dispatcher_node src/ltStar_dispatcher_node.cpp)
target_link_libraries(ltStar_dispatcher_node
  ${catkin_LIBRARIES} 
)

## C++ library
# add_library(ltStarOctree
#   src/ltStarOctree.cpp 
# )

# target_link_libraries(ltStarOctree
#   ${catkin_LIBRARIES}  
# )

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
  # target_link_libraries(collectResults ${catkin_LIBRARIES} )

  # ## Octree @ work in progress ##
  # catkin_add_gtest(workInProgress 
  #   test/work_in_progress_test.cpp
  #   WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  # target_link_libraries(workInProgress ${catkin_LIBRARIES} )

  ## Octree @ depth and size ##depthSizeTest
  catkin_add_gtest(depthSizeTest 
    test/depth_voxelSide_tests.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(depthSizeTest ${catkin_LIBRARIES} )

  ## Octree @ neighbors ##
  catkin_add_gtest(neighborTests 
    test/neighbor_tests.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(neighborTests ${catkin_LIBRARIES} )

  ## Lazy Theta Star @ open ##
  catkin_add_gtest(openTests 
    test/open_tests.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(openTests ${catkin_LIBRARIES} )

  # Lazy Theta Star ##
  catkin_add_gtest(ltstarTests 
    test/lazyTheta_tests.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(ltstarTests ${catkin_LIBRARIES} )
  catkin_add_gtest(ltstarTestsFixture 
    test/lazyTheta_tests_Fixture.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(ltstarTestsFixture ${catkin_LIBRARIES} )

endif()

