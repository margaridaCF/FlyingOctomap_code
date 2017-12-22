cmake_minimum_required(VERSION 2.8.3)
project(thrun)

add_definitions(-std=c++11 )
set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
#find_package(octomap REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs  octomap_msgs octomap_ros)


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES thrun
  CATKIN_DEPENDS roscpp   octomap_msgs octomap_ros #sensor_msgs
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} )

## Declare a C++ library
add_library(point2d
  src/point2d_lib.cpp
)
add_library(mapper2d
  src/mapper2d.cpp
)

add_library(frontierCellsSandbox
  src/frontier_cells_sandbox.cpp src/qdbmp.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(thrun ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(mapper_node src/nodes/mapper_node.cpp src/mapper2d.cpp src/point2d_lib.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(thrun_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(point2d
  ${catkin_LIBRARIES}
)
target_link_libraries(mapper2d
  ${catkin_LIBRARIES}
)

target_link_libraries(mapper_node
  ${catkin_LIBRARIES}
)

target_link_libraries(frontierCellsSandbox
  ${catkin_LIBRARIES} point2d 
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(mapper2D-test 
  	test/mapper2d_test.cpp
  	WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(mapper2D-test ${catkin_LIBRARIES} mapper2d point2d)

  get_filename_component(TEST_WITH_DATA_TEST_DIR "test/data" ABSOLUTE)
  catkin_add_gtest(mapper2D-integration-test 
    test/mapper2d_octree_integration.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(mapper2D-integration-test ${catkin_LIBRARIES} mapper2d point2d)

  catkin_add_gtest(mapper2D-integration-oneObstacle-test 
    test/mapper2d_octree_integration_oneObstacle.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(mapper2D-integration-oneObstacle-test ${catkin_LIBRARIES} mapper2d point2d)

  catkin_add_gtest(point2D-test 
    test/point2d_test.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(point2D-test ${catkin_LIBRARIES} point2d)

  catkin_add_gtest(frontier-test 
    test/frontier_cells_sandbox_tests.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(frontier-test ${catkin_LIBRARIES}   point2d frontierCellsSandbox)
endif()

