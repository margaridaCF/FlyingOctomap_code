cmake_minimum_required(VERSION 2.8.3)
project(gridBenchmark)

add_definitions(-std=c++11 )
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS roscpp octomap_msgs octomap_ros)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gridBenchmark
  CATKIN_DEPENDS roscpp  octomap_msgs octomap_ros 
#  DEPENDS system_lib
)


###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS} )

## C++ library
add_library(gridBenchmark
  src/grid_benchmark.cpp src/qdbmp.cpp
)

target_link_libraries(gridBenchmark
  ${catkin_LIBRARIES}  
)

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)

  get_filename_component(TEST_WITH_DATA_TEST_DIR "test/data" ABSOLUTE)
  ## Data Collection ##
  catkin_add_gtest(gridBenchmark-dataCollection 
    test/grid_benchmark_dataCollection.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(gridBenchmark-dataCollection ${catkin_LIBRARIES} gridBenchmark)

  # # Algorithm Tests ##
  # catkin_add_gtest(algorithmTests 
  #   test/algorithm_tests.cpp
  #   WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  # target_link_libraries(algorithmTests ${catkin_LIBRARIES} gridBenchmark)

  # # Grid Benchmark Tests ##
  # catkin_add_gtest(gridBenchmarkTests 
  #   test/grid_benchmark_tests.cpp
  #   WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  # target_link_libraries(gridBenchmarkTests ${catkin_LIBRARIES} gridBenchmark)
endif()

