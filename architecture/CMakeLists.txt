cmake_minimum_required(VERSION 2.8.3)
project(architecture)

add_definitions(-std=c++11 )
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS roscpp )
include_directories(include ${catkin_INCLUDE_DIRS} )

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES thrun
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

########### 
## Build ##
###########
add_executable(px4_mavros_offboard_node src/px4_mavros_offboard_node.cpp)
target_link_libraries(px4_mavros_offboard_node
  ${catkin_LIBRARIES} 
)

# #############
# ## Testing ##
# #############
# if (CATKIN_ENABLE_TESTING)
#   get_filename_component(TEST_WITH_DATA_TEST_DIR "test/data" ABSOLUTE)

  # ## Architecture @ name ##
  # catkin_add_gtest(name_Tests 
  #   test/name_tests.cpp
  #   WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  # target_link_libraries(name_Tests ${catkin_LIBRARIES} )

# endif()