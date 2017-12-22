cmake_minimum_required(VERSION 2.8.3)
project(pcl_maker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
  tf
  pcl_conversions
  pcl_ros
)

find_package(OpenCV REQUIRED)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(pc_maker_node src/pc_maker_node.cpp)
target_link_libraries(pc_maker_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

add_executable(two_pc_maker_node src/two_pc_maker_node.cpp)
target_link_libraries(two_pc_maker_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)
