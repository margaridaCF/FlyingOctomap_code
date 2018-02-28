cmake_minimum_required(VERSION 2.8.3)
project(frontiers_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin_simple REQUIRED)

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
# cs_add_library(my_lib src/my_lib.cpp)

 # works just like cs_add_library, but it calls CMake's add_executable(...) instead.
# cs_add_executable(my_exec src/main.cpp)
# target_link_libraries(my_exec my_lib)

# cs_install()
# cs_install_scripts(scripts/my_script.py)

# cs_export()

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_frontiers_msgs.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
