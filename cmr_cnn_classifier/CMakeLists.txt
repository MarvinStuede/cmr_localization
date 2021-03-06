cmake_minimum_required(VERSION 2.8.3)
project(cmr_cnn_classifier)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  dynamic_reconfigure
)
#generate_dynamic_reconfigure_options(
 # cfg/map_modifier.cfg
#)
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#add_executable(map_modifier
 # src/map_modifier.cpp
 # include/cmr_navigation/map_modifier.h
#)

#add_dependencies(map_modifier ${PROJECT_NAME}_gencfg)

#target_link_libraries(map_modifier
#  ${catkin_LIBRARIES}
#)
