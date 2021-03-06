cmake_minimum_required(VERSION 2.8.3)
project(cmr_localization)

add_compile_options(-std=c++11)

find_package(
  catkin REQUIRED)

catkin_package(
)

include_directories(
)

option(DOWNLOAD_BAG_DATA "Download bag files with data for tests"  OFF)

if(CATKIN_ENABLE_TESTING)

  if(DOWNLOAD_BAG_DATA)
     execute_process(COMMAND rosrun cmr_localization get_bag.sh)
  endif()

  find_package(rostest REQUIRED)

  add_rostest_gtest(test_rtabmap
    test/rtabmap.test
    test/test_rtabmap.cpp
   )

  target_link_libraries(
    test_rtabmap
    ${catkin_LIBRARIES}
   )
   add_dependencies(
     test_rtabmap
    ${catkin_EXPORTED_TARGETS}
  )

endif()
