cmake_minimum_required(VERSION 3.0.2)
project(synchronize)

find_package(catkin REQUIRED COMPONENTS rosbag message_filters roscpp rosconsole sensor_msgs)

catkin_package(INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS} ${rosbag_INCLUDE_DIRS})

add_library(synchronize_lib SHARED
                src/headers/synchronize.cpp
                src/headers/synchronize_helper.cpp
                include/synchronize/synchronize.h
                include/synchronize/synchronize_helper.h
                include/synchronize/public_simple_filter.h)

add_executable(synchronize_all src/synchronize_all.cpp)
add_executable(stereo_synch src/stereo_synch.cpp)
add_executable(stereo_inertial_synch src/stereo_inertial_synch.cpp)

target_link_libraries(synchronize_all synchronize_lib ${catkin_LIBRARIES} ${rosbag_LIBRARIES})
target_link_libraries(stereo_synch synchronize_lib ${catkin_LIBRARIES} ${rosbag_LIBRARIES})
target_link_libraries(stereo_inertial_synch synchronize_lib ${catkin_LIBRARIES} ${rosbag_LIBRARIES})