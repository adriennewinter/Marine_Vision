cmake_minimum_required(VERSION 3.0.2)
project(synchronize)

find_package(catkin REQUIRED COMPONENTS rosbag message_filters roscpp rospy sensor_msgs)

catkin_package(INCLUDE_DIRS include)

include_directories(${catkin_INCLUDE_DIRS} ${rosbag_INCLUDE_DIRS} include)

add_executable(${PROJECT_NAME}_all src/synchronize_all.cpp)
add_executable(stereo_synch src/stereo_synch.cpp)
add_executable(stereo_inertial_synch src/stereo_inertial_synch.cpp)

target_link_libraries(${PROJECT_NAME}_all ${catkin_LIBRARIES} ${rosbag_LIBRARIES})
target_link_libraries(stereo_synch ${catkin_LIBRARIES} ${rosbag_LIBRARIES})
target_link_libraries(stereo_inertial_synch ${catkin_LIBRARIES} ${rosbag_LIBRARIES})