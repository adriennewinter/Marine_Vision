cmake_minimum_required(VERSION 3.0.2)
project(rosbag_manipulations)

find_package(catkin REQUIRED COMPONENTS rosbag roscpp sensor_msgs rospy message_filters std_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${rosbag_INCLUDE_DIRS})

add_executable(split_bag  src/split_bag.cpp)
add_executable(join_bag  src/join_bag.cpp) 
add_executable(add_topics  src/add_topics.cpp) 

target_link_libraries(split_bag  ${catkin_LIBRARIES} ${rosbag_LIBRARIES})
target_link_libraries(join_bag  ${catkin_LIBRARIES} ${rosbag_LIBRARIES})
target_link_libraries(add_topics  ${catkin_LIBRARIES} ${rosbag_LIBRARIES})