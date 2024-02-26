cmake_minimum_required(VERSION 3.0.2)
project(bag_manipulations)

find_package(catkin REQUIRED rosbag roscpp sensor_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${rosbag_INCLUDE_DIRS})

add_executable(split_bag  src/split_bag.cpp) #add for each node in your package
target_link_libraries(split_bag  ${catkin_LIBRARIES} ${rosbag_LIBRARIES})

add_executable(join_bag  src/join_bag.cpp) #add for each node in your package
target_link_libraries(join_bag  ${catkin_LIBRARIES} ${rosbag_LIBRARIES})