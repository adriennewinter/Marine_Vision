cmake_minimum_required(VERSION 3.0.2)
project(synchronize)

find_package(catkin REQUIRED COMPONENTS rosbag message_filters roscpp rospy sensor_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${rosbag_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_all src/synchronize_all.cpp)
add_executable(${PROJECT_NAME}_all_CompressedImage src/synchronize_all_CompressedImage.cpp)
add_executable(stereo_synch src/stereo_synch.cpp)
add_executable(stereo_synch_CompressedImage src/stereo_synch_CompressedImage.cpp)
add_executable(stereo_inertial_synch src/stereo_inertial_synch.cpp)
add_executable(stereo_inertial_synch_CompressedImage src/stereo_inertial_synch_CompressedImage.cpp)

target_link_libraries(${PROJECT_NAME}_all ${catkin_LIBRARIES} ${rosbag_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_all_CompressedImage ${catkin_LIBRARIES} ${rosbag_LIBRARIES})
target_link_libraries(stereo_synch ${catkin_LIBRARIES} ${rosbag_LIBRARIES})
target_link_libraries(stereo_synch_CompressedImage ${catkin_LIBRARIES} ${rosbag_LIBRARIES})
target_link_libraries(stereo_inertial_synch ${catkin_LIBRARIES} ${rosbag_LIBRARIES})
target_link_libraries(stereo_inertial_synch_CompressedImage ${catkin_LIBRARIES} ${rosbag_LIBRARIES})