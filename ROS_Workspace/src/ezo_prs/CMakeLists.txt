cmake_minimum_required(VERSION 3.0.2)
project(ezo_prs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/ezo_prs_node.cpp)
add_executable(${PROJECT_NAME}_sub src/ezo_prs_sub.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_sub ${catkin_LIBRARIES})
