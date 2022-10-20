cmake_minimum_required(VERSION 3.0.2)
project(movegroup_camera)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  moveit_ros_planning_interface
  roscpp
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport moveit_ros_planning_interface roscpp sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pose_goal_camera src/pose_goal_camera.cpp)
target_link_libraries(pose_goal_camera ${catkin_LIBRARIES})
