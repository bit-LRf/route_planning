cmake_minimum_required(VERSION 3.0.2)
project(timepublisher)

find_package(catkin REQUIRED COMPONENTS
gazebo_plugins
gazebo_ros
gazebo_ros_control
message_generation
roscpp
rospy
std_msgs
tf2
tf2_geometry_msgs
tf2_ros
urdf
xacro
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES route_planning
 CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include ${catkin_INCLUDE_DIRS}

  ${catkin_INCLUDE_DIRS}
)

add_executable(timepublisher 
  src/timepublisher.cpp
)

add_dependencies(timepublisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(timepublisher
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)

add_definitions("-g -Wall")
