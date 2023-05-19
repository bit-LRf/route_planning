cmake_minimum_required(VERSION 3.0.2)
project(route_planning)

find_package(OpenCV REQUIRED QUIET)

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
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(main_plan 
  src/main_plan.cpp
  src/fun_plan.cpp
)

add_dependencies(main_plan ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(main_plan
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
)
