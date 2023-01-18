
cmake_minimum_required(VERSION 3.0.2)
project(vention_homework)

add_compile_options(-std=c++11)
add_definitions(-D_OS_UNIX)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation actionlib_msgs actionlib moveit_core moveit_ros_planning_interface moveit_visual_tools rviz_visual_tools vision_msgs)

find_package(Eigen3)

## Generate messages
generate_messages(DEPENDENCIES std_msgs actionlib_msgs geometry_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS moveit_core moveit_ros_planning_interface moveit_visual_tools rviz_visual_tools actionlib vision_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${PROJECT_SOURCE_DIR}/src)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
    
if(CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)
endif()

