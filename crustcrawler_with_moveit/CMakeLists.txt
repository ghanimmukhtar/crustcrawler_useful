cmake_minimum_required(VERSION 2.8.3)
project(crustcrawler_with_moveit)

find_package(catkin REQUIRED COMPONENTS
  crustcrawler_core_msgs
  moveit_ros_planning_interface
  roscpp
  rospy
  std_msgs
  tf
  trajectory_msgs
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES crustcrawler_with_moveit
#  CATKIN_DEPENDS crustcrawler_core_msgs moveit_ros_planning_interface roscpp rospy std_msgs tf trajectory_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(crustcrawler_test_moveit src/crustcrawler_test_moveit.cpp)
target_link_libraries(crustcrawler_test_moveit ${catkin_LIBRARIES}  )
