cmake_minimum_required(VERSION 2.8.3)
project(test_crustcrawler_utils)

find_package(catkin REQUIRED COMPONENTS
  crustcrawler_mover_utils
  roscpp
  rospy
  std_msgs
  tf
  geometry_msgs
  shape_msgs
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -fPIC")

find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

include_directories(${EIGEN3_INCLUDE_DIR} /usr/local/include)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test_crustcrawler_utils
#  CATKIN_DEPENDS crustcrawler_mover_utils roscpp rospy std_msgs tf geometry_msgs shape_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_crustcrawler_utils src/test_crustcrawler_utils.cpp)
target_link_libraries(test_crustcrawler_utils ${catkin_LIBRARIES}  )

add_executable(service_user src/service_user.cpp)
target_link_libraries(service_user ${catkin_LIBRARIES}  )
