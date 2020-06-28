cmake_minimum_required(VERSION 2.8.3)
project(robot_inverse_kinematic)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

set(EIGEN_INC_DIR /usr/local/include/Eigen/)
include_directories(${EIGEN_INC_DIR})


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  robot_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES davinci_planing
  CATKIN_DEPENDS roscpp rospy std_msgs 
)



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


add_executable(robot_inverse_single_ver1 src/robot_inverse_single_ver1.cpp)
target_link_libraries(robot_inverse_single_ver1 ${catkin_LIBRARIES} ${Boost_LIBRARIES})




