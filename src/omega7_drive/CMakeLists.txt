cmake_minimum_required(VERSION 2.8.3)
project(omega7_drive)

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

)


include_directories(
  "include/omega7_driver"
  ${catkin_INCLUDE_DIRS}
  "/home/kaanh/teloperation_v1/src/omega7_drive/include/omega7_drive"
)
add_executable(omega_single src/omega_single.cpp)
target_link_libraries(omega_single  ${catkin_LIBRARIES} ${Boost_LIBRARIES} "/home/kaanh/teloperation_v1/src/omega7_drive/include/omega7_drive/libdhd.so.3" "/home/kaanh/teloperation_v1/src/omega7_drive/include/omega7_drive/libdrd.so.3")


