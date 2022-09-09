cmake_minimum_required(VERSION 2.8.3)
project(aruco_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  genmsg
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ArucoDetection.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
)

catkin_package(
 DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
