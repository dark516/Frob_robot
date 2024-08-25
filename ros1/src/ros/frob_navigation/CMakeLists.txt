cmake_minimum_required(VERSION 3.0.2)
project(frob_navigation)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
