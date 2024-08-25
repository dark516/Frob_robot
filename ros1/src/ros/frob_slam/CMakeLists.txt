cmake_minimum_required(VERSION 3.0.2)
project(frob_slam)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  gmapping
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

LINK_DIRECTORIES("usr/local/lib")
