cmake_minimum_required(VERSION 3.0.2)
project(frob_teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  joy
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(frob_teleop src/frob_teleop.cpp)
target_link_libraries(frob_teleop ${catkin_LIBRARIES})
