cmake_minimum_required(VERSION 2.8.3)
project(frob_base)

add_compile_options(-std=c++11 -Wno-psabi)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  roscpp
  roslaunch
  hardware_interface
)

catkin_package(
  CATKIN_DEPENDS 
  	controller_manager 
  	roscpp 
  	hardware_interface
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node 
  src/frob_base.cpp
)

target_link_libraries(${PROJECT_NAME}_node 
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
