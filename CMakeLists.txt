cmake_minimum_required(VERSION 2.8.3)
project(recharge_behaviour)

find_package(catkin REQUIRED COMPONENTS
  roscpp
	std_msgs
  geometry_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs tf 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(recharge_behaviour src/recharge_behaviour.cpp src/main.cpp)

target_link_libraries(recharge_behaviour
   ${catkin_LIBRARIES}
)
