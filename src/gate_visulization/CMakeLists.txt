
cmake_minimum_required(VERSION 2.8.3)
project(gate_visualization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  message_runtime
  visualization_msgs
  tf
  pose_utils
)

generate_messages()

catkin_package()

#find_package(Armadillo REQUIRED)
#include_directories(${ARMADILLO_INCLUDE_DIRS})


include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(gate_visualization src/gate_visulization_node.cpp)

target_link_libraries(gate_visualization
   ${catkin_LIBRARIES}
   pose_utils
)


