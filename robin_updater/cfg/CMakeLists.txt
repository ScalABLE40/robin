cmake_minimum_required(VERSION 2.8.3)
project(robin_bridge_generated)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  robin_bridge
  std_msgs
  geometry_msgs
  message_generation
)


################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp robin_bridge std_msgs geometry_msgs message_generation
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(robin_inst_generated src/${PROJECT_NAME}/robin_inst.cpp)
add_executable(robin_node_generated src/robin_node.cpp)

add_dependencies(robin_inst_generated ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(robin_node_generated ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(robin_inst_generated
  ${catkin_LIBRARIES}
)

target_link_libraries(robin_node_generated
  ${catkin_LIBRARIES}
  robin_inst_generated
)

