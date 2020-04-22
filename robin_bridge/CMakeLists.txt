cmake_minimum_required(VERSION 2.8.3)
project(robin_bridge)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  StringVarLenArray.msg
  TestStruct.msg
  AccelStampedArray.msg
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
   LIBRARIES robin robin_shared_memory robin_semaphore
   CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_generation
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#TODO? merge libraries
add_library(robin src/${PROJECT_NAME}/robin.cpp)
add_library(robin_inst src/${PROJECT_NAME}/robin_inst.cpp)  #TODO try without
add_library(robin_shared_memory src/${PROJECT_NAME}/shared_memory.cpp)
add_library(robin_semaphore src/${PROJECT_NAME}/semaphore.cpp)

add_executable(robin_node src/robin_node.cpp)
add_dependencies(robin_inst ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(robin_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(robin_node
  ${catkin_LIBRARIES}
  robin_inst
)
target_link_libraries(robin_inst
  ${catkin_LIBRARIES}
  robin
)
target_link_libraries(robin
  ${catkin_LIBRARIES}
  robin_semaphore
  robin_shared_memory
)
target_link_libraries(robin_shared_memory
  ${catkin_LIBRARIES}
#   rt
)
target_link_libraries(robin_semaphore
  ${catkin_LIBRARIES}
)

##############
### Testing ##
###############

if(CATKIN_ENABLE_TESTING)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs -ftest-coverage")
  catkin_add_gtest(test_semaphore src/test/test_semaphore.cpp)
  if(TARGET test_semaphore)
    target_link_libraries(test_semaphore
      ${catkin_LIBRARIES}
      robin_semaphore
    )
  endif()
endif()
