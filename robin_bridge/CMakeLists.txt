cmake_minimum_required(VERSION 2.8.3)
project(robin_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  StringVarLenArray.msg
  TestStruct.msg
  AccelStampedArray.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES robin
#   DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library  #TODO? merge libraries
add_library(robin src/${PROJECT_NAME}/robin.cpp)
add_library(robin_publisher src/${PROJECT_NAME}/robin_publisher.cpp)
add_library(robin_subscriber src/${PROJECT_NAME}/robin_subscriber.cpp)
add_library(robin_inst src/${PROJECT_NAME}/robin_inst.cpp)  #TODO try without
add_library(robin_shared_memory src/${PROJECT_NAME}/shared_memory.cpp)
add_library(robin_semaphore src/${PROJECT_NAME}/semaphore.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(robin_node src/robin_node.cpp)
# add_executable(robin_timer src/robin_timer.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
set_target_properties(robin_node PROPERTIES OUTPUT_NAME robin PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(robin_node
  ${catkin_LIBRARIES}
  robin_inst
)
target_link_libraries(robin_inst
  robin_publisher
  robin_subscriber
)
target_link_libraries(robin_publisher
  ${catkin_LIBRARIES}
  robin
)
target_link_libraries(robin_subscriber
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
# target_link_libraries(robin_timer
#   robin_shared_memory_inst
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

##############
### Testing ##
###############

### Add gtest based cpp test target and link libraries
#catkin_add_gtest(test_semaphore src/test/test_semaphore.cpp)
#if(TARGET test_semaphore)
#  target_link_libraries(test_semaphore
#    ${catkin_LIBRARIES}
#    ${PROJECT_NAME}_semaphore
#  )
#endif()
#catkin_add_gtest(test_shared_memory src/test/test_shared_memory.cpp)
#if(TARGET test_shared_memory)
#  target_link_libraries(test_shared_memory
#    ${catkin_LIBRARIES}
#    ${PROJECT_NAME}_shared_memory
#  )
#endif()
##catkin_add_gtest(test_robin src/test/test_robin.cpp)
##if(TARGET test_robin)
##  target_link_libraries(test_robin
##    ${catkin_LIBRARIES}
##    ${PROJECT_NAME}
##  )
##endif()
##catkin_add_gtest(test_robin_node src/test/test_robin_node.cpp)
##if(TARGET test_robin_node)
##  target_link_libraries(test_robin_node
##    ${catkin_LIBRARIES}
##    ${PROJECT_NAME}
##  )
##endif()#

### Add folders to be run by python nosetests
## catkin_add_nosetests(test)#

### Using rostest
#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
##  # test semaphore
##  add_rostest_gtest(test_semaphore test/semaphore.test src/test/test_semaphore.cpp)
##  target_link_libraries(test_semaphore ${catkin_LIBRARIES} ${PROJECT_NAME}_semaphore)
##  ## test shared memory
##  add_rostest_gtest(test_shared_memory test/shared_memory.test src/test/test_shared_memory.cpp)
##  target_link_libraries(test_shared_memory ${catkin_LIBRARIES} ${PROJECT_NAME}_shared_memory)
##  ## test robin class
##  add_rostest_gtest(test_robin test/robin.test src/test/test_robin.cpp)
#  catkin_add_executable_with_gtest(test_robin test/robin.test src/test/test_robin.cpp)
#  target_link_libraries(test_robin ${catkin_LIBRARIES} ${PROJECT_NAME})
##  ## test robin node
##  #add_rostest_gtest(test_robin_node test/robin_node.test src/test/test_robin_node.cpp)
##  #target_link_libraries(test_robin_node ${catkin_LIBRARIES})
#endif()