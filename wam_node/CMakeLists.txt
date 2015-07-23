cmake_minimum_required(VERSION 2.8.3)
project(wam_node)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  roscpp
  tf
  wam_msgs
  wam_srvs
  rospy
  rosbag
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(BULLET bullet)

find_package(Barrett REQUIRED)
link_directories(${BARRETT_LIB_DIRS})
add_definitions(${BARRETT_DEFINITIONS})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wam_grasp_logging
#  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

###################
### BUILD INFO ####
###################
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${BARRETT_INCLUDE_DIRS})
include_directories(${BULLET_INCLUDE_DIRS})

add_executable(wam_node src/wam_node.cpp )
add_dependencies(wam_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(wam_node ${catkin_LIBRARIES} ${BARRETT_LIBRARIES} ${BULLET_LIBRARIES})
set_target_properties(wam_node PROPERTIES COMPILE_FLAGS "-Wall -g ")

target_link_libraries(wam_node ${BARRETT_LIBRARIES})

