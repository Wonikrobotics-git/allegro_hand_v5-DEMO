cmake_minimum_required(VERSION 2.8.3)
project(allegro_hand_controllers)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  allegro_hand_driver
  bhand
  libpcan
  roscpp
  sensor_msgs
  roslib
# roslaunch  # Disabled due to testing
)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)


###################################
## catkin specific configuration ##
###################################
catkin_package(
 # INCLUDE_DIRS include
#  LIBRARIES allegro_hand_controllers
  CATKIN_DEPENDS allegro_hand_driver bhand libpcan roscpp sensor_msgs std_msgs
 
 DEPENDS Boost system_lib
)



###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
   ${YAML_CPP_INCLUDE_DIR}
)

add_library(allegro_node src/allegro_node.cpp)

## Declare executables
add_executable(allegro_node_grasp src/allegro_node_grasp.cpp)
target_link_libraries(allegro_node_grasp allegro_node ${catkin_LIBRARIES}  ${YAML_CPP_LIBRARIES})



# This is disabled until roslaunch tests can pass required arguments.
# roslaunch_add_file_check(launch/allegro_hand.launch)

#############
## Install ##
#############

install(TARGETS
  allegro_node
  allegro_node_grasp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
  FILES_MATCHING PATTERN "*.launch"
)
