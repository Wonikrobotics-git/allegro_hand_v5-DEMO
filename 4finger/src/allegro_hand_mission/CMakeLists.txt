cmake_minimum_required(VERSION 3.0.2)
project(allegro_hand_mission)

## Compile as C++11
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  bhand
)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS bhand
)

## Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(mission_node src/mission.cpp)

## Add dependencies to ensure proper build order
add_dependencies(mission_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Link libraries
target_link_libraries(mission_node
  ${catkin_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)