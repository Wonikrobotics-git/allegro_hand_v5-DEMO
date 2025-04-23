cmake_minimum_required(VERSION 3.0.2)
project(rviz_embedded)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Qt5 및 ROS 패키지 찾기
find_package(Qt5 REQUIRED Core Widgets)
find_package(roscpp REQUIRED)
find_package(roslib REQUIRED)
find_package(rviz REQUIRED)

include_directories(
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${rviz_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

# 실행 파일 추가
add_executable(rviz_embedded_node src/rviz_main.cpp)

# Qt5 및 RViz 라이브러리 링크
target_link_libraries(rviz_embedded_node
  Qt5::Core
  Qt5::Widgets
  ${roscpp_LIBRARIES}
  ${rviz_LIBRARIES}
)

# 헤더 파일 디렉토리 추가

# install(TARGETS rviz_embedded_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )