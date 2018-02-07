cmake_minimum_required(VERSION 2.8.3)
project(limitcloud)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
  roscpp
  sensor_msgs)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS})

add_executable(limitcloud src/limitcloud.cpp)
target_link_libraries(limitcloud ${catkin_LIBRARIES})