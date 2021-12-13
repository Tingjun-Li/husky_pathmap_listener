cmake_minimum_required(VERSION 2.8.3)
project(pathmap)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  tf
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  message_filters
)


## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pathmap_listenser_node "${PROJECT_SOURCE_DIR}/pathmap_listener.cpp")
target_link_libraries(pathmap_listenser_node ${catkin_LIBRARIES})

# %EndTag(FULLTEXT)%