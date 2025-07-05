cmake_minimum_required(VERSION 2.8.3)
project(wam_node)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_srvs
  wam_msgs
  wam_srvs
)
find_package(Barrett REQUIRED)
include_directories(${BARRETT_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

## Boost
set(boost_components system thread python)
find_package(Boost REQUIRED ${boost_components})
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

set(exported_include_dirs ${exported_include_dirs} ${Boost_INCLUDE_DIRS})
set(exported_link_dirs ${exported_link_dirs} ${Boost_LIBRARY_DIRS})

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_srvs
  wam_msgs
  wam_srvs
)

add_executable(wam_node src/wam_node.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wam_node ${catkin_LIBRARIES} ${BARRETT_LIBRARIES})
