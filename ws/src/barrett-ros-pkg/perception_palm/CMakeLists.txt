cmake_minimum_required(VERSION 2.8.3)
project(perception_palm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  wam_msgs
  wam_srvs
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(UDEV REQUIRED libudev)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  message_runtime
  wam_msgs
  wam_srvs
)

include_directories(include
  ${catkin_INCLUDE_DIRS} ${PROJECT_NAME}/include/MCP2210-Library ${UDEV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} include/MCP2210-Library/ppalm_driver.cpp include/MCP2210-Library/hid.c include/MCP2210-Library/mcp2210.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${UDEV_LIBRARIES})
add_executable(perception_palm_node src/palm_mcp.cpp)
target_link_libraries(perception_palm_node
   ${PROJECT_NAME} ${catkin_LIBRARIES}
 )
add_dependencies(perception_palm_node ${catkin_EXPORTED_TARGETS} wam_msgs_gencpp)

install(FILES include/MCP2210-Library/ir_calibration.txt
  DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/
)

option(${PROJECT_NAME}_INSTALL_UDEV_RULES "Install udev rules for MCP2210 Library" ON)
if (${PROJECT_NAME}_INSTALL_UDEV_RULES)
install (
FILES include/MCP2210-Library/99-hid.rules
DESTINATION "/etc/udev/rules.d"
COMPONENT "udev")
else ()
message (STATUS "Udev rules for MCP2210 Library not being installed")
endif ()
