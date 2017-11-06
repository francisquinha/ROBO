cmake_minimum_required(VERSION 2.8.3)
project(reactive_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  stdr_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

set(CMAKE_BUILD_TYPE Release)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    tf
    stdr_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
)

####################### Line Following ##################################
add_executable(robot_wallfollow_node src/main.cpp src/wall_following.cpp)
add_dependencies(robot_wallfollow_node stdr_msgs_gencpp) # wait for stdr_msgs to be build
target_link_libraries(robot_wallfollow_node ${catkin_LIBRARIES})

# Install excecutables
install(TARGETS robot_wallfollow_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
