cmake_minimum_required(VERSION 3.0.2)
project(virtual_map)

# Find catkin macros and libraries
# if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_conversions
  geometry_msgs
  object_msgs
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# # Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/virtual_map.cc
# )

# Declare a C++ executable
# With catkin_make all packages are built within a single CMake context
# The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node
  src/virtual_map.cpp
  src/virtual_map_node.cpp
)

add_dependencies(${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
)

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
