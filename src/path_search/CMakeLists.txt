cmake_minimum_required(VERSION 3.0.2)
project(path_search)

# set(TEST True)
set(TEST False)

# 启用C++17标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    obstacles_prediction
)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES astar kinodynamic_astar dwa ${PCL_LIBRARIES}
)

add_library(node_libs
    src/node/node.cpp
)

add_library(astar
    src/astar/astar.cpp
)
target_link_libraries(astar
    node_libs
    ${PCL_LIBRARIES}
)

add_library(kinodynamic_astar
    src/kinodynamic_astar/kinodynamic_astar.cpp
)
target_link_libraries(kinodynamic_astar
    node_libs
    ${PCL_LIBRARIES}
)

add_library(dwa
    src/dwa/dwa.cpp
)
target_link_libraries(dwa
    node_libs
    astar
    ${PCL_LIBRARIES}
)

if (TEST)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    visualization_msgs
    pcl_conversions
    obstacles_prediction
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
add_executable(test_astar
    test/test_astar.cpp
)
target_link_libraries(test_astar
    astar
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
add_executable(test_kino_astar
    test/test_kino_astar.cpp
)
target_link_libraries(test_kino_astar
    kinodynamic_astar
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(test_dwa test/test_dwa.cpp)
target_link_libraries(test_dwa node_libs dwa ${catkin_LIBRARIES} ${PCL_LIBRARIES})
endif()
