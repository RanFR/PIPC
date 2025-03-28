cmake_minimum_required(VERSION 3.0)
project(obstacles_prediction)

set(USE_TEST True)
# set(USE_TEST False)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    sensor_msgs
    pcl_conversions
    visualization_msgs
    object_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

find_package(OsqpEigen REQUIRED)
find_package(osqp REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME} OsqpEigen::OsqpEigen
    CATKIN_DEPENDS geometry_msgs sensor_msgs pcl_conversions object_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/obstacles_prediction.cpp
)

add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    OsqpEigen::OsqpEigen
)

if (USE_TEST)
add_executable(test_obstacles_prediction
    test/test_obstacles_prediction.cpp
)
add_dependencies(test_obstacles_prediction
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(test_obstacles_prediction
    ${PROJECT_NAME}
)
endif(USE_TEST)
