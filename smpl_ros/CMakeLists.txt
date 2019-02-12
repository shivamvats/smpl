cmake_minimum_required(VERSION 2.8.12)
project(smpl_ros)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(Eigen3 REQUIRED)

find_package(sbpl REQUIRED)
set(sbpl_INCLUDE_DIRS ${SBPL_INCLUDE_DIRS})
set(sbpl_LIBRARIES ${SBPL_LIBRARIES})

find_package(
    catkin
    REQUIRED
    COMPONENTS
        eigen_conversions
        leatherman
        moveit_core
        moveit_msgs
        roscpp
        trajectory_msgs
        visualization_msgs)

find_package(smpl REQUIRED)

catkin_package(
    INCLUDE_DIRS
        include ${EIGEN3_INCLUDE_DIR}
    LIBRARIES
        smpl_ros
    CATKIN_DEPENDS
        leatherman
        moveit_core
        moveit_msgs
        roscpp
        trajectory_msgs
        visualization_msgs
    DEPENDS
        Boost
        sbpl
        smpl)

add_definitions(-DSV_PACKAGE_NAME="smpl_ros")

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include_directories(SYSTEM ${sbpl_INCLUDE_DIRS})

add_library(
    smpl_ros
    src/debug/visualizer_ros.cpp
    src/debug/marker_conversions.cpp
    src/ros/factories.cpp
    src/ros/planner_interface.cpp
    src/ros/propagation_distance_field.cpp)

target_link_libraries(smpl_ros ${Boost_LIBRARIES})
target_link_libraries(smpl_ros ${catkin_LIBRARIES})
target_link_libraries(smpl_ros smpl::smpl)

install(
    DIRECTORY include/smpl/
    DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/smpl)
install(
    TARGETS smpl_ros
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
