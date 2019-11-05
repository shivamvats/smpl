cmake_minimum_required(VERSION 2.8.3)
project(trac_ik_robot_model)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(
    catkin
    REQUIRED
    COMPONENTS
        eigen_conversions
        trac_ik_lib
        leatherman
        roscpp
        smpl_urdf_robot_model
        urdf)

find_package(orocos_kdl REQUIRED)
find_package(smpl REQUIRED)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        trac_ik_robot_model
    CATKIN_DEPENDS
        roscpp
        smpl_urdf_robot_model
        urdf
        trac_ik_lib
    DEPENDS
        orocos_kdl
        smpl)

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${orocos_kdl_INCLUDE_DIRS})

add_library(trac_ik_robot_model src/trac_ik_robot_model.cpp)
target_link_libraries(trac_ik_robot_model ${catkin_LIBRARIES})
target_link_libraries(trac_ik_robot_model ${orocos_kdl_LIBRARIES})
target_link_libraries(trac_ik_robot_model smpl::smpl)

#add_executable(test_trac_ik src/test_trac_ik_robot_model.cpp)
#target_link_libraries(test_trac_ik trac_ik_robot_model)
#target_link_libraries(test_trac_ik ${leatherman_LIBRARIES})
#target_link_libraries(test_trac_ik ${roscpp_LIBRARIES})

install(
    DIRECTORY include/trac_ik_robot_model/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(
    TARGETS trac_ik_robot_model #test_trac_ik
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
