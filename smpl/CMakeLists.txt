cmake_minimum_required(VERSION 2.8.12)
project(smpl)

###################################
# overall build mode and features #
###################################

set(SMPL_CONSOLE_ROS ON CACHE BOOL "Replace smpl log macros with rosconsole macros")
set(SMPL_SV_VISUALIZATION_MSGS ON CACHE BOOL "Include visualization_msgs in visualizer interface")

# Seed options using the default behavior of BUILD_SHARED_LIBS
#if(BUILD_SHARED_LIBS)
#    option(SMPL_BUILD_SHARED "Build smpl as a shared library" ON)
#    option(SMPL_BUILD_STATIC "Build smpl as a static library" OFF)
#else()
#    option(SMPL_BUILD_SHARED "Build smpl as a shared library" OFF)
#    option(SMPL_BUILD_STATIC "Build smpl as a static library" ON)
#endif()

# Less ideal default behavior, but integrates better with catkin since
# BUILD_SHARED_LIBS is not automatically passed to non-catkin packages.
option(SMPL_BUILD_SHARED "Build smpl as a shared library" ON)
option(SMPL_BUILD_STATIC "Build smpl as a static library" OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

#######################
# version information #
#######################

set(SMPL_MAJOR_VERSION 0)
set(SMPL_MINOR_VERSION 0)
set(SMPL_PATCH_VERSION 0)
set(SMPL_VERSION ${SMPL_MAJOR_VERSION}.${SMPL_MINOR_VERSION}.${SMPL_PATCH_VERSION})

####################################################
# allow overriding install directories for install #
####################################################

set(SMPL_INSTALL_LIB_DIR     lib             CACHE PATH "Installation directory for libraries")
set(SMPL_INSTALL_INCLUDE_DIR include         CACHE PATH "Installation directory for header files")
set(SMPL_INSTALL_CMAKE_DIR   lib/cmake/smpl  CACHE PATH "Installation directory for CMake files")
set(SMPL_INSTALL_BIN_DIR     bin             CACHE PATH "Installation directory for executables")
foreach(path LIB INCLUDE CMAKE BIN)
    set(var SMPL_INSTALL_${path}_DIR)
    if(NOT IS_ABSOLUTE "${${var}}")
        set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
    endif()
endforeach()

########################
# resolve dependencies #
########################

find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
find_package(Eigen3 REQUIRED)
find_package(sbpl REQUIRED)
#find_package(smpl_urdf_robot_model)
#find_package(sbpl_kdl_robot_model)

if(SMPL_CONSOLE_ROS)
    find_package(roscpp QUIET)
    if(roscpp_FOUND)
        set(SMPL_HAS_ROSCONSOLE 1)
    else()
        set(SMPL_HAS_ROSCONSOLE 0)
    endif()
else()
    set(SMPL_HAS_ROSCONSOLE 0)
endif()

if(SMPL_SV_VISUALIZATION_MSGS)
    find_package(visualization_msgs QUIET)
    if (visualization_msgs_FOUND)
        set(SMPL_HAS_VISUALIZATION_MSGS 1)
    else()
        set(SMPL_HAS_VISUALIZATION_MSGS 0)
    endif()
else()
    set(SMPL_HAS_VISUALIZATION_MSGS 0)
endif()

#####################################
# generate .cmake files for install #
#####################################

file(RELATIVE_PATH REL_INCLUDE_DIR "${SMPL_INSTALL_CMAKE_DIR}" "${SMPL_INSTALL_INCLUDE_DIR}")
set(CONF_INCLUDE_DIRS "\${SMPL_CMAKE_DIR}/${REL_INCLUDE_DIR}")

configure_file(smpl-config.cmake.in "${PROJECT_BINARY_DIR}/smpl-config.cmake" @ONLY)
configure_file(smpl-config-version.cmake.in "${PROJECT_BINARY_DIR}/smpl-config-version.cmake" @ONLY)

#################################
# generate configuration header #
#################################

set(SMPL_CLOCK_API SMPL_CLOCK_CHRONO_HIGH_RESOLUTION)

configure_file(
    ${PROJECT_SOURCE_DIR}/include/smpl/config.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/include/smpl/config.h)

########################
# compile time options #
########################

if(NOT MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

if(SMPL_HAS_ROSCONSOLE)
    add_definitions(-DSMPL_CONSOLE_ROS)
endif()

add_definitions(-DSV_PACKAGE_NAME="smpl")

set(SMPL_LIBRARY_SOURCES
    src/csv_parser.cpp
    src/collision_checker.cpp
    src/console/ansi.cpp
    src/console/console.cpp
    src/occupancy_grid.cpp
    src/planning_params.cpp
    src/post_processing.cpp
    src/robot_model.cpp
    src/bfs3d/bfs3d.cpp
    src/debug/colors.cpp
    src/debug/marker_utils.cpp
    src/debug/visualize.cpp
    src/distance_map/chessboard_distance_map.cpp
    src/distance_map/distance_map_common.cpp
    src/distance_map/edge_euclid_distance_map.cpp
    src/distance_map/euclid_distance_map.cpp
    src/distance_map/sparse_distance_map.cpp
    src/geometry/bounding_spheres.cpp
    src/geometry/intersect.cpp
    src/geometry/mesh_utils.cpp
    src/geometry/voxelize.cpp
    src/graph/action_space.cpp
    src/graph/adaptive_workspace_lattice.cpp
    src/graph/experience_graph.cpp
    src/graph/manip_lattice.cpp
    src/graph/manip_lattice_egraph.cpp
    src/graph/manip_lattice_action_space.cpp
    src/graph/robot_planning_space.cpp
    src/graph/workspace_lattice.cpp
    src/graph/workspace_lattice_base.cpp
    src/graph/workspace_lattice_egraph.cpp
    src/graph/simple_workspace_lattice_action_space.cpp
    src/heuristic/attractor_heuristic.cpp
    src/heuristic/bfs_heuristic.cpp
    src/heuristic/bfs_fullbody_heuristic.cpp
    src/heuristic/egraph_bfs_heuristic.cpp
    src/heuristic/generic_egraph_heuristic.cpp
    src/heuristic/euclid_dist_heuristic.cpp
    src/heuristic/euclid_diffdrive_heuristic.cpp
    src/heuristic/euclid_fullbody_heuristic.cpp
    src/heuristic/robot_heuristic.cpp
    src/heuristic/joint_dist_heuristic.cpp
    src/heuristic/multi_frame_bfs_heuristic.cpp
    src/heuristic/sparse_egraph_dijkstra_heuristic.cpp
    src/heuristic/zero_heuristic.cpp
    src/search/fmhastar.cpp
    src/search/meta_mhastar_dts.cpp
    src/search/mhastarpp.cpp
    src/search/umhastar.cpp
    src/search/arastar.cpp
    src/search/experience_graph_planner.cpp
    src/search/adaptive_planner.cpp
    src/search/lazy_arastar.cpp
    src/search/lazy_mhastar.cpp
    src/search/smhastar.cpp
    src/search/awastar.cpp
    src/steer/steer.cpp
    src/unicycle/dubins.cpp
    src/unicycle/unicycle.cpp)

set(CMAKE_DEBUG_POSTFIX "_d")

# Gather include directories
list(APPEND PRIVATE_HEADERS ${CMAKE_CURRENT_BINARY_DIR}/include)
list(APPEND PRIVATE_HEADERS include)

if(SMPL_HAS_ROSCONSOLE)
    list(APPEND PUBLIC_HEADERS ${roscpp_INCLUDE_DIRS})
endif()
if(SMPL_HAS_VISUALIZATION_MSGS)
    list(APPEND PUBLIC_HEADERS ${visualization_msgs_INCLUDE_DIRS})
endif()

list(APPEND PUBLIC_HEADERS ${EIGEN3_INCLUDE_DIR})
list(APPEND PUBLIC_HEADERS ${Boost_INCLUDE_DIRS})
list(APPEND PUBLIC_HEADERS ${SBPL_INCLUDE_DIRS})
#list(APPEND PUBLIC_HEADERS ${smpl_urdf_robot_model_INCLUDE_DIRS})
#list(APPEND PUBLIC_HEADERS ${sbpl_kdl_robot_model_INCLUDE_DIRS})

list(APPEND INTERFACE_HEADERS ${SMPL_INSTALL_INCLUDE_DIR})

# Gather libraries
list(APPEND PRIVATE_LIBRARIES ${Boost_FILESYSTEM_LIBRARY})
list(APPEND PRIVATE_LIBRARIES ${Boost_PROGRAM_OPTIONS_LIBRARY})
list(APPEND PRIVATE_LIBRARIES ${Boost_SYSTEM_LIBRARY})

if(SMPL_HAS_ROSCONSOLE)
    list(APPEND PUBLIC_LIBRARIES ${roscpp_LIBRARIES})
endif()
list(APPEND PUBLIC_LIBRARIES ${SBPL_LIBRARIES})
#list(APPEND PUBLIC_LIBRARIES ${smpl_urdf_robot_model_LIBRARIES})
#list(APPEND PUBLIC_LIBRARIES ${sbpl_kdl_robot_model_LIBRARIES})

macro(include_common_directories name)
    target_include_directories(${name} PRIVATE ${PRIVATE_HEADERS})
    target_include_directories(${name} SYSTEM PUBLIC ${PUBLIC_HEADERS})
    target_include_directories(${name} INTERFACE ${INTERFACE_HEADERS})
endmacro()
macro(link_common_libraries name)
    target_link_libraries(${name} PRIVATE ${PRIVATE_LIBRARIES})
    target_link_libraries(${name} PUBLIC ${PUBLIC_LIBRARIES})
    if(NOT Boost_USE_STATIC_LIBS)
        target_compile_definitions(${name} PRIVATE -DBOOST_ALL_DYN_LINK)
    endif()

    if(MSVC)
        target_compile_definitions(${name} PRIVATE -DBOOST_ALL_NO_LIB)
    endif()
endmacro()
macro(add_common_compile_definitions name)
    if(NOT Boost_USE_STATIC_LIBS)
        target_compile_definitions(${name} PRIVATE -DBOOST_ALL_DYN_LINK)
    endif()

    if(MSVC)
        target_compile_definitions(${name} PRIVATE -DBOOST_ALL_NO_LIB)
    endif()
endmacro()

####################################
# Build the shared library version #
####################################

if(SMPL_BUILD_SHARED)
    add_library(smpl-shared SHARED ${SMPL_LIBRARY_SOURCES})

    set_target_properties(smpl-shared PROPERTIES OUTPUT_NAME smpl)

    set_target_properties(
        smpl-shared
        PROPERTIES
            VERSION ${SMPL_VERSION}
            SOVERSION ${SMPL_MAJOR_VERSION}.${SMPL_MINOR_VERSION})

    include_common_directories(smpl-shared)
    link_common_libraries(smpl-shared)
    add_common_compile_definitions(smpl-shared)
endif()

####################################
# Build the static library version #
####################################

if(SMPL_BUILD_STATIC)
    add_library(smpl-static STATIC ${SMPL_LIBRARY_SOURCES})

    set_target_properties(smpl-static PROPERTIES OUTPUT_NAME smpl)
    if(MSVC)
        set_target_properties(smpl-static PROPERTIES PREFIX lib)
    endif()

    include_common_directories(smpl-static)
    link_common_libraries(smpl-static)
    add_common_compile_definitions(smpl-static)
endif()

################
# Installation #
################

set(SMPL_INCLUDE_DESTINATION ${SMPL_INSTALL_INCLUDE_DIR}/smpl)
set(SMPL_ARCHIVE_DESTINATION ${SMPL_INSTALL_LIB_DIR})
set(SMPL_LIBRARY_DESTINATION ${SMPL_INSTALL_LIB_DIR})
set(SMPL_RUNTIME_DESTINATION ${SMPL_INSTALL_BIN_DIR})

install(
    DIRECTORY include/smpl/
    DESTINATION ${SMPL_INCLUDE_DESTINATION}
    COMPONENT dev
    FILES_MATCHING PATTERN *.h PATTERN *.hpp)
install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/include/smpl/config.h
    DESTINATION ${SMPL_INCLUDE_DESTINATION}
    COMPONENT dev)

if(SMPL_BUILD_SHARED)
    list(APPEND BUILT_LIBRARIES smpl-shared)
endif()
if(SMPL_BUILD_STATIC)
    list(APPEND BUILT_LIBRARIES smpl-static)
endif()

install(
    TARGETS ${BUILT_LIBRARIES}
    EXPORT smpl-targets
    ARCHIVE DESTINATION ${SMPL_ARCHIVE_DESTINATION}
    LIBRARY DESTINATION ${SMPL_LIBRARY_DESTINATION}
    RUNTIME DESTINATION ${SMPL_RUNTIME_DESTINATION})

install(
    EXPORT smpl-targets
    DESTINATION ${SMPL_INSTALL_CMAKE_DIR}
    NAMESPACE smpl::
    COMPONENT dev)
install(
    FILES
        "${PROJECT_BINARY_DIR}/smpl-config.cmake"
        "${PROJECT_BINARY_DIR}/smpl-config-version.cmake"
    DESTINATION ${SMPL_INSTALL_CMAKE_DIR}
    COMPONENT dev)

