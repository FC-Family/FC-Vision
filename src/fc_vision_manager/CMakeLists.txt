cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 OLD)
project(fc_vision_manager)

add_definitions(-D _GLIBCXX_USE_CXX11_ABI=1)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
get_filename_component(PROJECT_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
# LIBIGL
set(LIBIGL_INCLUDE_DIR "${PROJECT_ROOT_DIR}/3rdparty/libigl/include")
set(LIBIGL_LIBRARY "${PROJECT_ROOT_DIR}/3rdparty/libigl/libigl.a")
find_package(catkin REQUIRED COMPONENTS
    active_perception
    airsim_ros_pkgs
    geometry_msgs
    message_filters
    nav_msgs
    path_searching
    plan_env
    quadrotor_msgs
    roscpp
    roslib
    sensor_msgs
    std_msgs
    tf
    vis_utils
    visualization_msgs
    visibility_replan
)
find_package(roslib)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES manager_node
  CATKIN_DEPENDS active_perception airsim_ros_pkgs geometry_msgs message_filters nav_msgs path_searching plan_env quadrotor_msgs roscpp roslib sensor_msgs std_msgs tf vis_utils visualization_msgs visibility_replan
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include ${PROJECT_ROOT_DIR} ${PROJECT_ROOT_DIR}/utils
    ${LIBIGL_INCLUDE_DIR}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

link_directories(
    ${PCL_LIBRARY_DIRS})

add_library (manager_node SHARED
    src/manager_node.cpp
    src/manager_utils.cpp
)
target_link_libraries( manager_node
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${LIBIGL_LIBRARY}
    yaml-cpp
)

add_executable(fc_vision
    src/fc_vision_manager.cpp
    )

target_link_libraries( fc_vision
    manager_node
    )

add_executable( traj_server
    src/traj_server.cpp
)

target_link_libraries( traj_server
    ${catkin_LIBRARIES}
    yaml-cpp
)

add_dependencies(manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(fc_vision ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
