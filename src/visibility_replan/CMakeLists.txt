cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 OLD)
project(visibility_replan)

add_definitions(-D _GLIBCXX_USE_CXX11_ABI=1)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
get_filename_component(PROJECT_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    std_msgs
    plan_env
    active_perception
    quadrotor_msgs
    vis_utils
    sop
    path_searching
    traj_generator
)
find_package(roslib)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vis_replan replan_fsm
  CATKIN_DEPENDS plan_env vis_utils sop path_searching traj_generator
  DEPENDS OpenCV
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include ${PROJECT_ROOT_DIR} ${PROJECT_ROOT_DIR}/utils
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(
    ${PCL_LIBRARY_DIRS})

add_library (vis_replan SHARED
    src/visibility_replan.cpp
    src/visibility_replan_utils.cpp
    src/visibility_replan_tools.cpp
)

target_link_libraries( vis_replan
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    yaml-cpp
)

add_library (replan_fsm
    src/replan_fsm.cpp
    src/replan_fsm_tools.cpp
)

target_link_libraries( replan_fsm
    vis_replan
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    yaml-cpp
)
