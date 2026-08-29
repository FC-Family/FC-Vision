cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 OLD)
project(traj_generator)

add_definitions(-D _GLIBCXX_USE_CXX11_ABI=1)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
get_filename_component(PROJECT_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)

find_package(catkin REQUIRED COMPONENTS
    vis_utils
    roscpp
    roslib
    std_msgs
    plan_env
    quadrotor_msgs
)
find_package(roslib)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES traj_gen
  CATKIN_DEPENDS plan_env
  DEPENDS OpenCV
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include ${PROJECT_ROOT_DIR} ${PROJECT_ROOT_DIR}/utils
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
)

link_directories(
    ${PCL_LIBRARY_DIRS})

add_library (traj_gen
    src/traj_gen.cpp
)

target_link_libraries( traj_gen
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${OMPL_LIBRARIES}
    yaml-cpp
)
