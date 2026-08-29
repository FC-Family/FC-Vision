cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 OLD)
project(sop)

add_definitions(-D _GLIBCXX_USE_CXX11_ABI=1)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
get_filename_component(PROJECT_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)

find_package(catkin REQUIRED COMPONENTS
    roscpp
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sop_solver
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include ${PROJECT_ROOT_DIR}
    ${Eigen3_INCLUDE_DIRS} 
)

add_library( sop_solver
    src/sop.cpp
    src/hungarian.cpp
    src/solver.cpp
    src/digraph.cpp
    src/edge.cpp
    src/history_node.cpp
    src/solver_state.cpp
    )

target_link_libraries( sop_solver
    ${catkin_LIBRARIES} 
    )