cmake_minimum_required(VERSION 2.8.3)
project(pose_graph_utils)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
add_compile_options(-std=c++11 -msse -msse2 -msse3 -msse4)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -msse -msse2 -msse3 -msse4")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -msse -msse2 -msse3 -msse4")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

find_package(Eigen3 REQUIRED)
find_package(G2O REQUIRED)

find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
)

catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
  DEPENDS
    EIGEN3
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS 
    cmake_modules
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIR}
  ${G2O_INCLUDE_DIRS}
  /usr/include/suitesparse
)

LIST(APPEND LINK_LIBS
  ${catkin_LIBRARIES}
  ${G2O_CORE_LIBRARY} 
  ${G2O_STUFF_LIBRARY} 
  ${G2O_SOLVER_CHOLMOD} 
  ${G2O_SOLVER_CSPARSE} 
  ${G2O_SOLVER_DENSE}
  ${G2O_SOLVER_PCG} 
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_SLAM3D_ADDONS}
  ${G2O_TYPES_DATA}
  ${G2O_TYPES_SBA}
  cholmod
  cxsparse
)


## Build library from source files
add_library(${PROJECT_NAME}
  src/slam_3d/pose_graph_g2o.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${LINK_LIBS}
)

