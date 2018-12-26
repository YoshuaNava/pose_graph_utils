cmake_minimum_required(VERSION 2.8.3)
project(pose_graph_utils)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    g2o_catkin
    gtsam_catkin
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  DEPENDS EIGEN3
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    cmake_modules
    g2o_catkin
    gtsam_catkin
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  /usr/include/suitesparse
)

LIST(APPEND LINK_LIBS 
  ${catkin_LIBRARIES}
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

