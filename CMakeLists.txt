cmake_minimum_required(VERSION 3.1.2)
project(pathfinder)

find_package(catkin REQUIRED
  roscpp
  cmake_modules
  nodelet
  mrs_lib
  std_msgs
  nav_msgs
  mrs_msgs
  )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set(CMAKE_CXX_FLAGS "-std=c++1z -O3")

find_package(octomap REQUIRED)
find_package(dynamicEDT3D REQUIRED)

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp nodelet mrs_lib std_msgs nav_msgs mrs_msgs
  LIBRARIES Pathfinder MrsAstarPlanner
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${octomap_INCLUDE_DIRS}
  )

# astar planner

add_library(MrsAstarPlanner
  src/astar_planner.cpp
  )

add_dependencies(MrsAstarPlanner
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsAstarPlanner
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${DYNAMICEDT3D_LIBRARIES}
  )

# pathfinder

add_library(Pathfinder
  src/pathfinder.cpp
  )

add_dependencies(Pathfinder
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(Pathfinder
  MrsAstarPlanner
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${DYNAMICEDT3D_LIBRARIES}
  )
