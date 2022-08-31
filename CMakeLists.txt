cmake_minimum_required(VERSION 3.1.2)
project(pathfinder)

add_definitions("-DOCTOMAP_NODEBUGOUT")

find_package(catkin REQUIRED
  roscpp
  cmake_modules
  nodelet
  mrs_lib
  std_msgs
  nav_msgs
  mrs_msgs
  octomap_ros
  octomap_msgs
  mrs_subt_planning_lib
  )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(octomap REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)

find_package(dynamicEDT3D REQUIRED)

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp nodelet mrs_lib std_msgs nav_msgs mrs_msgs mrs_subt_planning_lib
  LIBRARIES Pathfinder MrsAstarPlanner
  DEPENDS OCTOMAP
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${dynamicEDT3D_INCLUDE_DIRS}
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
