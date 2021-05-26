cmake_minimum_required(VERSION 2.8.3)
project(pathfinder)

find_package(catkin REQUIRED
  roscpp
  std_msgs
  nav_msgs
  mrs_lib
  )

find_package(octomap REQUIRED)
find_package(dynamicEDT3D REQUIRED)

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs nav_msgs mrs_lib mrs_msgs
  LIBRARIES astar_planner
  )

set(CMAKE_CXX_FLAGS "-std=c++1z -O3")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${octomap_INCLUDE_DIRS}
  )

add_library(astar_planner src/astar_planner.cpp)
add_dependencies(astar_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(astar_planner ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${DYNAMICEDT3D_LIBRARIES})

add_executable(pathfinder src/pathfinder.cpp)
add_dependencies(pathfinder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pathfinder astar_planner ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${DYNAMICEDT3D_LIBRARIES})
