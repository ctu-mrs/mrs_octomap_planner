cmake_minimum_required(VERSION 3.16)
project(mrs_octomap_planner)

add_definitions(-DOCTOMAP_NODEBUGOUT)
add_definitions(-O3)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# disables anoying googletest warning
set(CATKIN_ENABLE_TESTING FALSE)

# disables anoying PCL warnings
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")

set(CATKIN_DEPENDENCIES
  cmake_modules
  mrs_lib
  mrs_msgs
  mrs_modules_msgs
  mrs_octomap_tools
  mrs_subt_planning_lib
  nav_msgs
  nodelet
  octomap_msgs
  octomap_ros
  roscpp
  rospy
  std_msgs
  )

set(LIBRARIES
  MrsOctomapPlanner_OctomapPlanner
  )

find_package(catkin REQUIRED
  ${CATKIN_DEPENDENCIES}
  )

find_package(octomap REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)

find_package(dynamicEDT3D REQUIRED)

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  LIBRARIES ${LIBRARIES}
  DEPENDS OCTOMAP
  )

include_directories(SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${dynamicEDT3D_INCLUDE_DIRS}
  )

# Octomap Planner

add_library(MrsOctomapPlanner_OctomapPlanner
  src/octomap_planner.cpp
  src/astar_planner.cpp
  )

add_dependencies(MrsOctomapPlanner_OctomapPlanner
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsOctomapPlanner_OctomapPlanner
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${DYNAMICEDT3D_LIBRARIES}
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
