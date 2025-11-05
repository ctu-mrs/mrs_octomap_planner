
## --------------------------------------------------------------
## |                        Octomap Planner                      |
## --------------------------------------------------------------

cmake_minimum_required(VERSION 3.5)
project(mrs_octomap_planner)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler options
add_compile_options(-O3 -DOCTOMAP_NODEBUGOUT)


# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)
#set(USE_ROS_TIMER 1)


if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[mrs_octomap_planner]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")

  add_definitions("-O1")
  add_definitions("-g")
endif()

# Find dependencies
set(DEPENDENCIES
  rclcpp_components
  mrs_modules_msgs
  mrs_lib
  mrs_msgs
  std_msgs
  std_srvs
  octomap_msgs
  octomap_ros
  geometry_msgs
  nav_msgs
  mrs_subt_planning_lib
  dynamicEDT3D
  PCL
  pcl_conversions
  pcl_ros
)

# set(LIBRARIES
#   MrsOctomapPlanner_OctomapPlanner
#   MrsOctomapPlanner_MinimalOctomapPlanner
#   )

set(LIBRARIES
  MrsOctomapPlanner_OctomapPlanner
  )

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()


include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${mrs_lib_INCLUDE_DIRS}
  ${mrs_modules_msgs_INCLUDE_DIRS}
  ${mrs_msgs_INCLUDE_DIRS}
  # ${mrs_subt_planning_lib_INCLUDE_DIRS}
  ~/ws_mapping/install/mrs_subt_planning_lib/include            ##need to be modified
  )




## --------------------------------------------------------------
## |                           Compile                          |
## --------------------------------------------------------------

add_library(MrsOctomapPlanner_OctomapPlanner SHARED
  src/octomap_planner.cpp
  src/minimal_octomap_planner.cpp
  src/astar_planner.cpp
)

target_link_libraries(MrsOctomapPlanner_OctomapPlanner
  mrs_subt_planning_lib
)

ament_target_dependencies(MrsOctomapPlanner_OctomapPlanner
  ${DEPENDENCIES}
)


## components
rclcpp_components_register_nodes(MrsOctomapPlanner_OctomapPlanner PLUGIN "mrs_octomap_planner::OctomapPlanner" EXECUTABLE MrsOctomapPlanner_OctomapPlanner)

target_compile_definitions(MrsOctomapPlanner_OctomapPlanner PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------


ament_export_libraries(
  ${LIBRARIES}
  )

install(
  TARGETS ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )


install(DIRECTORY include/
  DESTINATION .)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  )


install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()