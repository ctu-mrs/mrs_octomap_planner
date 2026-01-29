## --------------------------------------------------------------
## |                        Octomap Planner                      |
## --------------------------------------------------------------

cmake_minimum_required(VERSION 3.20)
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
  rclcpp
  rclcpp_components
  mrs_lib
  mrs_msgs
  mrs_modules_msgs
  mrs_subt_planning_lib
  nav_msgs
  octomap_msgs
  octomap_ros
  std_msgs
  geometry_msgs
  dynamicEDT3D
)

set(LIBRARIES
  MrsOctomapPlanner_OctomapPlanner
  MrsOctomapPlanner_MinimalOctomapPlanner
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

## --------------------------------------------------------------
## |                           Compile                          |
## --------------------------------------------------------------

function(add_mrs_library target source)
  add_library(${target} SHARED
    ${source}
    src/astar_planner.cpp
  )

  target_link_libraries(${target}
    dynamicedt3d
  )

  ament_target_dependencies(${target}
    ${DEPENDENCIES}
  )

  target_include_directories(${target}
    PUBLIC
      $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
  )

  target_compile_definitions(${target}
    PRIVATE USE_ROS_TIMER=$<BOOL:${USE_ROS_TIMER}>
  )
endfunction()

add_mrs_library(
  MrsOctomapPlanner_OctomapPlanner
  src/octomap_planner.cpp
)

rclcpp_components_register_nodes(
  MrsOctomapPlanner_OctomapPlanner
  PLUGIN "mrs_octomap_planner::OctomapPlanner"
)

add_mrs_library(
  MrsOctomapPlanner_MinimalOctomapPlanner
  src/minimal_octomap_planner.cpp
)

rclcpp_components_register_nodes(
  MrsOctomapPlanner_MinimalOctomapPlanner
  PLUGIN "mrs_octomap_planner::MinimalOctomapPlanner"
)

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

ament_export_libraries(
  ${LIBRARIES}
)

install(TARGETS ${LIBRARIES} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION .
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
